from compas.data import Data
from compas.robots import Configuration, RobotModel  # noqa: F401
from compas.datastructures import Mesh  # noqa: F401
from compas_fab.robots import Tool  # noqa: F401


try:
    from compas_fab.planning import (
        Action,
        Workpiece,
        SceneState,
        RoboticMovement,
        LinearMovement,
        FreeMovement,
    )  # noqa: F401
except ImportError:
    from .action import Action, RoboticMovement, LinearMovement, FreeMovement  # noqa: F401
    from .workpiece import Workpiece  # noqa: F401
    from .state import SceneState  # noqa: F401

try:
    from typing import Optional, List, Type  # noqa: F401
except ImportError:
    pass

from compas.geometry import Frame, Transformation

__all__ = [
    "AssemblyProcess",
]


# class WorkpieceLocation(Data):
#     """Class for describing the locations of a workpiece in the assembly process.
#     Supports three locations typically find in an assembly process:
#     'pickup', 'before_approach', 'assembled'.
#     """

#     def __init__(self):
#         super(WorkpieceLocation, self).__init__()
#         self.pickup_frame = None  # type: Frame
#         self.before_approach_frame = None  # type: Frame
#         self.assembled_frame = None  # type: Frame

#     @property
#     def data(self):
#         data = {}
#         data['pickup_frame'] = self.pickup_frame
#         data['before_approach_frame'] = self.before_approach_frame
#         data['assembled_frame'] = self.assembled_frame
#         return data

#     @data.setter
#     def data(self, data):
#         self.pickup_frame = data.get('pickup_frame', self.pickup_frame)
#         self.before_approach_frame = data.get('before_approach_frame', self.before_approach_frame)
#         self.assembled_frame = data.get('assembled_frame', self.assembled_frame)


class AssemblyProcess(Data):
    """Base class for an Assembly Process.
    Suppoorts only one tool in the process, hence no tool change is supported.

    Attributes
    ----------
    workpieces : dict[str, Workpiece]
        Dictionary of workpieces used in the process.
        The keys are the workpiece id.
    tool : :class:`compas_fab.robots.Tool`
        The tool used in the process.

    assembly_sequence : list[str]
        List of workpiece ids in the order they are assembled.
    actions : list[Action]
        List of actions in the process.
    """

    def __init__(self):
        super(AssemblyProcess, self).__init__()
        self.workpieces = {}  # type: dict[str, Workpiece]
        self.tool = None  # type: Tool
        self._robot_model = None  # type: RobotModel
        self.static_collision_meshes = {}  # type: dict[str, Mesh]
        self.assembly_sequence = []  # type: list[str]
        self.actions = []  # type: list[Action]

        # TODO: Consider refactoring this to a WorkpieceLocation class
        # Store the frames of the workpieces
        self.workpiece_storage_frame = {}
        self.workpiece_before_approach_frame = {}
        self.workpiece_assembled_frame = {}

        # TODO:Consider generializing this to support multiple tools

        # Store the initial frame of the robot
        self.robot_initial_frame = Frame.worldXY()
        self.robot_initial_configuration = None

    @property
    def data(self):
        data = {}
        data["workpieces"] = self.workpieces
        data["tool"] = self.tool
        data["assembly_sequence"] = self.assembly_sequence
        data["actions"] = self.actions
        data["workpiece_storage_frame"] = self.workpiece_storage_frame
        data["workpiece_before_approach_frame"] = self.workpiece_before_approach_frame
        data["workpiece_assembled_frame"] = self.workpiece_assembled_frame
        data["robot_initial_frame"] = self.robot_initial_frame
        data["robot_initial_configuration"] = self.robot_initial_configuration
        return data

    @data.setter
    def data(self, data):
        self.workpieces = data.get("workpieces", self.workpieces)
        self.tool = data.get("tool", self.tool)
        self.assembly_sequence = data.get("assembly_sequence", self.assembly_sequence)
        self.actions = data.get("actions", self.actions)
        self.workpiece_storage_frame = data.get("workpiece_storage_frame", self.workpiece_storage_frame)
        self.workpiece_before_approach_frame = data.get(
            "workpiece_before_approach_frame", self.workpiece_before_approach_frame
        )
        self.workpiece_assembled_frame = data.get("workpiece_assembled_frame", self.workpiece_assembled_frame)
        self.robot_initial_frame = data.get("robot_initial_frame", self.robot_initial_frame)
        self.robot_initial_configuration = data.get("robot_initial_configuration", self.robot_initial_configuration)

    @property
    def workpiece_ids(self):
        """List of workpiece ids in the process."""
        return list(self.workpieces.keys())

    @property
    def tool_id(self):
        """The id of the tool used in the process."""
        return self.tool.name

    def get_initial_state(self):
        # type: () -> SceneState
        """Returns the initial state of the process.

        Returns
        -------
        :class:`compas_fab.planning.SceneState`
            The initial state of the process.
        """
        scene_state = SceneState(self.workpiece_ids, [self.tool_id])

        # Set initial state of workpieces (the workpieces by default are stored in the world XY plane)
        for workpiece_id in self.workpiece_ids:
            scene_state.workpiece_states[workpiece_id].frame = self.workpiece_storage_frame.get(
                workpiece_id, Frame.worldXY()
            )

        # Set initial state of tool (the tool by default is attached to robot flange directly)
        scene_state.tool_states[self.tool_id].frame = self.robot_initial_frame
        scene_state.tool_states[self.tool_id].attached_to_robot = True
        scene_state.tool_states[self.tool_id].attached_to_robot_grasp = Transformation()

        # Set initial state of robot (the initial frame and configuration is set manually by user)
        scene_state.robot_state.frame = self.robot_initial_frame
        scene_state.robot_state.configuration = self.robot_initial_configuration

        return scene_state

    def get_intermediate_state(self, state_index, debug=False):
        # type: (int, bool) -> SceneState
        """Parses the actions in the process and returns an intermediate state of the process.

        Parameters
        ----------
        state_index : int
            The index of state to return. index 0 is equal to the initial state.
            The index must be smaller than or equal to the number of actions in the process.
        """
        assert state_index <= len(
            self.actions
        ), "state_index must be smaller than or equal to the number of actions in the process."
        scene_state = self.get_initial_state()
        for action in self.actions[:state_index]:
            action.apply_to(scene_state, debug=debug)
        return scene_state

    def get_robotic_actions(self):
        # type: () -> list[RoboticMovement]
        """Returns the robotic actions in the process."""
        return [action for action in self.actions if isinstance(action, RoboticMovement)]

    def get_next_robotic_action(self, action, type=None):
        # type: (Action, Type) -> Optional[RoboticMovement]
        """Returns the next robotic action in the process.
        If type is specified, returns the next robotic action of the specified type.

        Parameters
        ----------
        action : :class:`compas_fab.planning.Action`
            The action to query.
        type : type, optional
            The type of the next robotic action to query.
            :class:`compas_fab.planning.FreeMovement` or `compas_fab.planning.LinearMovement`.
        """
        robotic_actions = self.get_robotic_actions()
        action_index = robotic_actions.index(action)
        while action_index < len(robotic_actions) - 2:
            action_index += 1
            next_action = robotic_actions[action_index]
            if type is None or isinstance(next_action, type):
                return next_action
        return None

    def get_previous_robotic_action(self, action, type=None):
        # type: (Action, Type) -> Optional[RoboticMovement]
        """Returns the previous robotic action in the process.
        If type is specified, returns the next robotic action of the specified type.

        Parameters
        ----------
        action : :class:`compas_fab.planning.Action`
            The action to query.
        type : type, optional
            The type of the next robotic action to query.
            :class:`compas_fab.planning.FreeMovement` or `compas_fab.planning.LinearMovement`.
        """
        robotic_actions = self.get_robotic_actions()
        action_index = robotic_actions.index(action)
        while action_index > 0:
            action_index -= 1
            prev_action = robotic_actions[action_index]
            if type is None or isinstance(prev_action, type):
                return prev_action
        return None

    def get_action_starting_configuration(self, action):
        # type: (RoboticMovement) -> Optional[Configuration]
        """Returns the starting configuration of an action.
        Returns None if the action does not have a starting configuration.

        Cases where the action has a starting configuration are:
        - this action is the first robotic movement in the process
            (return the initial robotic configuration)
        - the previous robotic movement has a planned tracjectory
            (return the last trajectory point of the planned trajectory)
        - the previous robotic movement has a fixed configuration
            (return the fixed configuration)

        Parameters
        ----------
        action : :class:`compas_fab.planning.RoboticMovement`
            The robotic movement action to query.
        """
        previous_action = self.get_previous_robotic_action(action)
        if previous_action is None:
            return self.robot_initial_configuration
        elif previous_action.planned_trajectory is not None:
            return previous_action.planned_trajectory.points[-1]
        elif previous_action.fixed_configuration is not None:
            return previous_action.fixed_configuration
        else:
            return None

    def get_action_ending_configuration(self, action):
        # type: (RoboticMovement) -> Optional[Configuration]
        """Returns the ending configuration of an action.
        Returns None if the action does not have an ending configuration.

        Cases where the action has an ending configuration are:
        - this robotic movement has a fixed configuration
            (return the fixed configuration)
        - the next robotic movement has a planned trajectory
            (return the first trajectory point of the planned trajectory)

        Parameters
        ----------
        action : :class:`compas_fab.planning.RoboticMovement`
            The robotic movement action to query.
        """
        next_action = self.get_next_robotic_action(action)
        if action.fixed_configuration is not None:
            return action.fixed_configuration
        elif next_action is not None and next_action.planned_trajectory is not None:
            return next_action.planned_trajectory.points[0]
        else:
            return None

    def get_movement_groups(self):
        # type: () -> List[List[RoboticMovement]]
        """Returns the movement groups of the process.
        Each group contains a list of ordered robotic movements of the same type.
        """
        movement_groups = []
        robotic_actions = self.get_robotic_actions()

        # Bootstrap the first group
        last_movement_type = type(robotic_actions[0])
        movement_group = [robotic_actions[0]]

        for action in robotic_actions[1:]:
            if isinstance(action, last_movement_type):
                movement_group.append(action)
            else:
                # New movement group
                movement_groups.append(movement_group)
                movement_group = [action]
            last_movement_type = type(action)
        # Add the last movement group
        movement_groups.append(movement_group)
        return movement_groups

    def get_movement_group(self, action):
        # type: (RoboticMovement, type) -> List[RoboticMovement]
        """Returns the entire movement group of an RoboticMovement action
        returns a list of ordered RoboticMovement that the given movement resides"""
        assert isinstance(action, RoboticMovement)
        for movement_group in self.get_movement_groups():
            if action in movement_group:
                return movement_group

    def get_linear_movement_groups(self):
        # type: () -> List[List[LinearMovement]]
        """Returns the linear movement groups of the process.
        Each group contains a list of ordered linear movements.
        """
        movement_groups = []
        for movement_group in self.get_movement_groups():
            if isinstance(movement_group[0], LinearMovement):
                movement_groups.append(movement_group)
        return movement_groups

    def get_free_movement_groups(self):
        # type: () -> List[List[FreeMovement]]
        """Returns the free movement groups of the process.
        Each group contains a list of ordered free movements.
        """
        movement_groups = []
        for movement_group in self.get_movement_groups():
            if isinstance(movement_group[0], FreeMovement):
                movement_groups.append(movement_group)
        return movement_groups

    def get_linear_movement_group(self, action):
        # type: (LinearMovement) -> List[LinearMovement]
        """Returns the linear movement group of an LinearMovement action.

        Parameters
        ----------
        action : :class:`compas_fab.planning.LinearMovement`
            The linear movement action to query.
        """
        assert isinstance(action, LinearMovement)
        return self.get_movement_group(action)

    def get_free_movement_group(self, action):
        # type: (FreeMovement) -> List[FreeMovement]
        """Returns the free movement group of an FreeMovement action.

        Parameters
        ----------
        action : :class:`compas_fab.planning.FreeMovement`
            The free movement action to query.
        """
        assert isinstance(action, FreeMovement)
        return self.get_movement_group(action)

    def get_prev_movement_group(self, action):
        # type: (RoboticMovement) -> List[RoboticMovement]
        action_type = type(action)
        if action_type == LinearMovement:
            prev_group_type = FreeMovement
        else:
            prev_group_type = LinearMovement
        prev_movement = self.get_previous_robotic_action(action, prev_group_type)
        if prev_movement is not None:
            return self.get_movement_group(prev_movement)
        else:
            return []

    def get_next_movement_group(self, action):
        # type: (RoboticMovement) -> List[RoboticMovement]
        action_type = type(action)
        if action_type == LinearMovement:
            next_group_type = FreeMovement
        else:
            next_group_type = LinearMovement
        next_movement = self.get_next_robotic_action(action, next_group_type)
        if next_movement is not None:
            return self.get_movement_group(next_movement)
        else:
            return []
