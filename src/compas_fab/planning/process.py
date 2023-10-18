from compas.data import Data
from compas_fab.robots import Tool  # noqa: F401


try:
    from compas_fab.planning import Action, Workpiece, SceneState  # noqa: F401
except ImportError:
    from .action import Action  # noqa: F401
    from .workpiece import Workpiece  # noqa: F401
    from .state import SceneState  # noqa: F401

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
        data['workpieces'] = self.workpieces
        data['tool'] = self.tool
        data['assembly_sequence'] = self.assembly_sequence
        data['actions'] = self.actions
        return data

    @data.setter
    def data(self, data):
        self.workpieces = data.get('workpieces', self.workpieces)
        self.tool = data.get('tool', self.tool)
        self.assembly_sequence = data.get('assembly_sequence', self.assembly_sequence)
        self.actions = data.get('actions', self.actions)

    @property
    def workpiece_ids(self):
        """List of workpiece ids in the process."""
        return list(self.workpieces.keys())

    @property
    def tool_id(self):
        """The id of the tool used in the process."""
        return self.tool.name

    def get_initial_state(self):
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
                workpiece_id, Frame.worldXY())

        # Set initial state of tool (the tool by default is attached to robot flange directly)
        scene_state.tool_states[self.tool_id].frame = self.robot_initial_frame
        scene_state.tool_states[self.tool_id].attached_to_robot = True
        scene_state.tool_states[self.tool_id].attached_to_robot_grasp = Transformation()

        # Set initial state of robot (the initial frame and configuration is set manually by user)
        scene_state.robot_state.frame = self.robot_initial_frame
        scene_state.robot_state.configuration = self.robot_initial_configuration

        return scene_state

    def get_intermediate_state(self, state_index, debug=False):
        """Parses the actions in the process and returns an intermediate state of the process.

        Parameters
        ----------
        state_index : int
            The index of state to return. index 0 is equal to the initial state.
            The index must be smaller than or equal to the number of actions in the process.
        """
        assert state_index <= len(
            self.actions), "state_index must be smaller than or equal to the number of actions in the process."
        scene_state = self.get_initial_state()
        for action in self.actions[:state_index]:
            action.apply_to(scene_state, debug=debug)
        return scene_state
