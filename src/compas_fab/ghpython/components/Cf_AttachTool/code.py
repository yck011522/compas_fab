"""
Attach a tool to the robot.

COMPAS FAB v0.28.0
"""
from ghpythonlib.componentbase import executingcomponent as component
from compas_rhino.conversions import RhinoMesh
from compas_rhino.conversions import plane_to_compas_frame

from compas.geometry import Frame
from compas_fab.robots import Tool


class AttachToolComponent(component):
    def RunScript(self, robot, visual_mesh, collision_mesh, tcf_plane, group):
        if robot and robot.client and robot.client.is_connected and visual_mesh:
            if not collision_mesh:
                collision_mesh = visual_mesh

            c_visual_mesh = RhinoMesh.from_geometry(visual_mesh).to_compas()
            c_collision_mesh = RhinoMesh.from_geometry(collision_mesh).to_compas()

            if not tcf_plane:
                frame = Frame.worldXY()
            else:
                frame = plane_to_compas_frame(tcf_plane)
            tool = Tool(c_visual_mesh, frame, c_collision_mesh)

            robot.attach_tool(tool, group)

        return robot
