from compas.data import Data
from compas.geometry import Transformation
from compas.datastructures import Mesh  # noqa F401

__all__ = [
    "Workpiece",
]


class Workpiece(Data):
    """Class for describing a workpiece.

    Attributes
    ----------
    mesh : list[Mesh]
        The meshes of the workpiece.
    grasp : :class:`compas.geometry.Transformation`
        The grasp frame of the workpiece.
    """

    def __init__(self):
        super(Workpiece, self).__init__()
        self.mesh = []  # type: list[Mesh]
        self.grasp = Transformation()  # type: Transformation

    @property
    def data(self):
        data = {}
        data['mesh'] = self.mesh
        data['grasp'] = self.grasp
        return data

    @data.setter
    def data(self, data):
        self.mesh = data.get('mesh', self.mesh)
        self.grasp = data.get('grasp', self.grasp)
