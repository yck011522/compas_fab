"""
********************************************************************************
compas_fab.ghpython
********************************************************************************

.. currentmodule:: compas_fab.ghpython

Artists
-------

In **COMPAS**, the `artists` are classes that assist with the visualization of
datastructures and models, in a way that maintains the data separated from the
specific CAD interfaces, while providing a way to leverage native performance
of the CAD environment.

.. autosummary::
    :toctree: generated/
    :nosignatures:

    ReachabilityMapArtist

Additionally, this package provides deprecated aliases to the
:class:`~compas_ghpython.artists.RobotModelArtist` implemented in COMPAS.


"""
import compas

if compas.RHINO:
    from .artists import (
        ReachabilityMapArtist,
        RobotArtist,
        RobotModelArtist,
    )

    from .path_planning import *  # noqa: F401,F403

    __all__ = [
        "ReachabilityMapArtist",
        "RobotArtist",  # deprecated
        "RobotModelArtist",  # deprecated
    ]
