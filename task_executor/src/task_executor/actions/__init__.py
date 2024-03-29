#!/usr/bin/env python3
# Creates the registry of action definitions to actions to use

from __future__ import print_function

from task_executor.abstract_step import AbstractStep

from .move import MoveAction
from .navigation import Navigation
from .get_beliefs import GetBeliefsAction
from .move_planar import MovePlanarAction
from .wait import WaitAction
from .reposition import RepositionAction
from .segmentation import SegmentationAction
from .switch_mode import SwitchModeAction
from .grasp import GraspAction
from .pick import Pick
from .reposition_cam import RepositionCam
from .place import Place

class Actions(object):
    """
    A registry of actions. It is recommended that you create this object with
    :func:`get_default_actions`. In order to use the actions, they must be
    intialized, which includes connecting to their action servers, services,
    etc. You can use the :attr:`initialized` attribute of this object to know
    if the actions are initialized or not.
    """

    def __init__(self, registry):
        """
        Args:
            registry (dict) : This is a dict of name -> Action class mappings
        """
        self.registry = { key: klass() for key, klass in registry.items() }

        # Flag for if the actions are initialized
        self.initialized = False

        # Quick sanity check because I don't trust people. Also set the action
        # as an attribute for '.' based referencing
        for key, action in self.registry.items():
            assert isinstance(action, AbstractStep)
            setattr(self, key, action)

    def __getitem__(self, key):
        return self.registry[key]

    def init(self):
        for key, action in self.registry.items():
            action.init(key)

        # Mark the actions as initialized
        self.initialized = True


# The default actions contain all the action interfaces that are known to this
# package
default_actions_dict = {
    'move': Navigation,
    'get_beliefs': GetBeliefsAction,
    'move_planar': MovePlanarAction,
    'wait': WaitAction,
    'reposition': RepositionAction,
    'switch_mode': SwitchModeAction,
    'segmentation': SegmentationAction,
    'grasp': GraspAction,
    'pick': Pick,
    'reposition_cam': RepositionCam,
    'place': Place
}

def get_default_actions():
    """
    Provides a consistent manner of fetching all the actions that are available
    on the robot. This is the preferred manner of getting the actions:

    >>> actions = get_default_actions()
    >>> move_params = { 'location': 'waypoints.origin' }
    >>> actions.move(**move_params)

    Returns:
        (Actions) : The default registry of all the actions that are available
    """
    return Actions(default_actions_dict)