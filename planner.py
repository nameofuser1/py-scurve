from abc import ABCMeta, abstractmethod


class TrajectoryPlanner(object):
    __metaclass__ = ABCMeta

    def __init__(self):
        pass

    @abstractmethod
    def plan_trajectory(self):
        pass
