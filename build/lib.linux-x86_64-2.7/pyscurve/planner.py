from abc import ABCMeta, abstractmethod


class TrajectoryPlanner(object):
    __metaclass__ = ABCMeta

    def __init__(self):
        pass

    @abstractmethod
    def plan_trajectory(self):
        pass

    def _check_shape(self, *args):
        sh = len(args[0])

        for arg in args:
            if sh != len(arg):
                raise ValueError("All parameters must have the same dimention")

        return (sh,)
