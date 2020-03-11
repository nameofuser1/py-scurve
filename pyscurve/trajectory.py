import numpy as np
import logging
from matplotlib import pyplot as plt


ACCELERATION_ID = 0
SPEED_ID = 1
POSITION_ID = 2

OPTIMIZER_THRESHOLD = 0.01
EPSILON = 0.0001

trajectory_logger = logging.getLogger("trajectory_logger")


class PlanningError(Exception):

    def __init__(self, msg):
        super(Exception, self).__init__(msg)


class Trajectory(object):

    def __init__(self, debug=True):
        self._debug = debug
        self._trajectory = None
        self._time = 0
        self._dof = 0
        self._p_logged = 0

    @property
    def debug(self):
        return self._debug

    @debug.setter
    def debug(self, v):
        self._debug = v

    @property
    def time(self):
        return self._time

    @time.setter
    def time(self, v):
        self._time = v

    @property
    def dof(self):
        return self._dof

    @dof.setter
    def dof(self, v):
        self._dof = v

    @property
    def trajectory(self):
        return self._trajectory

    @trajectory.setter
    def trajectory(self, v):
        self._trajectory = v

    def __call__(self, time):
        point = np.zeros((self.dof, 3), dtype=np.float32)
        for t, dof in zip(self.trajectory, range(self.dof)):
            dof_point = t(time)
            np.put(point[dof], range(3), dof_point)

            if self.debug:
                trajectory_logger.debug(
                    "DOF {} point number: {}: {}:{}:{}".format(dof,
                                                               self._p_logged,
                                                               *point[dof]))
            self._p_logged += 1
        return point


def plot_trajectory(traj, dt):
    dof = traj.dof
    timesteps = int(max(traj.time) / dt)
    time = np.linspace(0, max(traj.time), timesteps)

    # NOW
    # profiles[t]           --- profiles for each DOF at time x[t]
    # profiles[t][d]        --- profile for d DOF at time x[t]
    # profiles[t][d][k]     --- accel/vel/pos profile for d DOF at time x[t]
    p_list = [traj(t) for t in time]
    profiles = np.asarray(p_list)

    # NEED
    # profiles[d]       --- profiles for each DOF 0 <= d <= DOF number
    # profiles[d][k]    --- accel/vel/pos profile for DOF d where j
    # profiles[d][k][t] --- accel/vel/pos at time x[k] for DOF i
    # profiles = np.reshape(profiles, (dof, 3, timesteps))
    r_profiles = np.zeros((dof, 3, timesteps))
    for d in range(dof):
        for p in range(3):
            r_profiles[d, p, :] = profiles[:, d, p]

    fig = plt.figure(0)
    # fig.suptitle("DOF profiles")

    for i, profile in zip(range(dof), r_profiles):
        plt.subplot(300 + dof*10 + (i+1))
        plt.title("Acceleration profile")
        plt.plot(time, profile[ACCELERATION_ID][:])
        plt.xlim()
        plt.ylim()

        plt.subplot(300 + dof*10 + (i+1)+dof)
        plt.title("Speed profile")
        plt.plot(time, profile[SPEED_ID][:])
        plt.xlim()
        plt.ylim()

        plt.subplot(300 + dof*10 + (i+1)+dof*2)
        plt.title("Position profile")
        plt.plot(time, profile[POSITION_ID][:])
        plt.xlim()
        plt.ylim()

    plt.tight_layout()
    plt.show()
