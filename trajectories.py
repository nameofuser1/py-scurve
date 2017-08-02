import numpy as np
from matplotlib import pyplot as plt
from constant_time_optimizer import optimize_trajectory


ACCELERATION_ID = 0
SPEED_ID = 1
POSITION_ID = 2

OPTIMIZER_THRESHOLD = 0.01
EPSILON = 0.1


class PlanningError(Exception):

    def __init__(self, msg):
        super(Exception, self).__init__(msg)


class Trajectory(object):

    def __init__(self):
        self._trajectory = None
        self._time = 0
        self._dof = 0
        self._p_logged = 0

    @staticmethod
    def plan_trajectory(x, y, v0, a0, v_max, a_max):
        dof, T, trajectory_f = trapezoidal_profile(x, y, v0, a0,
                                                   v_max, a_max)

        traj = Trajectory()
        traj.time = T
        traj.trajectory = trajectory_f
        traj.dof = dof

        return traj

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
        point =  np.zeros((self.dof, 3), dtype=np.float32)
        for t, dof in zip(self.trajectory, range(self.dof)):
            dof_point = t(time)
            np.put(point[dof], range(3), dof_point)

            if self._p_logged < 20:
                print("DOF {} point number: {}: {}:{}:{}".format(dof,
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
    print(p_list[-10:])
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

    print(profiles.shape)
    print(r_profiles.shape)
    print(profiles[-10:, :, :])
    print(r_profiles[:, :, -10:])

    fig = plt.figure(0)
    fig.suptitle("DOF profiles")

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




if __name__ == "__main__":
    src = [1+0.0, 0.0]
    dst = [2+0.0, 15.0]
    a0 = [0+0.0, 0.]
    v0 = [0+0.0, 0.]
    a_max = [2+0.0, 3.]
    v_max = [3+0.0, 5.]

    try:
        trajectory = Trajectory.plan_trajectory(src, dst, v0, a0, v_max, a_max)
        plot_trajectory(trajectory, 0.01)
    except PlanningError as e:
        print(e)
