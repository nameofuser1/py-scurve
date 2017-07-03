import numpy as np
from matplotlib import pyplot as plt


ACCELERATION_ID = 0
SPEED_ID = 1
POSITION_ID = 2


class PlanningError(Exception):

    def __init__(self, msg):
        super(Exception, self).__init__(msg)


def check_profile_params(x, y, v0, a0, a_max, v_max):
    if a_max is None and v_max is None:
        raise ValueError("Please specify either a_max or v_max")

    res = len(x) == len(y) == len(v0) == len(a0) == len(a_max) == len(v_max)

    if not res:
        raise ValueError("Number of dimentions in parameters does not match")

    return len(x)


def trap_plan_with_acceleration(x, y, a0, v_max, a_max):
    pass


def trap_plan_with_time(x, y, v0, a0, v_max, a_max):
    pass


def trapezoidal_profile(x, y, v0, a0, v_max, a_max):
    """
    x, y    --- lists containing source and destination points
    T       --- execution time
    v0, a0  --- lists containing initial conditions(end velocity/acceleration
                    will be the same)
    v_max, a_max    --- max acceleration/velocity
    """
    dof = check_profile_params(x, y, v0, a0, v_max, a_max)

    T = []
    TA = []
    TC = []
    SA = []
    SC = []

    for i in range(dof):
        ta = (v_max[i] - v0[i] + 0.0)/(a_max[i] + 0.0)
        sa = a_max[i]*(ta**2)/2. + (v0[i]*ta)

        if y[i] - x[i] < 2*sa:
            raise PlanningError("Failed to plan trapezoidal profile")

        t = (y[i] - x[i] - a_max[i]*(ta**2) - 2.*v0[i]*ta)/v_max[i] + 2.*ta
        tc = t - 2.*ta
        sc = tc*v_max[i]+0.0

        T.append(t)
        TA.append(ta)
        TC.append(tc)
        SA.append(sa)
        SC.append(sc)

    print("T: " + str(T))
    print("TA: " + str(TA))
    print("TC: " + str(TC))
    print("SA: " + str(SA))
    print("SC: " + str(SC))

    def trajectory(_t):
        """
        Returns 3 points for each DOF: acceleration, speed, position at time _t
        """
        if _t < 0:
            raise ValueError("Time must be positive number")

        point = np.zeros((dof, 3))

        for i, t, ta, tc, sa, sc in zip(range(dof), T, TA, TC, SA, SC):
            if _t == 0:
                point[i][ACCELERATION_ID] = a0[i]
                point[i][SPEED_ID] = v0[i]
                point[i][POSITION_ID] = x[i]
            elif 0 < _t <= ta:
                point[i][ACCELERATION_ID] = a_max[i]
                point[i][SPEED_ID] = a_max[i]*_t + v0[i]
                point[i][POSITION_ID] = x[i] + a_max[i]*(_t**2)/2 + v0[i]*_t

            elif ta <= _t < ta+tc:
                point[i][ACCELERATION_ID] = 0
                point[i][SPEED_ID] = v_max[i]
                point[i][POSITION_ID] = x[i] + sa + v_max[i]*(_t - ta)

            elif ta+tc <= _t < t:
                point[i][ACCELERATION_ID] = -a_max[i]
                point[i][SPEED_ID] = v_max[i] - a_max[i]*(_t - ta - tc)
                point[i][POSITION_ID] = x[i] + sa+sc\
                    - a_max[i]*((_t-ta-tc)**2)/2 + v_max[i]*(_t-ta-tc)

            elif _t == t:
                point[i][ACCELERATION_ID] = a0[i]
                point[i][SPEED_ID] = v0[i]
                point[i][POSITION_ID] = y[i]

            else:
                raise ValueError("Time exceeds limit")

        return point

    return dof, T, trajectory


def plot_trajectory(traj, dt):
    timesteps = traj.time[0] / dt
    dof = traj.dof

    time = np.linspace(0, traj.time[0], timesteps)
    # NOW
    # profiles[t]           --- profiles for each DOF at time x[t]
    # profiles[t][d]        --- profile for d DOF at time x[t]
    # profiles[t][d][k]     --- accel/vel/pos profile for d DOF at time x[t]
    profiles = np.asarray(map(traj.trajectory, time))

    # NEED
    # profiles[d]       --- profiles for each DOF 0 <= d <= DOF number
    # profiles[d][k]    --- accel/vel/pos profile for DOF d where j
    # profiles[d][k][t] --- accel/vel/pos at time x[k] for DOF i
    # profiles = np.reshape(profiles, (dof, 3, timesteps))
    r_profiles = np.zeros((dof, 3, timesteps))
    for d in range(dof):
        for p in range(3):
            r_profiles[d, p, :] = profiles[:, d, p]

    for i, profile in zip(range(dof), r_profiles):
        fig = plt.figure(i)
        fig.suptitle("DOF %d profiles" % i)

        plt.subplot(311)
        plt.title("Acceleration profile")
        plt.plot(time, profile[ACCELERATION_ID][:])

        plt.subplot(312)
        plt.title("Speed profile")
        plt.plot(time, profile[SPEED_ID][:])

        plt.subplot(313)
        plt.title("Position profile")
        plt.plot(time, profile[POSITION_ID][:])

        plt.tight_layout()
        plt.show()


class Trajectory(object):

    def __init__(self):
        self._trajectory = None
        self._time = 0
        self._dof = 0

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
        return self.trajectory(time)


if __name__ == "__main__":
    src = [1+0.0]
    dst = [6+0.0]
    a0 = [1+0.0]
    v0 = [1+0.0]
    a_max = [2+0.0]
    v_max = [3+0.0]

    try:
        trajectory = Trajectory.plan_trajectory(src, dst, v0, a0, v_max, a_max)
        plot_trajectory(trajectory, 0.01)
    except PlanningError as e:
        print(e)
