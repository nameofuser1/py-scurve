from trajectories import Trajectory, PlanningError, ACCELERATION_ID
from trajectories import SPEED_ID, POSITION_ID



def check_profile_params(x, y, v0, a0, a_max, v_max):
    if a_max is None and v_max is None:
        raise ValueError("Please specify either a_max or v_max")

    res = len(x) == len(y) == len(v0) == len(a0) == len(a_max) == len(v_max)

    if not res:
        raise ValueError("Number of dimentions in parameters does not match")

    return len(x)


def trapezoidal_planner(x, y, v0, a0, v_max, a_max):
    """
    x, y    --- lists containing source and destination points
    T       --- execution time
    v0, a0  --- lists containing initial conditions(end velocity/acceleration
                    will be the same)
    v_max, a_max    --- max acceleration/velocity

    Returns trajectory parameters:
        t  ---  time trajectory execution takes
        ta ---  time of acceleration/deceleration segments
        tc ---  constant speed segment time
        sa ---  length of acceleration segment
        sc ---  length of constant speed segment
        vc ---  speed at constant speed segment(or maximum speed achieved)
    """
    ta = (v_max - v0)/(a_max)
    sa = a_max*(ta**2)/2. + (v0*ta)

    if y - x < 2*sa:
        raise PlanningError("Failed to plan trapezoidal profile")

    t = (y - x - a_max*(ta**2) - 2.*v0*ta)/v_max + 2.*ta
    tc = t - 2.*ta
    sc = tc*v_max+0.0
    vc = v_max
    a = a_max

    return (t, ta, tc, sa, sc, vc, a)


def minimum_time_planner(x, y, v0, a0, v_max, a_max):
    """
    x, y    --- lists containing source and destination points
    T       --- execution time
    v0, a0  --- lists containing initial conditions(end velocity/acceleration
                    will be the same)
    v_max, a_max    --- max acceleration/velocity

    Returns trajectory parameters:
        t  ---  time trajectory execution takes
        ta ---  time of acceleration/deceleration segments
        tc ---  constant speed segment time
        sa ---  length of acceleration segment
        sc ---  length of constant speed segment
        vc ---  speed at constant speed segment(or maximum speed achieved)
    """
    a = a_max
    D = np.sqrt((v0**2) + (y-x)*a)
    denom = (a/2.)

    t1 = (-v0 + D)/denom
    t2 = (-v0 - D)/denom

    t = max(t1, t2)
    ta = 0.5*t
    tc = 0
    sa = v0*ta + a*(ta**2)/2.
    sc = 0
    vc = v0 + a*ta
    a = a_max

    if vc > v_max:
        raise PlanningError("Maximum speed exceeds bound")

    return (t, ta, tc, sa, sc, vc, a)


def constant_time_planner(x, y, v0, a0, v_max, a_max, t):
    S = y-x
    result = optimize_trajectory(S, v0, a_max, t)

    a = result[0]
    ta = result[1]
    tc = abs(t - 2*ta)
    vc = v0 + a*ta
    sa = v0*ta + a*(ta**2)/2.
    sc = tc*vc

    if abs(S - 2*sa + sc) > OPTIMIZER_THRESHOLD:
        raise PlanningError("Failed  to optimize_trajectory.")

    return (t, ta, tc, sa, sc, vc, a)


def trapezoidal_profile(x, y, v0, a0, v_max, a_max):
    """
    x, y    --- lists containing source and destination points
    T       --- execution time
    v0, a0  --- lists containing initial conditions(end velocity/acceleration
                    will be the same)
    v_max, a_max    --- max acceleration/velocity

    Returns number of DOF, list of times each DOF trajectory takes to be
        executed and function of time which returns desired acceleration,
        speed and position at time t.
    """
    dof = check_profile_params(x, y, v0, a0, v_max, a_max)

    T = []
    TA = []
    TC = []
    SA = []
    SC = []
    VC = []
    A = []

    trajectory_params = [T, TA, TC, SA, SC, VC, A]

    for i in range(dof):
        try:
            params = minimum_time_planner(x[i], y[i], v0[i],
                                          a0[i], v_max[i], a_max[i])
        except PlanningError:
            print("Failed to plan minimum time profile")
            params = trapezoidal_planner(x[i], y[i], v0[i],
                                         a0[i], v_max[i], a_max[i])

        for p_list, p in zip(trajectory_params, params):
            p_list.append(p)

    print("T: " + str(T))
    print("TA: " + str(TA))
    print("TC: " + str(TC))
    print("SA: " + str(SA))
    print("SC: " + str(SC))
    print("VC: " + str(VC))
    print("A: " + str(A))

    def trajectory(_t):
        """
        Returns 3 points for each DOF: acceleration, speed, position at time _t
        """
        if _t < 0:
            raise ValueError("Time must be positive number")

        point = np.zeros((dof, 3))

        for i, t, ta, tc, sa, sc, vc, a in\
                zip(range(dof), T, TA, TC, SA, SC, VC, A):

            if _t == 0:
                point[i][ACCELERATION_ID] = a0[i]
                point[i][SPEED_ID] = v0[i]
                point[i][POSITION_ID] = x[i]
            elif 0 < _t <= ta:
                point[i][ACCELERATION_ID] = a
                point[i][SPEED_ID] = a*_t + v0[i]
                point[i][POSITION_ID] = x[i] + a*(_t**2)/2 + v0[i]*_t

            elif ta <= _t < ta+tc:
                point[i][ACCELERATION_ID] = 0
                point[i][SPEED_ID] = vc
                point[i][POSITION_ID] = x[i] + sa + vc*(_t - ta)

            elif ta+tc <= _t < t:
                point[i][ACCELERATION_ID] = -a
                point[i][SPEED_ID] = vc - a*(_t - ta - tc)
                point[i][POSITION_ID] = x[i] + sa+sc\
                    - a*((_t-ta-tc)**2)/2 + vc*(_t-ta-tc)

            elif _t >= t:
                point[i][ACCELERATION_ID] = a0[i]
                point[i][SPEED_ID] = v0[i]
                point[i][POSITION_ID] = y[i]

            else:
                raise ValueError("Time exceeds limit")

        return point

    return dof, T, trajectory
