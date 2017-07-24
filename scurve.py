import numpy as np
from trajectories import PlanningError, EPSILON


def __scurve_check_possibility(q0, q1, v0, v1, v_max, a_max, j_max):
    dv = np.abs(v1 - v0)
    dq = np.abs(q1 - q0)

    time_to_reach_max_a = a_max/j_max
    time_to_set_set_speeds = np.sqrt(dv/j_max)

    Tj = min(time_to_reach_max_a, time_to_set_set_speeds)

    if Tj == time_to_reach_max_a:
        return dq > 0.5*(v0 + v1)*(Tj+dv/a_max)

    elif Tj < time_to_reach_max_a:
        return dq > Tj*(v0+v1)

    else:
        raise PlanningError("Something went wrong")


def __compute_maximum_speed_reached(q0, q1, v0, v1, v_max, a_max, j_max):
    """
    For explanation look at page 79 of
        'Trajectory planning for automatic machines and robots(2008)'
    """
    # Acceleration period
    if (v_max-v0)/j_max < a_max**2:
        # a_max is not reached
        Tj1 = np.sqrt((v_max-v0)/j_max)
        Ta = 2*Tj1
    else:
        # a_max is reached
        Tj1 = a_max/j_max
        Ta = Tj1 + (v_max-v0)/a_max

    # Deceleration period
    if (v_max-v1)*j_max < a_max**2:
        # a_min is not reached
        Tj2 = np.sqrt((v_max-v1)/j_max)
        Td = 2*Tj2
    else:
        # a_min is reached
        Tj2 = a_max/j_max
        Td = Tj2 + (v_max-v1)/a_max

    Tv = (q1-q0)/v_max - (Ta/2)*(1+v0/v_max)-(Td/2)*(1+v1/v_max)

    if Tv < 0:
        raise PlanningError("Maximum velocity is not reached. Failed to plan"
                            " trajectory")

    return Tj1, Ta, Tj2, Td, Tv


def __compute_maximum_speed_not_reached(q0, q1, v0, v1, v_max, a_max, j_max):
    """
    For explanation look at page 79 of
        'Trajectory planning for automatic machines and robots(2008)'

    Right now assuming that maximum/minimum accelerations is reached
        other case is not treated well. TODO: fix it.
    """

    # Assuming that a_max/a_min is reached
    Tj1 = Tj2 = Tj = a_max/j_max
    Tv = 0

    v = (a_max**2)/j_max
    delta = ((a_max**4)/(j_max**2)) + 2*((v0**2)+(v1**2)) +\
        a_max*(4*(q1-q0)-2*(a_max/j_max)*(v0+v1))

    Ta = (v - 2*v0 + np.sqrt(delta))/(2*a_max)
    Td = (v - 2*v1 + np.sqrt(delta))/(2*a_max)

    if (np.abs(Ta - 2*Tj) > EPSILON) or (np.abs(Td - 2*Tj) > EPSILON):
        raise PlanningError("Maximum acceletaion is not reached. Failed to"
                            " plan trajectory")

    return Tj1, Ta, Tj2, Td, Tv




def __cost_function_jacobian(x, v0, v1, v_max, a_max, j_max):
    """
    Calculated in MATLAB
    """
    as1 = x[0]
    as2 = x[1]
    vs = x[2]
    jac = np.zeros((3,))

    as1sq = as1**2
    as2sq = as2**2

    as1cu = as1**3
    as2cu = as2**3

    j_max_2 = j_max*2
    j_max_3 = j_max*3
    j_max_6 = j_max*6

    j_maxsq = j_maxsq
    j_maxsq_2 = j_maxsq*2
    j_maxsq_3 = j_maxsq*3
    j_maxsq_6 = j_maxsq*6

    delta01 = v0 - v1
    deltas0 = v0 - vs

    jac[0] = 1/j_max - ((as1/j_max + (deltas0)/as1)**2/2 -
                        (as1sq/(j_max_2) + v0)*(1/j_max - (deltas0)/as1sq) +
                        (as1sq/(j_max_6) + v0)/j_max + as1sq/(j_maxsq_3) +
                        (as1sq*(v0 - as1*(as1/j_max +
                                          (deltas0)/as1) + as1sq/(j_max_2)))/
                        j_maxsq - (as1*(as1/j_max + (deltas0)/as1))/j_max +
                        as1*(as1/j_max + (deltas0)/as1)*(1/j_max -
                                                         (deltas0)/as1sq) -
                        (as1cu*(as1*(1/j_max - (deltas0)/as1sq) +
                                (deltas0)/as1))/(j_maxsq_3))/vs +\
        (deltas0)/as1sq

    jac[1] = 3/j_max - ((- as2sq/(j_max_2) + vs)*(1/j_max + (delta01)/as2sq) +
                        (as2/j_max - (delta01)/as2)**2/2 -
                        (as2*(as2/j_max - (delta01)/as2)**2 - vs +
                         (5*as2sq)/(j_max_6))/j_max + (-as2sq/(j_max_6) + vs)/
                        j_max - as2sq/(j_maxsq_3) -
                        (as2*(as2/j_max - (delta01)/as2))/j_max +
                        as2*(as2/j_max - (delta01)/as2)*(1/j_max +
                                                         (delta01)/as2sq) -
                        (as2*((as2/j_max - (delta01)/as2)**2 +
                              (5*as2)/(j_max_3) + 2*as2*(as2/j_max -
                                                         (delta01)/as2)*
                              (1/j_max + (delta01)/as2sq)))/j_max)/vs +\
        (delta01)/as2sq

    jac[2] = (as1/j_max - (3*as2)/j_max + (delta01)/as2 + (deltas0)/as1 -
              (as1sq/(j_max_2) + v0)/as1 - as1cu/(j_maxsq_3))/vs + 1/as1 +\
        ((as2/j_max - (delta01)/as2)*(- as2sq/(j_max_2) + vs) -
         (as1/j_max + (deltas0)/as1)*(as1sq/(j_max_2) + v0) - S +
         (as2*(as2/j_max - (delta01)/as2)**2)/2 + (as1*(as1/j_max +
                                                       (deltas0)/as1)**2)/2 +
         (as1cu*(v0 - as1*(as1/j_max + (deltas0)/as1) +
                 as1sq/(j_max_2)))/(j_maxsq_3) + (as1*(as1sq/(j_max_6) +
                                                       v0))/j_max +
         (as2*(- as2sq/(j_max_6) + vs))/j_max -
         (as2*(as2*(as2/j_max - (delta01)/as2)**2 - vs +
               (5*as2sq)/(j_max_6)))/j_max)/vs**2

    return jac


def __optimized_min_time_scurve(q0, q1, v0, v1, v_max, a_max, j_max):
    """
    Solving optimization problem. Here that of time minimization
    """

    def wrapped_jacobian(x):
        return __cost_function_jacobian(v0, v1, v_max, a_max, j_max)

    pass


def scurve_profile_no_opt(q0, q1, v0, v1, v_max, a_max, j_max):
    if __scurve_check_possibility(q0, q1, v0, v1, v_max, a_max, j_max):
        try:
            Tj1, Ta, Tj2, Td, Tv =\
                __compute_maximum_speed_reached(q0, q1, v0, v1,
                                                v_max, a_max, j_max)
        except PlanningError as e:
            print(e)

            Tj1, Ta, Tj2, Td, Tv =\
                __compute_maximum_speed_not_reached(q0, q1, v0, v1,
                                                    v_max, a_max, j_max)

        return Tj1, Ta, Tj2, Td, Tv

    else:
        raise PlanningError("Trajectory is not feasible")


if __name__ == "__main__":
    q0 =0.
    q1 = 90.
    v0 = 0.
    v1 = 0.
    v_max = 30.
    a_max = 10.
    j_max = 4.

    print(scurve_profile_no_opt(q0, q1, v0, v1, v_max, a_max, j_max))

