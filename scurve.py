import numpy as np
from trajectories import Trajectory, plot_trajectory, PlanningError, EPSILON
from trajectories import ACCELERATION_ID, SPEED_ID, POSITION_ID
from planner import TrajectoryPlanner


class ScurvePlanner(TrajectoryPlanner):

    def __init__(self):
        self.s = 1

    def __scurve_check_possibility(self, q0, q1, v0, v1, v_max, a_max, j_max):
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

    def __compute_maximum_speed_reached(self, q0, q1, v0, v1,
                                        v_max, a_max, j_max):
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
            raise PlanningError("Maximum velocity is not reached. "
                                "Failed to plan trajectory")

        return Tj1, Ta, Tj2, Td, Tv

    def __sign_transforms(self, q0, q1, v0, v1, v_max, a_max, j_max):
        v_min = -v_max
        a_min = -a_max
        j_min = -j_max

        vs1 = (self.s+1)/2
        vs2 = (self.s-1)/2

        _q0 = self.s*q0
        _q1 = self.s*q1
        _v0 = self.s*v0
        _v1 = self.s*v1
        _v_max = vs1*v_max + vs2*v_min
        _a_max = vs1*a_max + vs2*a_min
        _j_max = vs1*j_max + vs2*j_min

        print("%f %f %f %f %f %f %f" % (q0, q1, v0, v1, v_max, a_max, j_max))
        print("%f %f %f %f %f %f %f" % (_q0, _q1, _v0,
                                        _v1, _v_max, _a_max, _j_max))

        return _q0, _q1, _v0, _v1, _v_max, _a_max, _j_max

    def __point_sign_transform(self, p):
        new_p = np.zeros(p.shape, dtype=np.float32)

        for i in range(p.shape[0]):
            for j in range(p.shape[1]):
                new_p[i][j] = p[i][j] * self.s

        return new_p

    def __compute_maximum_speed_not_reached(self, q0, q1, v0, v1,
                                            v_max, a_max, j_max):
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

        if (np.abs(Ta - 2*Tj) < EPSILON) or (np.abs(Td - 2*Tj) < EPSILON):
            raise PlanningError("Maximum acceletaion is not reached. Failed to"
                                " plan trajectory")

        return Tj1, Ta, Tj2, Td, Tv

    def get_trajectory_func(self, Ta, Tj1, Td, Tj2, Tv,
                            q0, q1, v0, v1, v_max, a_max, j_max):
        T = Ta + Td + Tv
        a_lim_a = j_max*Tj1
        a_lim_d = -j_max*Tj2
        v_lim = v0 + (Ta-Tj1)*a_lim_a

        def trajectory(t):
            # Acceleration phase
            if 0 <= t < Tj1:
                a = j_max*t
                v = v0 + j_max*(t**2)/2
                q = q0 + v0*t + j_max*(t**3)/6

            elif Tj1 <= t < Ta - Tj1:
                a = a_lim_a
                v = v0 + a_lim_a*(t-Tj1/2)
                q = q0 + v0*t + a_lim_a*(3*(t**2) - 3*Tj1*t + Tj1**2)/6

            elif Ta-Tj1 <= t < Ta:
                tt = Ta - t

                a = j_max*tt
                v = v_lim - j_max*(tt**2)/2
                q = q0 + (v_lim+v0)*Ta/2 - v_lim*tt + j_max*(tt**3)/6

            # Constant velocity phase
            elif Ta <= t < Ta + Tv:
                a = 0
                v = v_lim
                q = q0 + (v_lim+v0)*Ta/2 + v_lim*(t-Ta)

            # Deceleration phase
            elif T - Td <= t < T-Td+Tj2:
                tt = t-T+Td

                a = -j_max*tt
                v = v_lim - j_max*(tt**2)/2
                q = q1 - (v_lim+v1)*Td/2 + v_lim*tt -\
                    j_max*(tt**3)/6

            elif T-Td+Tj2 <= t < T-Tj2:
                tt = t-T+Td

                a = a_lim_d
                v = v_lim + a_lim_d*(tt-Tj2/2)
                q = q1 - (v_lim+v1)*Td/2 + v_lim*tt +\
                    a_lim_d*(3*(tt**2) - 3*Tj2*tt + Tj2**2)/6

            elif T-Tj2 <= t:
                tt = T-t

                a = -j_max*tt
                v = v1 + j_max*(tt**2)/2
                q = q1 - v1*tt - j_max*(tt**3)/6

            point = np.zeros((1, 3), dtype=np.float32)
            point[0][ACCELERATION_ID] = a
            point[0][SPEED_ID] = v
            point[0][POSITION_ID] = q

            return self.__point_sign_transform(point)

        return trajectory

    def scurve_profile_no_opt(self, q0, q1, v0, v1, v_max, a_max, j_max):
        if self.__scurve_check_possibility(q0, q1, v0, v1, v_max, a_max, j_max):
            try:
                Tj1, Ta, Tj2, Td, Tv =\
                    self.__compute_maximum_speed_reached(q0, q1, v0, v1,
                                                         v_max, a_max, j_max)
            except PlanningError as e:
                print(e)

                Tj1, Ta, Tj2, Td, Tv =\
                    self.__compute_maximum_speed_not_reached(q0, q1, v0, v1,
                                                             v_max, a_max,
                                                             j_max)

            return Tj1, Ta, Tj2, Td, Tv

        else:
            raise PlanningError("Trajectory is not feasible")

    def scurve_profile_const_time(self, T, q0, q1, v0, v1, v_max,
                                  a_max, j_max, l=0.9, max_iter=2000):
        _T = 0
        _a_max = a_max
        _it_num = 0
        exceptions_in_row = 0

        while (abs(_T-T) >= EPSILON) and (_a_max >= EPSILON) and\
                (_it_num < max_iter):
            try:
                Tj1, Ta, Tj2, Td, Tv = self.scurve_profile_no_opt(q0, q1, v0,
                                                                  v1,
                                                                  v_max,
                                                                  _a_max,
                                                                  j_max)

                _T = Ta + Td + Tv
                exceptions_in_row = 0

            except PlanningError:
                exceptions_in_row += 1

            _a_max *= l
            _it_num += 1

        if abs(_T-T) >= EPSILON:
            raise PlanningError("Failed to fit trajectory in given time.\r\n"
                                "Exceptions in row: %f\r\n"
                                "Iterations number: %f\r\n"
                                "Current acceleration %f"
                                % (exceptions_in_row, _it_num, _a_max))
        else:
            print("_T: %f, T: %f, dT: %f" % (_T, T, abs(_T-T)))

        return Tj1, Ta, Tj2, Td, Tv

    def plan_trajectory(self, q0, q1, v0, v1, v_max, a_max, j_max, T=None):
        # For sign transforms
        self.s = np.sign(q1-q0)

        zipped_args =\
            self.__sign_transforms(q0, q1, v0, v1, v_max, a_max, j_max)

        if T is None:
            Tj1, Ta, Tj2, Td, Tv =\
                self.scurve_profile_no_opt(*zipped_args)

        else:
            Tj1, Ta, Tj2, Td, Tv = self.scurve_profile_const_time(T,
                                                                  *zipped_args)

        T = Ta + Td + Tv
        tr_func = self.get_trajectory_func(Ta, Tj1, Td, Tj2, Tv,
                                           *zipped_args)

        print("T: %f\r\nTa: %f\r\nTd: %f\r\nTv: %f" % (T, Ta, Td, Tv))

        tr = Trajectory()
        tr.time = (T,)
        tr.trajectory = tr_func
        tr.dof = 1

        return tr


if __name__ == "__main__":
    q0 = -2.
    q1 = 20.
    v0 = 1.
    v1 = 5.
    v_max = 20.
    a_max = 20
    j_max = 100.

    p = ScurvePlanner()

    tr = p.plan_trajectory(q0, q1, v0, v1, v_max, a_max, j_max, T=3.17)
    plot_trajectory(tr, 0.01)
