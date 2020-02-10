import numpy as np
from .trajectory import Trajectory, PlanningError, EPSILON
from .trajectory import ACCELERATION_ID, SPEED_ID, POSITION_ID
from .planner import TrajectoryPlanner
import logging


logging.basicConfig(format='%(message)s', level=logging.DEBUG)

planning_logger = logging.getLogger(__name__)

# create console handler and set level to debug
# ch = logging.StreamHandler()
# ch.setLevel(logging.INFO)

# planning_logger.addHandler(ch)


class ScurvePlanner(TrajectoryPlanner):

    def __init__(self, debug=False):
        if debug:
            planning_logger.setLevel(logging.DEBUG)
        else:
            planning_logger.setLevel(logging.CRITICAL)
            
        self.s = 1

    def __scurve_check_possibility(self, q0, q1, v0, v1, v_max, a_max, j_max):
        """
        Check whether trajectory is feasible. If not raises PlanningError
        """
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
        planning_logger.info("Computing maximum speed reached profile")

        # Acceleration period
        if (v_max-v0)*j_max < a_max**2:
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

    def __compute_maximum_speed_not_reached(self, q0, q1, v0, v1,
                                            v_max, a_max, j_max):
        """
        For explanation look at page 79 of
            'Trajectory planning for automatic machines and robots(2008)'
        """
        # Assuming that a_max/a_min is reached
        planning_logger.info("Computing maximum speed not reached profile")

        Tj1 = Tj2 = Tj = a_max/j_max
        Tv = 0

        v = (a_max**2)/j_max
        delta = ((a_max**4)/(j_max**2)) + 2*((v0**2)+(v1**2)) +\
            a_max*(4*(q1-q0)-2*(a_max/j_max)*(v0+v1))

        Ta = (v - 2*v0 + np.sqrt(delta))/(2*a_max)
        Td = (v - 2*v1 + np.sqrt(delta))/(2*a_max)

        if (Ta - 2*Tj < EPSILON) or (Td - 2*Tj < EPSILON):
            raise PlanningError("Maximum acceletaion is not reached. Failed to"
                                " plan trajectory")

        return Tj1, Ta, Tj2, Td, Tv

    def __scurve_search_planning(self, q0, q1, v0, v1, v_max, a_max,
                                 j_max, l=0.99, max_iter=2000,
                                 dt_thresh=0.01, T=None):
        """
        Trying to achieve requirements with iteratively decreasing maximum
            possible acceleration.

        Look at 'Trajectory planning for automatic machines and robots(2008)'
        """
        planning_logger.info("Starting search planning")

        _a_max = a_max
        it = 0

        while (it < max_iter) and (_a_max > EPSILON):
            try:
                Tj1, Ta, Tj2, Td, Tv =\
                    self.__compute_maximum_speed_not_reached(q0, q1, v0, v1,
                                                             v_max, _a_max,
                                                             j_max)

                if T is None:
                    return Tj1, Ta, Tj2, Td, Tv

                if abs(T - Ta - Td - Tv) <= dt_thresh:
                    return Tj1, Ta, Tj2, Td, Tv
                else:
                    _a_max *= l
                    it += 1

            except PlanningError:
                it += 1
                _a_max *= l

        raise PlanningError("Failed to find appropriate a_max")

    def __sign_transforms(self, q0, q1, v0, v1, v_max, a_max, j_max):
        """
        Sign transforms for being able to calculate trajectory with q1 < q0

        Look at 'Trajectory planning for automatic machines and robots(2008)'
        """
        v_min = -v_max
        a_min = -a_max
        j_min = -j_max

        s = np.sign(q1-q0)
        vs1 = (s+1)/2
        vs2 = (s-1)/2

        _q0 = s*q0
        _q1 = s*q1
        _v0 = s*v0
        _v1 = s*v1
        _v_max = vs1*v_max + vs2*v_min
        _a_max = vs1*a_max + vs2*a_min
        _j_max = vs1*j_max + vs2*j_min

        return _q0, _q1, _v0, _v1, _v_max, _a_max, _j_max

    def __point_sign_transform(self, q0, q1, p):
        """
        Transforms point back to the original sign
        """
        s = np.sign(q1-q0)
        return s*p

    def __get_trajectory_func(self, Tj1, Ta, Tj2, Td, Tv,
                              q0, q1, v0, v1, v_max, a_max, j_max):
        """
        Returns function of time given trajectory parameters
        """
        T = Ta + Td + Tv
        a_lim_a = j_max*Tj1
        a_lim_d = -j_max*Tj2
        v_lim = v0 + (Ta-Tj1)*a_lim_a

        def trajectory(t):
            """
            Returns numpy array with shape (3,) which contains acceleration,
                speed and position for a given time t
            """
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

            elif T-Tj2 <= t < T:
                tt = T-t

                a = -j_max*tt
                v = v1 + j_max*(tt**2)/2
                q = q1 - v1*tt - j_max*(tt**3)/6

            else:
                a = 0
                v = v1
                q = q1

            point = np.zeros((3,), dtype=np.float32)
            point[ACCELERATION_ID] = a
            point[SPEED_ID] = v
            point[POSITION_ID] = q

            return point

        return trajectory

    def __get_trajectory_function(self, q0, q1, v0, v1, v_max, a_max, j_max,
                                  Tj1, Ta, Tj2, Td, Tv):
        """
        Returns function wich wrapps trajectory function with sign transforms
        """
        zipped_args = self.__sign_transforms(q0, q1, v0, v1, v_max, a_max,
                                             j_max)

        traj_func = self.__get_trajectory_func(Tj1, Ta, Tj2,
                                               Td, Tv, *zipped_args)

        def sign_back_transformed(t):
            return self.__point_sign_transform(q0, q1, traj_func(t))

        return sign_back_transformed

    def __scurve_profile_no_opt(self, q0, q1, v0, v1, v_max, a_max, j_max):
        """
        Computes s-curve trajectory parameters which are:
            Tj1     --- non-zero constant jerk period while accelerating
            Ta      --- total acceleration period time
            Tj2     --- non-zero constant jerk period while decelerating
            Td      --- total deceleration time
            Tv      --- constant speed time
        """
        if self.__scurve_check_possibility(q0, q1, v0, v1, v_max, a_max, j_max):
            try:
                Tj1, Ta, Tj2, Td, Tv =\
                    self.__compute_maximum_speed_reached(q0, q1, v0, v1,
                                                         v_max, a_max, j_max)
            except PlanningError as e:
                planning_logger.warn(e)

                try:
                    Tj1, Ta, Tj2, Td, Tv =\
                        self.__compute_maximum_speed_not_reached(q0, q1, v0, v1,
                                                                 v_max, a_max,
                                                                 j_max)
                except PlanningError as e:
                    planning_logger.warn(e)

                    try:
                        Tj1, Ta, Tj2, Td, Tv =\
                            self.__scurve_search_planning(q0, q1, v0, v1, v_max,
                                                          a_max, j_max)
                    except PlanningError as e:
                        planning_logger.warn(e)
                        raise PlanningError("Trajectory is infeasible")

            return np.asarray([Tj1, Ta, Tj2, Td, Tv], dtype=np.float32)

        else:
            raise PlanningError("Trajectory is not feasible")

    def __put_params(self, params_list, params, dof):
        for i in range(len(params_list)):
            params_list[i][dof] = params[i]

    def __get_dof_time(self, params_list, dof):
        return params_list[1][dof] + params_list[3][dof] + params_list[4][dof]

    def __get_traj_params_containers(self, sh):
        T = np.zeros(sh)
        Ta = np.zeros(sh)
        Tj1 = np.zeros(sh)
        Td = np.zeros(sh)
        Tj2 = np.zeros(sh)
        Tv = np.zeros(sh)

        return T, Tj1, Ta, Tj2, Td, Tv

    def __plan_trajectory_1D(self, q0, q1, v0, v1, v_max, a_max, j_max, T=None):
        """
        Computes optimal time scurve trajectory or trying to fit it in time T

        returns list of trajecotry parameters
        """
        zipped_args = self.__sign_transforms(q0, q1, v0, v1, v_max, a_max,
                                             j_max)

        planning_logger.info("Planning trajectory with given parameters")
        planning_logger.info("%f %f %f %f %f %f %f" %
                             (q0, q1, v0, v1, v_max, a_max, j_max) +
                             str(T))
        planning_logger.debug("Sign transform result")
        planning_logger.debug("{} {} {} {} {} {} {}".format(*zipped_args))

        if T is None:
            planning_logger.info("Computing Optimal time profile")
            res = self.__scurve_profile_no_opt(*zipped_args)
        else:
            planning_logger.info("Computing constant time profile")
            res = self.__scurve_search_planning(*zipped_args, T=T)

        T = res[1] + res[3] + res[4]
        a_max_c = res[0]*j_max
        a_min_c = a_max_c - res[2]*j_max
        planning_logger.info(
            "Planning results:\r\n\t"
            "Maximum acceleration: {}\r\n\t"
            "Minimum acceleration: {}\r\n\t"
            "T: {}\r\n\t"
            "Tj1: {}\r\n\t"
            "Ta: {}\r\n\t"
            "Tj2: {}\r\n\t"
            "Td: {}\r\n\t"
            "Tv: {}\r\n\r\n".format(a_max_c, a_min_c, T, *res))

        return res

    def plan_trajectory(self, q0, q1, v0, v1, v_max, a_max, j_max, t=None):
        """
        Plan scurve trajectory with give constraints

        returns function of time which returns acceleration, velocity and
            position for time t
        """

        planning_logger.info("********************************************"
                             "\r\n\t"
                             "NEW TRAJECTORY\r\n"
                             "********************************************")
        sh = self._check_shape(q0, q1, v0, v1)
        ndof = sh[0]

        # Easy slices with numpy
        task_list = np.asarray([q0, q1, v0, v1, [v_max]*ndof,
                               [a_max]*ndof, [j_max]*ndof],
                               dtype=np.float32)

        # Get arrays to save results into
        T, Tj1, Ta, Tj2, Td, Tv = self.__get_traj_params_containers(sh)
        trajectory_params = np.asarray([Tj1, Ta, Tj2, Td, Tv], dtype=np.float32)
        trajectory_funcs = []

        # Calculating params for the longest trajectory
        dq = np.subtract(q1, q0)
        max_displacement_id = np.argmax(np.abs(dq))

        planning_logger.info("Computing the longest DOF trajectory with id"
                             " %d" % max_displacement_id)

        max_displacement_params =\
            self.__plan_trajectory_1D(*task_list[:, max_displacement_id], T=t)

        self.__put_params(trajectory_params,
                          max_displacement_params,
                          max_displacement_id)

        max_displacement_time = self.__get_dof_time(trajectory_params,
                                                    max_displacement_id)
        T[max_displacement_id] = max_displacement_time

        for _q0, _q1, _v0, _v1, ii in zip(q0, q1, v0, v1, range(ndof)):
            if ii == max_displacement_id:
                continue

            planning_logger.info("Computing %d DOF trajectory" % ii)

            # In case if final velocity is non-zero
            # We need to synchronize it in time with the trajectory with
            # the longest execution time
            if _v1 != 0:
                traj_params =\
                    self.__plan_trajectory_1D(_q0, _q1, _v0, _v1, a_max,
                                              v_max, j_max,
                                              T=max_displacement_time)

            # if final velocity is zero we do not need to worry about
            # syncronization
            else:
                traj_params = self.__plan_trajectory_1D(_q0, _q1, _v0, _v1,
                                                        a_max, v_max, j_max)

            T[ii] = Ta[ii] + Td[ii] + Tv[ii]
            self.__put_params(trajectory_params, traj_params, ii)

        # Getting trajectory functions
        for dof in range(ndof):
            tr_func = self.__get_trajectory_function(q0[dof], q1[dof],
                                                     v0[dof], v1[dof], v_max,
                                                     a_max, j_max,
                                                     *trajectory_params[:, dof])
            trajectory_funcs.append(tr_func)

        tr = Trajectory()
        tr.time = (T[max_displacement_id],)
        tr.trajectory = trajectory_funcs
        tr.dof = ndof

        return tr
