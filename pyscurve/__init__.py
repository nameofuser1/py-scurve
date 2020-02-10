import logging
import os
from .scurve import ScurvePlanner
from .trajectory import Trajectory, plot_trajectory


DEBUG = False

trajectory_logger = logging.getLogger("trajectory_logger")
planning_logger = logging.getLogger("planning_logger")


if DEBUG:
    cur_dir = os.path.dirname(os.path.abspath(__file__))

    FNAME_EXT = ".log"
    PLANNING_LOG_FNAME = cur_dir + os.path.sep + "planning" + FNAME_EXT
    TRAJECTORY_LOG_FNAME = cur_dir + os.path.sep + "trajectory" + FNAME_EXT

    planning_logger.setLevel(logging.DEBUG)
    trajectory_logger.setLevel(logging.DEBUG)

    plan_fhandler = logging.FileHandler(PLANNING_LOG_FNAME, 'w')
    plan_fhandler.setLevel(logging.DEBUG)

    traj_fhandler = logging.FileHandler(TRAJECTORY_LOG_FNAME, 'w')
    traj_fhandler.setLevel(logging.DEBUG)

    planning_formatter = logging.Formatter("%(asctime)s - %(levelname)s - "
                                        "LINE %(lineno)s "
                                        "-\r\n%(message)s")
    trajectory_formatter = logging.Formatter("%(message)s")

    plan_fhandler.setFormatter(planning_formatter)
    traj_fhandler.setFormatter(trajectory_formatter)

    planning_logger.addHandler(plan_fhandler)
    trajectory_logger.addHandler(traj_fhandler)

else:
    planning_logger.setLevel(logging.FATAL)
    trajectory_logger.setLevel(logging.FATAL)
