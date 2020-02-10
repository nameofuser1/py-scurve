from pyscurve import ScurvePlanner
from pyscurve import plot_trajectory

if __name__ == "__main__":
    q0 = [-2., 0., 10.]
    q1 = [20., 15., -10.]
    v0 = [0., 5., 0.]
    v1 = [2, 4., 0.]
    v_max = 30.
    a_max = 30.
    j_max = 100.
    T = 3.0

    p = ScurvePlanner()

    tr = p.plan_trajectory(q0, q1, v0, v1, v_max, a_max, j_max, t=T)
    plot_trajectory(tr, dt=0.01)