from pyscurve import ScurvePlanner, plot_trajectory

if __name__ == "__main__":
    p = ScurvePlanner()

    q0 = [-.2, 0., 0.]
    q1 = [.2, 1, -.30]
    v0 = [0., 1., 0.]
    v1 = [.1, 0.0, 0.2]
    v_max = 20.
    a_max = 30.
    j_max = 100.
    T = None

    tr = p.plan_trajectory(q0, q1, v0, v1, v_max, a_max, j_max)
    plot_trajectory(tr, 0.01)