from pyscurve import ScurvePlanner, plot_trajectory

if __name__ == "__main__":
    q0 = [-2.]
    q1 = [20.]
    v0 = [1.]
    v1 = [5.]
    v_max = 20.
    a_max = 15.
    j_max = 100.

    p = ScurvePlanner()
    tr = p.plan_trajectory(q0, q1, v0, v1, v_max, a_max, j_max)
    plot_trajectory(tr, 0.01)