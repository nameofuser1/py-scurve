from scipy.optimize import minimize


def optimization_function(S, v0, a_max, T):
    def cost_func(x):
        ta = x[0]
        a = x[1]

        return ((S - a*(ta**2) - 2*v0*ta - (v0+a*ta)*(T-2*ta))**2)

    def cost_func_grad(x):
        ta = x[0]
        a = x[1]

        # According to MATLAB (cost_gradient.m)
        da = 2*((ta**2)*(T - 2*ta) + (ta**2)) * (2*ta*v0 - S +
                                                 (a*(ta**2) + v0) *
                                                 (T - 2*ta) + a*(ta**2))

        dta = 2*(2*a*ta - 2*a*(ta**2) +
                 2*a*ta*(T - 2*ta))*(2*ta*v0 - S + (a*(ta**2) + v0) *
                                     (T - 2*ta) + a*(ta**2))

        return [dta, da]

    return cost_func, cost_func_grad


def optimize_trajectory(S, v0, a_max, T):
    cost_func, cost_func_grad = optimization_function(S, v0, a_max, T)

    # Initial guess
    x0 = [T/4, a_max/2]

    # Lower/upper bounds for ta and a
    bounds = [(0, T/2), (0, a_max)]

    # Minimizing
    results = minimize(cost_func, x0, bounds=bounds, method='SLSQP',
                       jac=cost_func_grad)

    return results.x


def plan_trajectory(S, v0, a_max, T):
    result.x = optimize_trajectory(S, v0, a_max, T)

    ta = x[0]
    a = x[1]




if __name__ == "__main__":
    S = 75
    v0 = 0
    a_max = 3
    T = 10

    print(optimize_trajectory(S, v0, a_max, T))
