from scipy.integrate import solve_ivp

def solve_dynamics():
    '''
    '''
    from dynamics import dynamics_trajectory

    result = solve_ivp(dynamics_trajectory)