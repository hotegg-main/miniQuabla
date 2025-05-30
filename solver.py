from scipy.integrate import solve_ivp
import numpy as np
import quaternion

def solve_dynamics(param):
    '''
    '''
    from dynamics import dynamics_trajectory, dynamics_parachute, event_land

    pos0, vel0, quat0, omega0, mass0 = param.get_initial_param()
    x0 = np.zeros(14)
    x0[0:3]     = pos0
    x0[3:6]     = vel0
    x0[6:10]    = quaternion.as_float_array(quat0)
    x0[10:13]   = omega0
    x0[13]      = mass0

    event_land.terminal = True
    result = solve_ivp(dynamics_trajectory, t_span=(0., param.t_max), t_eval=np.arange(0., param.t_max, param.dt), y0=x0, args=(param, ), events=event_land, rtol=1.e-06, atol=1.e-04)
    time_log = result.t
    pos_log   = result.y[0:3].T
    vel_log   = result.y[3:6].T
    quat_log  = result.y[6:10].T
    omega_log = result.y[10:13].T
    mass_log  = result.y[13]

    index_apogee, _, _ = check_apogee(time_log, pos_log)
    
    time0 = time_log[index_apogee]
    x0 = np.zeros(3)
    x0[0:3] = pos_log[index_apogee]

    result = solve_ivp(dynamics_parachute, t_span=(time0, param.t_max), t_eval=np.arange(time0, param.t_max, param.dt), y0=x0, args=(param, ), events=event_land, rtol=1.e-06, atol=1.e-04)
    # time_log = result.t
    # pos_log   = result.y[0:3].T
    # pos_fin = pos_log[-1]

    return time_log, pos_log, vel_log, quat_log, omega_log, mass_log

def check_launch_clear(time_log, pos_log):
    ''''''

def check_apogee(time_log, pos_log):

    index = np.argmax( - pos_log[:, 2])
    time_apogee = time_log[index]
    pos_apogee  = pos_log[index]

    return index, time_apogee, pos_apogee

def __debug():

    from PostProcess.time_history import calc_sub_values
    from PostProcess.time_history import plot_main_values

    param = Parameter('example/rocket_config.csv')
    time_log, pos_log, vel_log, quat_log, omega_log, mass_log \
        = solve_dynamics(param)
    plot_main_values('test', time_log, pos_log, vel_log, quat_log, omega_log, mass_log, param)
    calc_sub_values('test', time_log, pos_log, vel_log, quat_log, omega_log, mass_log, param)

if __name__=='__main__':

    from Parameter.parameter import Parameter

    print('Hello World!')
    __debug()
