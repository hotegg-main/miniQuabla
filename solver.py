from scipy.integrate import solve_ivp, odeint
import numpy as np
import quaternion
from Parameter.parameter import Parameter
import time

def run_single(path_config, path_result):

    from PostProcess.time_history import calc_sub_values
    from PostProcess.time_history import plot_main_values

    param = Parameter(path_config)
    
    elp = []
    elp.append(time.time())
    
    time_log, pos_log, vel_log, quat_log, omega_log, mass_log, time_log_para, pos_log_para \
        = solve_dynamics(param)
    elp.append(time.time())
    
    plot_main_values(path_result, time_log, pos_log, vel_log, quat_log, omega_log, mass_log, time_log_para, pos_log_para, param)
    elp.append(time.time())
    
    calc_sub_values(path_result, time_log, pos_log, vel_log, quat_log, omega_log, mass_log, param)
    elp.append(time.time())
    
    # 経過時間
    print('Solve ODE :', elp[1] - elp[0])
    print('Plot      :', elp[2] - elp[1])
    print('Calc Other:', elp[3] - elp[2])

    print(' ---> Simulation END Successfully!')

def run_loop(path_config, path_result):
    '''落下分散計算用の関数'''
    import multiprocessing
    from PostProcess.land_map import plot_kml
    # from tqdm import tqdm

    job_list = []
    manager = multiprocessing.Manager()
    result_list = manager.list()

    col = 7 # Speed
    row = 8 # Azimuth
    speed_sta  = 1.
    speed_step = 2.
    azimuth_sta = 0.
    row += 1
    azimuth_step = 360. / (row - 1)
    speed_end  = 1.
    azimuth_end = azimuth_step * row + azimuth_sta

    speed_array = np.linspace(speed_sta, speed_sta + col*speed_step, col+1)
    azimuth_array = np.deg2rad(np.linspace(azimuth_sta, azimuth_end, row+1))
    
    elp = []
    elp.append(time.time())
    for i in range(col):
        for j in range(row):
            job = dict()
            job['Column']   = i 
            job['Row']      = j
            job['Wind Speed']   = speed_array[i]
            job['Wind Azimuth'] = azimuth_array[j]
            job_list.append(job)

    p_list = []
    for job in job_list:

        p = multiprocessing.Process(target=solve_dynamics_for_loop, args=(path_config, job, result_list))
        p_list.append(p)
        p.start()
        # idx += 1

        for j, _p in enumerate(p_list):
            if not _p.is_alive():
                p_list.pop(j)

        # 使用プロセス数が上限に達したらプロセス終了を待つ
        if len(p_list) >= multiprocessing.cpu_count() - 1:
            # いずれかのプロセスの終了を待つ
            loopf=True
            while loopf:
                for j, _p in enumerate(p_list):
                    if not _p.is_alive():
                        p_list.pop(j)
                        loopf=False
                        break

    for p in p_list:
        p.join()
    
    elp.append(time.time())
    print('Elapse:', elp[1] - elp[0])

    result_hard = np.zeros((col, row, 3))
    result_soft = np.zeros((col, row, 3))
    for result in result_list:
        i = result['Column']    
        j = result['Row']    
        result_hard[i][j] = result['Pos_hard']
        result_soft[i][j] = result['Pos_soft']

    dummy = Parameter(path_config)
    plot_kml(path_result, dummy.launch.LLH, result_hard, result_soft, speed_array)
    
    print(' ---> Simulation END Successfully!')

def solve_dynamics(param: Parameter):
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
    time_sta = 0.
    time_end = param.t_max
    t_eval = np.append(
    np.arange(time_sta, param.engine.time_act * 1.2, param.dt),
    np.arange(param.engine.time_act * 1.2, time_end, param.dt * 5))
    
    result = solve_ivp(dynamics_trajectory, t_span=(time_sta, param.t_max), y0=x0, args=(param, ), events=event_land, dense_output=True, rtol=1.e-05, t_eval=t_eval)

    time_log  = result.t
    pos_log   = result.y[0:3].T
    vel_log   = result.y[3:6].T
    quat_log  = result.y[6:10].T
    omega_log = result.y[10:13].T
    mass_log  = result.y[13]

    index_apogee, _, _ = check_apogee(time_log, pos_log)
    
    time0 = time_log[index_apogee]
    x0 = np.zeros(3)
    x0[0:3] = pos_log[index_apogee]

    t_eval = np.arange(time0, param.t_max, param.dt)
    result = solve_ivp(dynamics_parachute, t_span=(time0, param.t_max), t_eval=t_eval, y0=x0, args=(param, ), events=event_land, rtol=1.e-06, atol=1.e-04)
    time_log_para = result.t
    pos_log_para  = result.y[0:3].T

    return time_log, pos_log, vel_log, quat_log, omega_log, mass_log, time_log_para, pos_log_para

def solve_dynamics_for_loop(path, job, result):
    '''
    落下分散計算用
    (条件、結果格納用リスト)
    ジョブリスト：
    番号、風向、風速
    '''
    param = Parameter(path)
    param.wind.speed_ref    = job['Wind Speed']
    param.wind.azimuth_ref  = job['Wind Azimuth']
    # i = job['Column']
    # j = job['Row']

    _, pos_log, _, _, _, _, _, pos_log_para = \
        solve_dynamics(param)
    
    res = dict()
    res['Column'] = job['Column']
    res['Row']    = job['Row']
    res['Pos_hard'] = pos_log[-1]
    res['Pos_soft'] = pos_log_para[-1]
    result.append(res)

def check_apogee(time_log, pos_log):

    index = np.argmax( - pos_log[:, 2])
    time_apogee = time_log[index]
    pos_apogee  = pos_log[index]

    return index, time_apogee, pos_apogee

# def __debug():

#     from PostProcess.time_history import calc_sub_values
#     from PostProcess.time_history import plot_main_values
#     import time

#     param = Parameter('example/rocket_config.csv')
#     path_result = 'test'
    
#     elp = []
#     elp.append(time.time())
#     time_log, pos_log, vel_log, quat_log, omega_log, mass_log, pos_log_para \
#         = solve_dynamics(param)
#     elp.append(time.time())
#     plot_main_values(path_result, time_log, pos_log, vel_log, quat_log, omega_log, mass_log, param)
#     elp.append(time.time())
#     calc_sub_values(path_result, time_log, pos_log, vel_log, quat_log, omega_log, mass_log, param)
#     elp.append(time.time())
#     print('Solve ODE :', elp[1] - elp[0])
#     print('Plot      :', elp[2] - elp[1])
#     print('Calc Other:', elp[3] - elp[2])

if __name__=='__main__':

    print('Hello World!')
    # __debug()
