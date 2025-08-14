from scipy.integrate import solve_ivp, odeint
import numpy as np
import quaternion
from Parameter.parameter import Parameter
import time
import simplekml
from PostProcess.directory_manage import make_result_directory

def run_single(path_config, path_result_src, name_case):

    from PostProcess.time_history import calc_sub_values, plot_main_values, calc_index_apogee

    param = Parameter(path_config)
    
    elp = []
    elp.append(time.time())
    
    time_log, pos_log, vel_log, quat_log, omega_log, mass_log \
        = solve_trajectory(param)
    
    # 初期値設定
    index = calc_index_apogee(pos_log)
    time0 = time_log[index]
    pos0  = pos_log[index]
    
    if param.payload.exist:
        # ペイロード放出あり
        time_log_payload, pos_log_payload = solve_parachute(param, param.payload, time0, pos0)
        
    else:
        time_log_payload    = None
        pos_log_payload     = None

    time_log_para, pos_log_para = solve_parachute(param, param.para, time0, pos0)
    
    elp.append(time.time())
    
    path_result = make_result_directory(path_result_src, param.name + '_' + name_case, 'single')
    plot_main_values(path_result, param, time_log, pos_log, vel_log, quat_log, omega_log, mass_log, time_log_para, pos_log_para, param.payload.exist, time_log_payload, pos_log_payload)
    elp.append(time.time())
    
    calc_sub_values(path_result, time_log, pos_log, vel_log, quat_log, omega_log, mass_log, time_log_para, pos_log_para, param)
    elp.append(time.time())
    
    # 経過時間
    print('Solve ODE :', np.round(elp[1] - elp[0], 3), 'sec')
    print('Plot      :', np.round(elp[2] - elp[1], 3), 'sec')
    print('Calc Other:', np.round(elp[3] - elp[2], 3), 'sec')

    print(' ---> Simulation END Successfully!')

def run_loop(path_config, path_result_src, cond):
    '''落下分散計算用の関数'''
    import multiprocessing
    from tqdm import tqdm

    job_list = []
    manager = multiprocessing.Manager()
    result_list = manager.list()

    col = cond['Speed']['num']   # Speed
    row = cond['Azimuth']['num'] # Azimuth
    
    speed_sta   = cond['Speed']['min']
    speed_step  = cond['Speed']['step']
    
    azimuth_sta  = cond['Azimuth']['min']
    azimuth_step = 360. / row
    azimuth_end  = azimuth_sta + azimuth_step * (row - 1)

    speed_array   = np.linspace(speed_sta, speed_sta + (col - 1)*speed_step, col)
    azimuth_array = np.linspace(azimuth_sta, azimuth_end, row)

    dummy = Parameter(path_config)
    path_result = make_result_directory(path_result_src, dummy.name, 'loop')
    
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
    for job in tqdm(job_list):

        p = multiprocessing.Process(target=solve_dynamics_for_loop, args=(path_config, job, result_list))
        p_list.append(p)
        p.start()

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
    
    # 経過時間
    elp.append(time.time())
    print('Elapse:', np.round(elp[1] - elp[0], 3), 'sec')

    # 結果整理
    dummy = Parameter(path_config)
    make_summay_for_loop(path_result, result_list, speed_array, azimuth_array, dummy)
    
    print(' ---> Simulation END Successfully!')

def solve_trajectory(param: Parameter):

    from dynamics import dynamics_trajectory, event_land

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
        np.arange(param.engine.time_act * 1.2, time_end, param.dt * 5)
    )
    t_eval[-1] = np.min([time_end, t_eval[-1]])
    
    result = solve_ivp(dynamics_trajectory, t_span=(time_sta, param.t_max), y0=x0, args=(param, ), events=event_land, dense_output=True, rtol=1.e-05, t_eval=t_eval)

    # 結果のコピー
    time_log  = result.t
    pos_log   = result.y[0:3].T
    vel_log   = result.y[3:6].T
    quat_log  = result.y[6:10].T
    omega_log = result.y[10:13].T
    mass_log  = result.y[13]

    return time_log, pos_log, vel_log, quat_log, omega_log, mass_log

def solve_parachute(param: Parameter, para, time0, pos0):

    from dynamics import dynamics_parachute, event_land_soft

    x0 = np.zeros(3)
    x0[0:3] = pos0

    event_land_soft.terminal = True
    time_end = param.t_max
    t_eval = np.arange(time0, param.t_max, param.dt * 10.)
    t_eval[-1] = np.min([time_end, t_eval[-1]])
    
    result = solve_ivp(dynamics_parachute, t_span=(time0, param.t_max), t_eval=t_eval, y0=x0, args=(param, para, ), events=event_land_soft, rtol=1.e-06, atol=1.e-04)
    
    # 結果のコピー
    time_log_para = result.t
    pos_log_para  = result.y[0:3].T

    return time_log_para, pos_log_para

def solve_dynamics_payload(param, time_log, pos_log, vel_log, quat_log, omega_log, mass_log, index):
    '''
    ペイロードの軌道とペイロード放出後の機体の軌道を再計算する
    '''
    from dynamics import dynamics_trajectory, dynamics_parachute, event_land, event_land_soft
    
    time0 = time_log[index]
    event_land_soft.terminal = True
    t_eval = np.arange(time0, param.t_max, param.dt * 10.)
    x0 = np.zeros(3)
    x0[0:3] = pos_log[index]

    # Payload
    result = solve_ivp(dynamics_parachute, t_span=(time0, param.t_max), t_eval=t_eval, y0=x0, args=(param, param.payload, ), events=event_land_soft, rtol=1.e-06, atol=1.e-04)
    time_log_payload = result.t
    pos_log_payload  = result.y[0:3].T

    # Trajectory 
    # ペイロード分離後の軌道を再計算
    x0 = np.zeros(14)
    x0[0:3]     = pos_log[index]
    x0[3:6]     = vel_log[index]
    x0[6:10]    = quat_log[index]
    x0[10:13]   = omega_log[index]
    x0[13]      = mass_log[index] - param.payload.mass

    event_land.terminal = True
    time_sta = time_log[index]
    time_end = param.t_max
    t_eval = np.arange(time_sta, time_end, param.dt * 5.)
    t_eval[-1] = np.min([time_end, t_eval[-1]])

    result = solve_ivp(dynamics_trajectory, t_span=(time_sta, param.t_max), y0=x0, args=(param, ), events=event_land, dense_output=True, rtol=1.e-05, t_eval=t_eval)

    time_log_new  = np.append(time_log[:index]  , result.t)
    pos_log_new   = np.append(pos_log[:index]   , result.y[0:3].T   , axis=0)
    vel_log_new   = np.append(vel_log[:index]   , result.y[3:6].T   , axis=0)
    quat_log_new  = np.append(quat_log[:index]  , result.y[6:10].T  , axis=0)
    omega_log_new = np.append(omega_log[:index] , result.y[10:13].T , axis=0)
    mass_log_new  = np.append(mass_log[:index]  , result.y[13])

    return time_log_new, pos_log_new, vel_log_new, quat_log_new, omega_log_new, mass_log_new, \
        time_log_payload, pos_log_payload

def solve_dynamics_for_loop(path, job, result):
    '''
    落下分散計算用
    (条件、結果格納用リスト)
    ジョブリスト：
    番号、風向、風速
    '''

    from PostProcess.time_history import calc_values_min, calc_index_apogee

    param = Parameter(path)
    param.wind.set_power_model(job['Wind Speed'], job['Wind Azimuth'])

    res = dict()

    time_log, pos_log, vel_log, quat_log, omega_log, mass_log \
        = solve_trajectory(param)
    
    # 初期値設定
    index = calc_index_apogee(pos_log)
    time0 = time_log[index]
    pos0  = pos_log[index]
    
    if param.payload.exist:
        # ペイロード放出あり
        
        time_log_payload, pos_log_payload = solve_parachute(param, param.payload, time0, pos0)
        
        res['Pos_payload'] = pos_log_payload[-1]

    time_log_para, pos_log_para = solve_parachute(param, param.para, time0, pos0)
    
    res['Column']   = job['Column']
    res['Row']      = job['Row']
    res['Pos_hard'] = pos_log[-1]
    res['Pos_soft'] = pos_log_para[-1]
    res['results']  = calc_values_min(time_log, pos_log, vel_log, quat_log, omega_log, mass_log, time_log_para, pos_log_para, param)
    if param.payload.exist:
        res['results']['pos_land_payload'] = pos_log_payload[-1]
    result.append(res)

def make_summay_for_loop(path, result_list, speed, azimuth, param: Parameter):

    import pandas as pd
    import os
    from PostProcess.land_map import output_land_map

    index_list = (speed)
    header_list = (azimuth)

    col = len(speed)
    row = len(azimuth)

    time_launch_clear   = np.zeros((col, row))
    vel_launch_clear    = np.zeros((col, row))
    time_apogee         = np.zeros((col, row))
    alt_apogee          = np.zeros((col, row))
    time_land_hard      = np.zeros((col, row))
    time_land_soft      = np.zeros((col, row))
    mach_max            = np.zeros((col, row))
    vel_air_max         = np.zeros((col, row))
    pos_land_hard       = np.zeros((col, row, 3))
    pos_land_soft       = np.zeros((col, row, 3))
    pos_land_payload    = np.zeros((col, row, 3))

    for result in result_list:

        i = result['Column']
        j = result['Row']
        res = result['results']

        time_launch_clear[i, j] = res['time_launch_clear']
        vel_launch_clear[i, j]  = res['vel_launch_clear']
        time_apogee[i, j]       = res['time_apogee']
        alt_apogee[i, j]        = res['alt_apogee']
        time_land_hard[i, j]    = res['time_land_hard']
        time_land_soft[i, j]    = res['time_land_soft']
        mach_max[i, j]          = res['mach_max']
        vel_air_max[i, j]       = res['vel_air_max']
        pos_land_hard[i, j]     = res['pos_land_hard']
        pos_land_soft[i, j]     = res['pos_land_soft']
        if param.payload.exist:
            pos_land_payload[i, j] = res['pos_land_payload']

    pd.DataFrame(time_launch_clear  , index=index_list, columns=header_list).to_csv(path + os.sep + 'time_launch_clear' + '.csv')
    pd.DataFrame(vel_launch_clear   , index=index_list, columns=header_list).to_csv(path + os.sep + 'vel_launch_clear' + '.csv')
    pd.DataFrame(time_apogee        , index=index_list, columns=header_list).to_csv(path + os.sep + 'time_apogee' + '.csv')
    pd.DataFrame(alt_apogee         , index=index_list, columns=header_list).to_csv(path + os.sep + 'alt_apogee' + '.csv')
    pd.DataFrame(time_land_hard     , index=index_list, columns=header_list).to_csv(path + os.sep + 'time_land_hard' + '.csv')
    pd.DataFrame(time_land_soft     , index=index_list, columns=header_list).to_csv(path + os.sep + 'time_land_soft' + '.csv')
    pd.DataFrame(mach_max           , index=index_list, columns=header_list).to_csv(path + os.sep + 'mach_max' + '.csv')
    pd.DataFrame(vel_air_max        , index=index_list, columns=header_list).to_csv(path + os.sep + 'vel_air_max' + '.csv')

    # 落下地点
    output_land_map(path, 'trajectory', param.launch.LLH, pos_land_hard, speed, azimuth, simplekml.Color.orange)
    output_land_map(path, 'parachute' , param.launch.LLH, pos_land_soft, speed, azimuth, simplekml.Color.aqua)
    if param.payload.exist:
        output_land_map(path, 'payload', param.launch.LLH, pos_land_payload, speed, azimuth, simplekml.Color.crimson)

if __name__=='__main__':

    print('Hello World!')
    # __debug()
