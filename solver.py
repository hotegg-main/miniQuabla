from scipy.integrate import solve_ivp, odeint
import numpy as np
import quaternion
from Parameter.parameter import Parameter
import time
import simplekml
from PostProcess.directory_manage import make_result_directory

def run_single(path_config, path_result_src, name_case):

    from PostProcess.time_history import calc_sub_values
    from PostProcess.time_history import plot_main_values

    param = Parameter(path_config)
    
    elp = []
    elp.append(time.time())
    
    time_log, pos_log, vel_log, quat_log, omega_log, mass_log, time_log_para, pos_log_para, index \
        = solve_dynamics(param)
    
    if param.payload.exist:
        # ペイロード放出あり
        time_log, pos_log, vel_log, quat_log, omega_log, mass_log, time_log_payload, pos_log_payload \
            = solve_dynamics_payload(param, time_log, pos_log, vel_log, quat_log, omega_log, mass_log, index)
        
    else:
        time_log_payload    = None
        pos_log_payload     = None
    
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
    from PostProcess.land_map import output_land_map
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

    speed_array   = np.linspace(speed_sta, speed_sta + col*speed_step, col+1)
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
    
    elp.append(time.time())
    print('Elapse:', np.round(elp[1] - elp[0], 3), 'sec')

    dummy = Parameter(path_config)
    
    result_hard = np.zeros((col, row, 3))
    result_soft = np.zeros((col, row, 3))
    result_payload = np.zeros((col, row, 3))
    for result in result_list:
        i = result['Column']    
        j = result['Row']    
        result_hard[i][j] = result['Pos_hard']
        result_soft[i][j] = result['Pos_soft']
        if dummy.payload.exist:
            result_payload[i][j] = result['Pos_payload']

    output_land_map(path_result, 'trajectory', dummy.launch.LLH, dummy.launch.mag_dec, result_hard, speed_array, azimuth_array, simplekml.Color.orange)
    output_land_map(path_result, 'parachute' , dummy.launch.LLH, dummy.launch.mag_dec, result_soft, speed_array, azimuth_array, simplekml.Color.aqua)
    if dummy.payload.exist:
        output_land_map(path_result, 'payload', dummy.launch.LLH, dummy.launch.mag_dec, result_payload, speed_array, azimuth_array, simplekml.Color.crimson)
    
    print(' ---> Simulation END Successfully!')

def solve_dynamics(param: Parameter):
    '''
    Return:
        time_log        :時間の時間履歴
        pos_log         :位置のの時間履歴
        vel_log         :速度の時間履歴
        quat_log        :クォータニオンの時間履歴
        omega_log       :角速度の時間履歴
        mass_log        :質量の時間履歴
        time_log_para   :時間の時間履歴（減速落下）
        pos_log_para    :位置の時間履歴（減速落下）
        index           :パラシュート放出時のインデックス
    '''
    from dynamics import dynamics_trajectory, dynamics_parachute, event_land, event_land_soft

    # Trajectory
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
    
    result = solve_ivp(dynamics_trajectory, t_span=(time_sta, param.t_max), y0=x0, args=(param, ), events=event_land, dense_output=True, rtol=1.e-05, t_eval=t_eval)

    time_log  = result.t
    pos_log   = result.y[0:3].T
    vel_log   = result.y[3:6].T
    quat_log  = result.y[6:10].T
    omega_log = result.y[10:13].T
    mass_log  = result.y[13]

    index_apogee, _, _ = check_apogee(time_log, pos_log)
    
    # Parachute
    time0 = time_log[index_apogee]
    x0 = np.zeros(3)
    x0[0:3] = pos_log[index_apogee]

    event_land_soft.terminal = True
    t_eval = np.arange(time0, param.t_max, param.dt * 10.)
    result = solve_ivp(dynamics_parachute, t_span=(time0, param.t_max), t_eval=t_eval, y0=x0, args=(param, param.para, ), events=event_land_soft, rtol=1.e-06, atol=1.e-04)
    time_log_para = result.t
    pos_log_para  = result.y[0:3].T

    return time_log, pos_log, vel_log, quat_log, omega_log, mass_log, \
        time_log_para, pos_log_para, index_apogee

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

    param = Parameter(path)
    param.wind.set_power_model(job['Wind Speed'], job['Wind Azimuth'])

    res = dict()

    if param.payload.exist:
        # ペイロード放出あり
        
        time_log, pos_log, vel_log, quat_log, omega_log, mass_log, _, pos_log_para, index \
            = solve_dynamics(param)
        
        time_log, pos_log, vel_log, quat_log, omega_log, mass_log, _, pos_log_payload \
            = solve_dynamics_payload(param, time_log, pos_log, vel_log, quat_log, omega_log, mass_log, index)
        
        res['Pos_payload'] = pos_log_payload[-1]

    else:

        _, pos_log, _, _, _, _, _, pos_log_para, _ \
            = solve_dynamics(param)
    
    res['Column']   = job['Column']
    res['Row']      = job['Row']
    res['Pos_hard'] = pos_log[-1]
    res['Pos_soft'] = pos_log_para[-1]
    result.append(res)

def check_apogee(time_log, pos_log):

    index = np.argmax( - pos_log[:, 2])
    time_apogee = time_log[index]
    pos_apogee  = pos_log[index]

    return index, time_apogee, pos_apogee

if __name__=='__main__':

    print('Hello World!')
    # __debug()
