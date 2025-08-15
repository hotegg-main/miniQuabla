import numpy as np
import quaternion
from dynamics import *
import os
import pandas as pd
from Parameter.parameter import Parameter
import simplekml
from PostProcess.plot_graph import *

def calc_sub_values(path, time, pos, vel, quat, omega, mass, time_para, pos_para, param: Parameter):

    quat_new    = quaternion.from_float_array(quat)
    dcm         = quaternion.as_rotation_matrix(quat_new)
    dcmT        = np.array([d.T for d in dcm])
    euler       = np.rad2deg(np.array([np.array([np.arctan2(d[0, 1], d[0, 0]), np.arcsin(-d[0, 2]), np.arctan2(d[1, 2], d[2, 2])]) for d in dcmT]))

    altitude            = np.abs(pos[:, 2])
    downrange           = np.array([np.linalg.norm(p[:2]) for p in pos])
    grav, rho, cs       = param.atmos.get_atmosphere(altitude)
    vel_NED             = np.array([d @ v for d, v in zip(dcm, vel)])
    wind_NED            = np.array([param.wind.get_wind_NED(alt) for alt in altitude])
    vel_air             = np.array([d.T @ calc_air_speed(v, w) for d, v, w in zip(dcm, vel_NED, wind_NED)])
    vel_air_abs         = np.array([np.linalg.norm(v) for v in vel_air])
    aoa                 = np.array([calc_angle_of_attack(v) for v in vel_air])
    mach                = calc_mach_number(vel_air_abs, cs)
    dynamic_pressure    = calc_dynamic_pressure(vel_air_abs, rho)
    coeff_A, coeff_Na   = param.aero.get_coeffecient_force(mach)
    force_aero          = np.array([calc_aero_force(d, a, b, c_A, c_Na, param.geomet.area) for d, a, b, c_A, c_Na in zip(dynamic_pressure, aoa[:, 0], aoa[:, 1], coeff_A, coeff_Na)])
    force_thrust        = np.zeros((len(time), 3))
    force_thrust[:, 0]  = param.engine.get_thrust(time)
    force_gravity       = ([d.T @ (m * np.array([0., 0., g])) for d, m, g in zip(dcm, mass, grav)])
    acc_body            = calc_acceleration(force_aero, force_thrust, force_gravity, np.array([mass, mass, mass]).T, vel, omega)
    lcg                 = param.geomet.get_Lcg(time)
    lcp                 = param.aero.get_Lcp(mach)
    Fst                 = (lcg - lcp) / param.geomet.length * 100.

    coeff_lp = param.aero.coeff_lp
    coeff_mq = param.aero.coeff_mq
    coeff_nr = param.aero.coeff_nr
    Ij              = param.geomet.get_Ij(time).T
    area_ref        = param.geomet.area
    length_ref      = param.geomet.length
    diameter_ref    = param.geomet.diameter

    moment_aero         = np.array([calc_aero_moment(lg, lp, fa) for lg, lp, fa in zip(lcg, lcp, force_aero)])
    moment_aero_damp    = np.array([calc_aero_damping_moment(dp, v, o, coeff_lp, coeff_mq, coeff_nr, area_ref, length_ref, diameter_ref) for dp, v, o in zip(dynamic_pressure, vel_air_abs, omega)])
    moment_gyro         = np.array([calc_gyro_moment(o, i) for o, i in zip(omega, Ij)])
    omega_dot           = calc_omega_dot(moment_aero, moment_aero_damp, moment_gyro, Ij)

    altitude_para   = np.abs(pos_para[:, 2])
    downrange_para  = np.array([np.linalg.norm(p[:2]) for p in pos_para])
    vel_descent     = np.array([param.para.get_velocity(t, alt) for t, alt in zip(time_para, altitude_para)])

    ########################################
    # Summary
    ########################################
    Fst_max, Fst_min    = calc_max_min_Fst(Fst)
    index_launch_clear  = calc_index_launch_clear(pos, dcm, param, lcg)
    index_apogee        = calc_index_apogee(pos)
    index_max_air_speed = calc_max_air_speed(vel_air_abs)
    index_max_Q         = calc_max_Q(dynamic_pressure)
    index_max_mach      = calc_max_mach(mach)
    index_burn_act      = np.argmax(time >= param.engine.time_act)

    aoa[:index_launch_clear, 0] = 0.
    acc_body[:index_launch_clear, 1:3] = 0.

    ########################################
    # to CSV
    ########################################
    pd.DataFrame({

        'Time [sec]'                    : time,
        'North [m]'                     : pos[:, 0],
        'East [m]'                      : pos[:, 1],
        'Down [m]'                      : pos[:, 2],
        'Velocity_BODY-X [m/s]'         : vel[:, 0],
        'Velocity_BODY-Y [m/s]'         : vel[:, 1],
        'Velocity_BODY-Z [m/s]'         : vel[:, 2],
        'Omega_BODY-X [rad/s]'          : omega[:, 0],
        'Omega_BODY-Y [rad/s]'          : omega[:, 1],
        'Omega_BODY-Z [rad/s]'          : omega[:, 2],
        'Quaternion0'                   : quat[:, 0],
        'Quaternion1'                   : quat[:, 1],
        'Quaternion2'                   : quat[:, 2],
        'Quaternion3'                   : quat[:, 3],
        'Mass [kg]'                     : mass,
        'Altitude [km]'                 : altitude  * 1.E-03,
        'Downrange [km]'                : downrange * 1.E-03,
        'Gravity [m/s2]'                : grav, 
        'Air Density [kg/m3]'           : rho, 
        'Sound Speed [m/s]'             : cs,
        'Azimuth [deg]'                 : euler[:, 0],
        'Elevation [deg]'               : euler[:, 1],
        'Roll [deg]'                    : euler[:, 2],
        'Velocity_NED-X [m/s]'          : vel_NED[:, 0],
        'Velocity_NED-Y [m/s]'          : vel_NED[:, 1],
        'Velocity_NED-Z [m/s]'          : vel_NED[:, 2],
        'Wind_NED-X [m/s]'              : wind_NED[:, 0],
        'Wind_NED-Y [m/s]'              : wind_NED[:, 1],
        'Wind_NED-Z [m/s]'              : wind_NED[:, 2],
        'Air_Speed_BODY-X [m/s]'        : vel_air[:, 0],
        'Air_Speed_BODY-Y [m/s]'        : vel_air[:, 1],
        'Air_Speed_BODY-Z [m/s]'        : vel_air[:, 2],
        'Air_Speed_Norm [m/s]'          : vel_air_abs,
        'Angle of Attack [deg]'         : np.rad2deg(aoa[:, 0]),
        'Angle of Side-Slip [deg]'      : np.rad2deg(aoa[:, 1]),
        'Mach Numpber [-]'              : mach,
        'Dynamic Pressure [kPa]'        : dynamic_pressure * 1.E-03,
        'Axial Coefficient [-]'         : coeff_A,
        'Slope of Normal Force [1/rad]' : coeff_Na,
        'Axial Force [N]'               : - force_aero[:, 0],
        'Side Force [N]'                : - force_aero[:, 1],
        'Normal Force [N]'              : - force_aero[:, 2],
        'Thrust [N]'                    : force_thrust[:, 0],
        'Acceleration_BODY-X [m/s2]'    : acc_body[:, 0],
        'Acceleration_BODY-Y [m/s2]'    : acc_body[:, 1],
        'Acceleration_BODY-Z [m/s2]'    : acc_body[:, 2],
        'Length-C.G. from End [m]'      : lcg,
        'Length-C.P. from End [m]'      : lcp,
        'Static Magin Fst [%]'          : Fst,
        'Moment of Inertia-X [kg*m2]'   : Ij[:, 0],
        'Moment of Inertia-Y [kg*m2]'   : Ij[:, 1],
        'Moment of Inertia-Z [kg*m2]'   : Ij[:, 2],
        'Aero Moment-X [Nm]'            : moment_aero[:, 0],
        'Aero Moment-Y [Nm]'            : moment_aero[:, 1],
        'Aero Moment-Z [Nm]'            : moment_aero[:, 2],
        'Aero Damping Moment-X [Nm]'    : moment_aero_damp[:, 0],
        'Aero Damping Moment-Y [Nm]'    : moment_aero_damp[:, 1],
        'Aero Damping Moment-Z [Nm]'    : moment_aero_damp[:, 2],
        'Gyro Moment-X [Nm]'            : moment_gyro[:, 0],
        'Gyro Moment-Y [Nm]'            : moment_gyro[:, 1],
        'Gyro Moment-Z [Nm]'            : moment_gyro[:, 2],
        'Derivative of Omega-X [rad/s2]': omega_dot[:, 0],
        'Derivative of Omega-Y [rad/s2]': omega_dot[:, 1],
        'Derivative of Omega-Z [rad/s2]': omega_dot[:, 2],
    
    }).to_csv(path + os.sep + 'log.csv', index=False)


    lines_summary = [
        'Max. Fst [%]: ', str(round(Fst_max, 3)), '\n',
        'Min. Fst [%]: ', str(round(Fst_min, 3)), '\n',
        '\n',
        'Launch Clear, Time [sec]           : ', str(round(time[index_launch_clear], 3)), '\n'
        'Launch Clear, Velocity [m/s]       : ', str(round(vel[index_launch_clear, 0], 3)), '\n'
        'Launch Clear, Acceleration [m/s^2] : ', str(round(np.linalg.norm(acc_body[index_launch_clear]), 3)), '\n'
        'Launch Clear, Acceleration [G]     : ', str(round(np.linalg.norm(acc_body[index_launch_clear]) / grav[index_launch_clear], 3)), '\n'
        '\n',
        'Apogee, Time [sec]     : ', str(round(time[index_apogee], 3)), '\n'
        'Apogee, Altitude [km]  : ', str(round(altitude[index_apogee] * 1.e-03, 3)), '\n'
        'Apogee, Downrange [km] : ', str(round(downrange[index_apogee] * 1.e-03, 3)), '\n'
        'Apogee, Air Speed [m/s]: ', str(round(vel_air_abs[index_apogee], 3)), '\n'
        '\n',
        'Max Air Speed, Time [sec]        : ', str(round(time[index_max_air_speed], 3)), '\n'
        'Max Air Speed, Speed [m/s]       : ', str(round(vel_air_abs[index_max_air_speed], 3)), '\n'
        'Max Air Speed, Altitude [km]     : ', str(round(altitude[index_max_air_speed] * 1.e-03, 3)), '\n'
        '\n',
        'Max-Q, Time [sec]              : ', str(round(time[index_max_Q], 3)), '\n'
        'Max-Q, Dynamics Pressure [kPa] : ', str(round(dynamic_pressure[index_max_Q] * 1.e-03, 3)), '\n'
        'Max-Q, Altitude [km]           : ', str(round(altitude[index_max_Q] * 1.e-03, 3)), '\n'
        '\n',
        'Max Mach, Time [sec]       : ', str(round(time[index_max_mach], 3)), '\n'
        'Max Mach, Mach Number [-]  : ', str(round(mach[index_max_mach], 3)), '\n'
        'Max Mach, Altitude [km]    : ', str(round(altitude[index_max_mach] * 1.e-03, 3)), '\n'
        '\n',
        'Landing(Hard), Time [sec]        : ', str(round(time[-1], 3)), '\n'
        'Landing(Hard), Downrange [km]    : ', str(round(downrange[-1] * 1.e-03, 3)), '\n'
        'Landing(Hard), East, North [m, m]: ', str(round(pos[-1, 1], 3)), ', ', str(round(pos[-1, 0], 3)), '\n'
        '\n',
        'Landing(Soft), Time [sec]        : ', str(round(time_para[-1], 3)), '\n'
        'Landing(Soft), Downrange [km]    : ', str(round(downrange_para[-1] * 1.e-03, 3)), '\n'
        'Landing(Soft), East, North [m, m]: ', str(round(pos_para[-1, 1], 3)), ', ', str(round(pos_para[-1, 0], 3)), '\n'
    ]

    with open(path + os.sep + 'summary.txt', mode='w', encoding='utf-8') as f:
        f.writelines(lines_summary)

    ########################################
    # Graph
    ########################################
    plot_euler(path, time, euler)                           # オイラー角
    plot_force_aero(path, time, force_aero)                 # 空気力
    plot_force_thrust(path, time[:index_burn_act], force_thrust[:index_burn_act])             # 推力
    plot_acc_body(path, time, acc_body)                     # 機体軸系加速度
    plot_vel_air(path, time, vel_air)                       # 対気速度
    plot_dynamic_pressure(path, time, dynamic_pressure)     # 動圧
    plot_aoa_aos(path, time, aoa)                           # 迎角、横滑り角
    plot_mach(path, time, mach)                             # マッハ数
    plot_static_margin(path, time, Fst)                     # Fst
    plot_atomosphere(path, time, grav, rho, cs)             # 重力加速度、大気密度、音速
    plot_CG_CP(path, time, lcg, lcp)                        # 重心、圧力中心
    plot_coefficient_aero(path, time, coeff_A, coeff_Na)    # 安定微係数（空力係数）
    plot_moment_of_inertia(path, time, Ij)                  # 慣性モーメント
    plot_moment_aero(path, time, moment_aero)               # 空力モーメント
    plot_moment_aero_damping(path, time, moment_aero_damp)  # 空力減衰モーメント
    plot_moment_gyro(path, time, moment_gyro)               # ジャイロモーメント
    plot_moment(path, time, moment_aero, moment_aero_damp, moment_gyro)
    plot_omega_dot(path, time, omega_dot)                   # 角加速度
    plot_vel_descent(path, time_para, vel_descent)          # 降下速度

def calc_values_min(time, pos, vel, quat, omega, mass, time_para, pos_para, param: Parameter):
    '''
    ランチクリア時刻
    頂点到達時刻
    着地時刻
    最大到達高度
    最大マッハ数
    ランチクリア速度
    最大対気速度
    '''
    quat_new    = quaternion.from_float_array(quat)
    dcm         = quaternion.as_rotation_matrix(quat_new)

    altitude            = np.abs(pos[:, 2])
    _, _, cs       = param.atmos.get_atmosphere(altitude)
    vel_NED             = np.array([d @ v for d, v in zip(dcm, vel)])
    wind_NED            = np.array([param.wind.get_wind_NED(alt) for alt in altitude])
    vel_air             = np.array([d.T @ calc_air_speed(v, w) for d, v, w in zip(dcm, vel_NED, wind_NED)])
    vel_air_abs         = np.array([np.linalg.norm(v) for v in vel_air])
    mach                = calc_mach_number(vel_air_abs, cs)
    
    lcg                 = param.geomet.get_Lcg(time)

    # イベント発生時刻
    index_launch_clear  = calc_index_launch_clear(pos, dcm, param, lcg)
    index_apogee        = calc_index_apogee(pos)
    index_max_air_speed = calc_max_air_speed(vel_air_abs)
    index_max_mach      = calc_max_mach(mach)

    result = dict()

    # ランチクリア
    result['time_launch_clear'] = time[index_launch_clear]
    result['vel_launch_clear']  = vel[index_launch_clear, 0]
    
    # 頂点
    result['time_apogee'] = time[index_apogee]
    result['alt_apogee']  = altitude[index_apogee]
    
    # 着地（弾道）
    result['time_land_hard'] = time[-1]
    result['pos_land_hard']  = pos[-1]
    
    # 着地（減速）
    result['time_land_soft'] = time_para[-1]
    result['pos_land_soft']  = pos_para[-1]

    # 最大マッハ数
    result['mach_max'] = mach[index_max_mach]

    # 最大対気速度
    result['vel_air_max'] = vel_air_abs[index_max_air_speed]

    return result

def plot_main_values(path, param, time, pos, vel, quat, omega, mass, time_para, pos_para, exist_payload, time_payload=None, pos_payload=None):

    plot_mass(path, time, mass)
    plot_pos(path, time, pos)
    plot_vel(path, time, vel)
    plot_omega(path, time, omega)
    plot_quternion(path, time, quat)
    
    plot_pos_para(path, time, pos, time_para, pos_para)

    if exist_payload:
        plot_pos_payload(path, time_payload, pos_payload)
    
    plot_kml(path, exist_payload, param.launch.LLH, param.launch.mag_dec, pos, pos_para, pos_payload)

#####################################################################
# Subroutine 
#####################################################################

def plot_kml(path, exist_payload, launch_LLH, mag_dec, pos_hard, pos_soft, pos_payload):
    
    from PostProcess.coordinate import NED2LLHforKml
    
    llh_hard = [NED2LLHforKml(launch_LLH, pos) for pos in pos_hard]
    llh_soft = [NED2LLHforKml(launch_LLH, pos) for pos in pos_soft]
    
    kml = simplekml.Kml(open=1)
    
    __make_kml_linestring(kml, llh_hard, 'Trajectory', simplekml.Color.orange)
    __make_kml_linestring(kml, llh_soft, 'Parachute' , simplekml.Color.aqua)

    llh_land = []
    llh_land.append([llh_hard[-1][1], llh_hard[-1][0]])
    llh_land.append([llh_soft[-1][1], llh_soft[-1][0]])

    name_index = ['hard', 'soft']

    if exist_payload:
        llh_payload = [NED2LLHforKml(launch_LLH, pos) for pos in pos_payload]
        
        __make_kml_linestring(kml, llh_payload, 'Payload' , simplekml.Color.crimson)
        
        llh_land.append([llh_payload[-1][1], llh_payload[-1][0]])
        name_index.append('payload')

    kml.save(path + os.sep + 'flight_log' + '.kml')

    pd.DataFrame(llh_land, name_index, columns=['Latitude [deg]', 'Longtitude [deg]']).to_csv(path + os.sep + 'land_point.csv')

def __make_kml_linestring(kml, llh, name, color):

    line = kml.newlinestring(name=name)
    line.style.linestyle.width = 5
    line.style.linestyle.color = color
    line.extrude = 1
    line.altitudemode = simplekml.AltitudeMode.absolute
    
    coords = []
    i = 0
    for p in llh:
        if i%10 == 0:
            coords.append(p)
        i = i+1
    line.coords = coords

def calc_max_min_Fst(fst):

    index_max = np.argmax(fst)
    fst_max = fst[index_max]
    index_min = np.argmin(fst)
    fst_min = fst[index_min]

    return fst_max, fst_min

def calc_index_launch_clear(pos, dcm, param, lcg):
    ''''''
    distance = np.array([(d.T @ p)[0] for d, p in zip(dcm, pos)])
    return np.argmax(distance - lcg - param.launch.length > 0.)

def calc_index_apogee(pos):
    ''''''
    return np.argmax( - pos[:, 2])

def calc_max_air_speed(vel_air):
    ''''''
    return np.argmax(vel_air)

def calc_max_Q(dynamic_pressure):
    ''''''
    return np.argmax(dynamic_pressure)

def calc_max_mach(mach):
    ''''''
    return np.argmax(mach)