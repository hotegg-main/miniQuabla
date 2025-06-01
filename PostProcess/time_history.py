import numpy as np
import quaternion
from dynamics import *
import matplotlib.pyplot as plt
import os

# グラフの描画設定
plt.rcParams['font.family'] = 'Arial'
plt.rcParams['font.size']   = 12
plt.rcParams['figure.titlesize'] = 13
plt.rcParams["xtick.direction"]   = "in"
plt.rcParams["ytick.direction"]   = "in"
plt.rcParams["xtick.top"]         = True
plt.rcParams["ytick.right"]       = True
plt.rcParams["xtick.major.width"] = 1.5
plt.rcParams["ytick.major.width"] = 1.5
plt.rcParams["axes.linewidth"] = 1.5

def calc_sub_values(path, time, pos, vel, quat, omega, mass, param):

    quat_new    = quaternion.from_float_array(quat)
    dcm         = quaternion.as_rotation_matrix(quat_new)
    dcmT        = np.array([d.T for d in dcm])
    euler = np.rad2deg(np.array([np.array([np.arctan2(d[0, 1], d[0, 0]), np.arcsin(-d[0, 2]), np.arctan2(d[1, 2], d[2, 2])]) for d in dcmT]))

    alt_log = np.abs(pos[:, 2])
    downrange           = np.array([np.linalg.norm(p[:2]) for p in pos])
    grav, rho, cs       = param.atmos.get_atmosphere(alt_log)
    vel_NED             = np.array([d @ v for d, v in zip(dcm, vel)])
    wind_NED            = np.array([param.wind.get_wind_NED(alt) for alt in alt_log])
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
    Ij = param.geomet.get_Ij(time).T
    area_ref        = param.geomet.area
    length_ref      = param.geomet.length
    diameter_ref    = param.geomet.diameter

    moment_aero         = np.array([calc_aero_moment(lg, lp, fa) for lg, lp, fa in zip(lcg, lcp, force_aero)])
    moment_aero_damp    = np.array([calc_aero_damping_moment(dp, v, o, coeff_lp, coeff_mq, coeff_nr, area_ref, length_ref, diameter_ref) for dp, v, o in zip(dynamic_pressure, vel_air_abs, omega)])
    moment_gyro         = np.array([calc_gyro_moment(o, i) for o, i in zip(omega, Ij)])
    omega_dot           = calc_omega_dot(moment_aero, moment_aero_damp, moment_gyro, Ij)

    ########################################
    # Summary
    ########################################
    Fst_max, Fst_min    = calc_max_min_Fst(Fst)
    index_launch_clear  = calc_index_launch_clear(pos, dcm, param)
    index_apogee        = calc_index_apogee(pos)
    index_max_air_speed = calc_max_air_speed(vel_air_abs)
    index_max_Q         = calc_max_Q(dynamic_pressure)
    index_max_mach      = calc_max_mach(mach)

    aoa[:index_launch_clear] = 0.
    acc_body[:index_launch_clear, 1:3] = 0.

    lines_summary = [
        'Max. Fst [%]: ', str(round(Fst_max, 3)), '\n',
        'Min. Fst [%]: ', str(round(Fst_min, 3)), '\n',
        '\n',
        'Launch Clear, Time [sec]           : ', str(round(time[index_launch_clear], 3)), '\n'
        'Launch Clear, Velocity [m/s]       : ', str(round(vel_air_abs[index_launch_clear], 3)), '\n'
        'Launch Clear, Acceleration [m/s^2] : ', str(round(np.linalg.norm(acc_body[index_launch_clear]), 3)), '\n'
        'Launch Clear, Acceleration [G]     : ', str(round(np.linalg.norm(acc_body[index_launch_clear]) / grav[index_launch_clear], 3)), '\n'
        '\n',
        'Apogee, Time [sec]     : ', str(round(time[index_apogee], 3)), '\n'
        'Apogee, Altitude [km]  : ', str(round(alt_log[index_apogee] * 1.e-03, 3)), '\n'
        'Apogee, Dwonrange [km] : ', str(round(downrange[index_apogee] * 1.e-03, 3)), '\n'
        'Apogee, Air Speed [m/s]: ', str(round(vel_air_abs[index_apogee], 3)), '\n'
        '\n',
        'Landing(Hard), Time [sec]      : ', str(round(time[-1], 3)), '\n'
        'Landing(Hard), Downrange [km]  : ', str(round(downrange[-1] * 1.e-03, 3)), '\n'
        # 'Landing(Hard), NED [m, m]      : ', str(round(downrange[-1] * 1.e-03, 3)), '\n'
    ]

    with open(path + os.sep + 'summary.txt', mode='w', encoding='utf-8') as f:
        f.writelines(lines_summary)

    ########################################
    # Plot
    ########################################
    plot_euler(path, time, euler)                           # オイラー角
    plot_force_aero(path, time, force_aero)                 # 空気力
    plot_acc_body(path, time, acc_body)                     # 機体軸系加速度
    plot_vel_air(path, time, vel_air)                       # 対気速度
    plot_dynamic_pressure(path, time, dynamic_pressure)     # 動圧
    plot_aoa_aos(path, time, aoa)                           # 迎角、横滑り角
    plot_safety_mach(path, time, mach)                      # マッハ数
    plot_static_margin(path, time, Fst)                     # Fst
    plot_atomosphere(path, time, grav, rho, cs)             # 
    plot_CG_CP(path, time, lcg, lcp)                        # 重心、圧力中心
    plot_coefficient_aero(path, time, coeff_A, coeff_Na)    # 
    plot_moment_aero(path, time, moment_aero)               #
    plot_moment_aero_damping(path, time, moment_aero_damp)  #
    plot_moment_gyro(path, time, moment_gyro)               #
    plot_moment(path, time, moment_aero, moment_aero_damp, moment_gyro)
    plot_omega_dot(path, time, omega_dot)

def plot_main_values(path, time, pos, vel, quat, omega, mass, param):

    plot_mass(path, time, mass)
    plot_pos(path, time, pos)
    plot_vel(path, time, vel)
    plot_omega(path, time, omega)
    plot_quternion(path, time, quat)

#####################################################################
# Subroutine 
#####################################################################

def plot_mass(path, time, mass):

    plt.figure('Mass')
    plt.plot(time, mass)
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Mass [kg]')
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Mass' + '.png')
    plt.close()

def plot_pos(path, time, pos):

    plt.figure('Position')
    plt.plot(time, pos[:, 0], label='North')
    plt.plot(time, pos[:, 1], label='East')
    plt.plot(time, pos[:, 2], label='Down')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Distance [m]')
    plt.legend()
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Position' + '.png')
    plt.close()

def plot_vel(path, time, vel):

    plt.figure('Velocity')
    plt.plot(time, vel[:, 0], label='Roll')
    plt.plot(time, vel[:, 1], label='Pitch')
    plt.plot(time, vel[:, 2], label='Yaw')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Speed [m/s]')
    plt.legend()
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Velocity' + '.png')
    plt.close()

def plot_omega(path, time, omega):

    plt.figure('Anguler_Speed')
    plt.plot(time, omega[:, 0], label='Roll')
    plt.plot(time, omega[:, 1], label='Pitch')
    plt.plot(time, omega[:, 2], label='Yaw')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Anguler Speed [rad/s]')
    plt.legend()
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Anguler_Speed' + '.png')
    plt.close()

def plot_quternion(path, time, quat):

    norm = np.array([np.linalg.norm(q) for q in quat])
    plt.figure('Quaternion')
    plt.plot(time, quat[:, 0], label='q0')
    plt.plot(time, quat[:, 1], label='q1')
    plt.plot(time, quat[:, 2], label='q2')
    plt.plot(time, quat[:, 3], label='q3')
    plt.plot(time, norm, label='norm')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Quaternion [-]')
    plt.legend()
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Quaternion' + '.png')
    plt.close()

def plot_euler(path, time, euler):

    plt.figure('Euler_Angle')
    plt.plot(time, euler[:, 0], label='Azimuth')
    plt.plot(time, euler[:, 1], label='Elevation')
    plt.plot(time, euler[:, 2], label='Roll')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Angle [deg]')
    plt.legend()
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Euler_Angle' + '.png')
    plt.close()

def plot_force_aero(path, time, force):

    plt.figure('Force_aero')
    plt.plot(time, force[:, 0], label='Drag')
    plt.plot(time, force[:, 1], label='Side Force')
    plt.plot(time, force[:, 2], label='Normal Force')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Force [N]')
    plt.legend()
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Force_aero' + '.png')
    plt.close()

def plot_acc_body(path, time, acc):
    
    plt.figure('Acc_body')
    plt.plot(time, acc[:, 0], label='Roll')
    plt.plot(time, acc[:, 1], label='Pitch')
    plt.plot(time, acc[:, 2], label='Yaw')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Acceleration [$m/s^2$]')
    plt.legend()
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Acc_body' + '.png')
    plt.close()

def plot_vel_air(path, time, vel_air):
    
    plt.figure('Air_speed')
    plt.plot(time, vel_air[:, 0], label='Roll')
    plt.plot(time, vel_air[:, 1], label='Pitch')
    plt.plot(time, vel_air[:, 2], label='Yaw')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Speed [m/s]')
    plt.legend()
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Air_speed' + '.png')
    plt.close()

def plot_aoa_aos(path, time, aoa):
    
    plt.figure('AoA_AoS')
    plt.plot(time, np.rad2deg(aoa[:, 0]), label='Angle of Attack')
    plt.plot(time, np.rad2deg(aoa[:, 1]), label='Angle of Side-Slip')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Angle [deg]')
    # plt.ylim(bottom=-20., top=20.)
    plt.legend()
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'AoA_AoS' + '.png')
    plt.close()

def plot_dynamic_pressure(path, time, dynamic_pressure):

    plt.figure('Dynamic_Pressure')
    plt.plot(time, dynamic_pressure * 1.e-03)
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Pressure [kPa]')
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Dynamic_Pressure' + '.png')
    plt.close()

def plot_safety_mach(path, time, mach):

    plt.figure('Mach_Numpber')
    plt.plot(time, mach)
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Mach Numpber [-]')
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Mach_Numpber' + '.png')
    plt.close()

def plot_CG_CP(path, time, lcg, lcp):

    plt.figure('CG_CP')
    plt.plot(time, lcg, label='C.G.')
    plt.plot(time, lcp, label='C.P.')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Length from End [m]')
    plt.minorticks_on()
    plt.legend()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'CG_CP' + '.png')
    plt.close()

def plot_static_margin(path, time, fst):

    plt.figure('Static_Margin')
    plt.plot(time, fst)
    plt.xlim(left=0., right=time[-1])
    plt.ylim(bottom=5., top=25.)
    plt.xlabel('Time [sec]')
    plt.ylabel('Static Margin [-]')
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Static_Margin' + '.png')
    plt.close()

def plot_atomosphere(path, time, g, rho, cs):

    plt.figure('Atmosphere')
    
    plt.subplot(311)
    plt.plot(time, g)
    plt.xlim(left=0., right=time[-1])
    plt.ylabel('Gravity [$m/s^2$]')
    plt.minorticks_on()
    plt.grid(linestyle='--')
    
    plt.subplot(312)
    plt.plot(time, rho)
    plt.xlim(left=0., right=time[-1])
    plt.ylabel('Air Density [$kg/m^3$]')
    plt.minorticks_on()
    plt.grid(linestyle='--')
    
    plt.subplot(313)
    plt.plot(time, cs)
    plt.xlim(left=0., right=time[-1])
    plt.ylabel('Sound Speed [m/s]')
    plt.minorticks_on()
    plt.grid(linestyle='--')
    
    plt.xlabel('Time [sec]')

    plt.savefig(path + os.sep + 'Atmosphere' + '.png')

def plot_moment_aero(path, time, moment):

    plt.figure('Moment_aero')
    plt.plot(time, moment[:, 0], label='Roll')
    plt.plot(time, moment[:, 1], label='Pitch')
    plt.plot(time, moment[:, 2], label='Yaw')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Moment [Nm]')
    plt.minorticks_on()
    plt.legend()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Moment_aero' + '.png')
    plt.close()

def plot_moment_aero_damping(path, time, moment):

    plt.figure('Moment_aero_damp')
    plt.plot(time, moment[:, 0], label='Roll')
    plt.plot(time, moment[:, 1], label='Pitch')
    plt.plot(time, moment[:, 2], label='Yaw')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Moment [Nm]')
    plt.minorticks_on()
    plt.legend()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Moment_aero_damp' + '.png')
    plt.close()

def plot_moment_gyro(path, time, moment):

    plt.figure('Moment_gyro')
    plt.plot(time, moment[:, 0], label='Roll')
    plt.plot(time, moment[:, 1], label='Pitch')
    plt.plot(time, moment[:, 2], label='Yaw')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Moment [Nm]')
    plt.minorticks_on()
    plt.legend()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Moment_gyro' + '.png')
    plt.close()

def plot_moment(path, time, moment_aero, moment_aero_damp, moment_gyro):

    plt.figure('Moment-X')
    plt.plot(time, moment_aero[:, 0]        , label='Aero')
    plt.plot(time, moment_aero_damp[:, 0]   , label='Aero Damp.')
    plt.plot(time, moment_gyro[:, 0]        , label='Gyro')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Moment [Nm]')
    plt.minorticks_on()
    plt.legend()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Moment-X' + '.png')
    plt.close()
    
    plt.figure('Moment-Y')
    plt.plot(time, moment_aero[:, 1]        , label='Aero')
    plt.plot(time, moment_aero_damp[:, 1]   , label='Aero Damp.')
    plt.plot(time, moment_gyro[:, 1]        , label='Gyro')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Moment [Nm]')
    plt.minorticks_on()
    plt.legend()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Moment-Y' + '.png')
    plt.close()
    
    plt.figure('Moment-Z')
    plt.plot(time, moment_aero[:, 2]        , label='Aero')
    plt.plot(time, moment_aero_damp[:, 2]   , label='Aero Damp.')
    plt.plot(time, moment_gyro[:, 2]        , label='Gyro')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Moment [Nm]')
    plt.minorticks_on()
    plt.legend()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Moment-Z' + '.png')
    plt.close()

def plot_omega_dot(path, time, omega_dot):

    plt.figure('Omega_dot')
    plt.plot(time, omega_dot[:, 0], label='Roll')
    plt.plot(time, omega_dot[:, 1], label='Pitch')
    plt.plot(time, omega_dot[:, 2], label='Yaw')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Angular Acceleration [$rad/s^2$]')
    plt.minorticks_on()
    plt.legend()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Omega_dot' + '.png')
    plt.close()

def plot_coefficient_aero(path, time, coeff_A, coeff_Na):

    fig, ax1 = plt.subplots()
    ax1.set_title('Coefficient_Aero')
    ax2 = ax1.twinx()
    ax1.plot(time, coeff_A , label='$C_D$')
    ax1.plot(time, coeff_A, label='$C_{Na}$')
    ax2.plot(time, coeff_Na, label='$C_{Na}$')
    ax1.set_xlabel('Time [sec]')
    ax1.set_ylabel('Drag Coefficient [-]')
    ax2.set_ylabel('Slope of Normal Force Coefficient [1/rad]')
    ax1.set_xlim(left=0., right=time[-1])
    ax1.set_aspect('auto')
    ax1.grid()
    ax1.minorticks_on()
    ax1.legend()
    fig.savefig(path + os.sep + 'Coefficient_Aero' + '.png')
    plt.close()

def calc_max_min_Fst(fst):

    index_max = np.argmax(fst)
    fst_max = fst[index_max]
    index_min = np.argmin(fst)
    fst_min = fst[index_min]

    return fst_max, fst_min

def calc_index_launch_clear(pos, dcm, param):
    ''''''
    distance = np.array([(d.T @ p)[0] for d, p in zip(dcm, pos)])
    return np.argmax(distance - param.launch.length > 0.)

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

def calc_event_vaule(path, time, pos, vel, quat, omega, mass, param):
    '''
    Static Margin
    Launch Clear
    Apogee
    Max Air
    Max-Q
    Max Mach
    Max Normal
    Max Side
    Parachute Open
    Landing Trajectory
    Landing Parachute
    '''