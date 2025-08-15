import matplotlib.pyplot as plt
import os
import numpy as np

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

color = ['#FF4B00',
         '#005AFF',
         '#03AF7A',
         '#4DC4FF',
         '#F6AA00',
         ]

def plot_mass(path, time, mass):

    plt.figure('Mass')
    plt.plot(time, mass, color=color[0])
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Mass [kg]')
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Mass' + '.png')
    plt.close()

def plot_pos(path, time, pos):

    plt.figure('Position')
    plt.plot(time, pos[:, 0], color[0], label='North')
    plt.plot(time, pos[:, 1], color[1], label='East')
    plt.plot(time, pos[:, 2], color[2], label='Down')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Distance [m]')
    plt.legend()
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Position' + '.png')
    plt.close()

def plot_pos_para(path, time, pos, time_para, pos_para):

    index = np.argmax(time >= time_para[0])

    plt.figure('Position')
    plt.plot(time[:index], pos[:index, 0], color=color[0], label='North')
    plt.plot(time[:index], pos[:index, 1], color=color[1], label='East')
    plt.plot(time[:index], pos[:index, 2], color=color[2], label='Down')
    plt.plot(time_para, pos_para[:, 0], color=color[0], linestyle='--')
    plt.plot(time_para, pos_para[:, 1], color=color[1], linestyle='--')
    plt.plot(time_para, pos_para[:, 2], color=color[2], linestyle='--')
    plt.xlim(left=0., right=time_para[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Distance [m]')
    plt.legend()
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Position_soft' + '.png')
    plt.close()

def plot_pos_payload(path, time, pos):

    plt.figure('Position')
    plt.plot(time[:], pos[:, 0], color=color[0], label='North')
    plt.plot(time[:], pos[:, 1], color=color[1], label='East')
    plt.plot(time[:], pos[:, 2], color=color[2], label='Down')
    plt.xlim(left=time[0], right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Distance [m]')
    plt.legend()
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Position_payload' + '.png')
    plt.close()

def plot_vel(path, time, vel):

    plt.figure('Velocity')
    plt.plot(time, vel[:, 0], color=color[0], label='Roll')
    plt.plot(time, vel[:, 1], color=color[1], label='Pitch')
    plt.plot(time, vel[:, 2], color=color[2], label='Yaw')
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
    plt.plot(time, omega[:, 0], color=color[0], label='Roll')
    plt.plot(time, omega[:, 1], color=color[1], label='Pitch')
    plt.plot(time, omega[:, 2], color=color[2], label='Yaw')
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
    plt.plot(time, quat[:, 0], color=color[0], label='q0')
    plt.plot(time, quat[:, 1], color=color[1], label='q1')
    plt.plot(time, quat[:, 2], color=color[2], label='q2')
    plt.plot(time, quat[:, 3], color=color[3], label='q3')
    plt.plot(time, norm, color=color[4], label='norm')
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
    plt.plot(time, euler[:, 0], color=color[0], label='Azimuth')
    plt.plot(time, euler[:, 1], color=color[1], label='Elevation')
    plt.plot(time, euler[:, 2], color=color[2], label='Roll')
    plt.xlim(left=0., right=time[-1])
    plt.ylim(bottom=-180., top=180.)
    plt.xlabel('Time [sec]')
    plt.ylabel('Angle [deg]')
    plt.legend()
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Euler_Angle' + '.png')
    plt.close()

def plot_force_aero(path, time, force):

    plt.figure('Force_aero')
    plt.plot(time, force[:, 0], color=color[0], label='Drag')
    plt.plot(time, force[:, 1], color=color[1], label='Side Force')
    plt.plot(time, force[:, 2], color=color[2], label='Normal Force')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Force [N]')
    plt.legend()
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Force_aero' + '.png')
    plt.close()

def plot_force_thrust(path, time, force):

    plt.figure('Force_thrust')
    plt.plot(time, force[:, 0], color=color[0])
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Force [N]')
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Force_thrust' + '.png')
    plt.close()

def plot_acc_body(path, time, acc):
    
    plt.figure('Acc_body')
    plt.plot(time, acc[:, 0], color=color[0], label='Roll')
    plt.plot(time, acc[:, 1], color=color[1], label='Pitch')
    plt.plot(time, acc[:, 2], color=color[2], label='Yaw')
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
    plt.plot(time, vel_air[:, 0], color=color[0], label='Roll')
    plt.plot(time, vel_air[:, 1], color=color[1], label='Pitch')
    plt.plot(time, vel_air[:, 2], color=color[2], label='Yaw')
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
    plt.plot(time, np.rad2deg(aoa[:, 0]), color=color[0], label='Angle of Attack')
    plt.plot(time, np.rad2deg(aoa[:, 1]), color=color[1], label='Angle of Side-Slip')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Angle [deg]')
    plt.legend()
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'AoA_AoS' + '.png')
    plt.close()

def plot_dynamic_pressure(path, time, dynamic_pressure):

    plt.figure('Dynamic_Pressure')
    plt.plot(time, dynamic_pressure * 1.e-03, color=color[0])
    plt.xlim(left=0., right=time[-1])
    plt.ylim(bottom=0.)
    plt.xlabel('Time [sec]')
    plt.ylabel('Pressure [kPa]')
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Dynamic_Pressure' + '.png')
    plt.close()

def plot_mach(path, time, mach):

    plt.figure('Mach_Numpber')
    plt.plot(time, mach, color=color[0])
    plt.xlim(left=0., right=time[-1])
    plt.ylim(bottom=0.)
    plt.xlabel('Time [sec]')
    plt.ylabel('Mach Numpber [-]')
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Mach_Numpber' + '.png')
    plt.close()

def plot_CG_CP(path, time, lcg, lcp):

    plt.figure('CG_CP')
    plt.plot(time, lcg      , label='C.G.', color=color[0])
    plt.plot(time, lcp      , label='C.P.', color=color[1])
    # plt.plot(time, lcg_prop , label='C.G. (Prop.)', color=color[2])
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
    plt.plot(time, fst, color=color[0])
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
    plt.plot(time, g, color=color[0])
    plt.xlim(left=0., right=time[-1])
    plt.ylabel('Gravity [$m/s^2$]')
    plt.minorticks_on()
    plt.grid(linestyle='--')
    
    plt.subplot(312)
    plt.plot(time, rho, color=color[1])
    plt.xlim(left=0., right=time[-1])
    plt.ylabel('Air Density [$kg/m^3$]')
    plt.minorticks_on()
    plt.grid(linestyle='--')
    
    plt.subplot(313)
    plt.plot(time, cs, color=color[2])
    plt.xlim(left=0., right=time[-1])
    plt.ylabel('Sound Speed [m/s]')
    plt.minorticks_on()
    plt.grid(linestyle='--')
    
    plt.xlabel('Time [sec]')

    plt.savefig(path + os.sep + 'Atmosphere' + '.png')

def plot_moment_aero(path, time, moment):

    plt.figure('Moment_aero')
    plt.plot(time, moment[:, 0], color=color[0], label='Roll')
    plt.plot(time, moment[:, 1], color=color[1], label='Pitch')
    plt.plot(time, moment[:, 2], color=color[2], label='Yaw')
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
    plt.plot(time, moment[:, 0], color=color[0], label='Roll')
    plt.plot(time, moment[:, 1], color=color[1], label='Pitch')
    plt.plot(time, moment[:, 2], color=color[2], label='Yaw')
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
    plt.plot(time, moment[:, 0], color=color[0], label='Roll')
    plt.plot(time, moment[:, 1], color=color[1], label='Pitch')
    plt.plot(time, moment[:, 2], color=color[2], label='Yaw')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Moment [Nm]')
    plt.minorticks_on()
    plt.legend()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Moment_gyro' + '.png')
    plt.close()

def plot_moment(path, time, moment_aero, moment_aero_damp, moment_gyro):

    plt.figure('Moment')

    plt.subplot(311)
    plt.plot(time, moment_aero[:, 0]        , color=color[0], label='Aero')
    plt.plot(time, moment_aero_damp[:, 0]   , color=color[1], label='Aero Damp.')
    plt.plot(time, moment_gyro[:, 0]        , color=color[2], label='Gyro')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Moment-X [Nm]')
    plt.minorticks_on()
    plt.legend(bbox_to_anchor=(0.99, 1), loc='upper left', fontsize=10)
    plt.grid(linestyle='--')
    
    plt.subplot(312)
    plt.plot(time, moment_aero[:, 1]        , color=color[0], label='Aero')
    plt.plot(time, moment_aero_damp[:, 1]   , color=color[1], label='Aero Damp.')
    plt.plot(time, moment_gyro[:, 1]        , color=color[2], label='Gyro')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Moment-Y [Nm]')
    plt.minorticks_on()
    plt.legend(bbox_to_anchor=(0.99, 1), loc='upper left', fontsize=10)
    plt.grid(linestyle='--')
    
    plt.subplot(313)
    plt.plot(time, moment_aero[:, 2]        , color=color[0], label='Aero')
    plt.plot(time, moment_aero_damp[:, 2]   , color=color[1], label='Aero Damp.')
    plt.plot(time, moment_gyro[:, 2]        , color=color[2], label='Gyro')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Moment-Z [Nm]')
    plt.minorticks_on()
    plt.legend(bbox_to_anchor=(0.99, 1), loc='upper left', fontsize=10)
    plt.grid(linestyle='--')

    plt.subplots_adjust(right=0.8)
    plt.savefig(path + os.sep + 'Moment' + '.png')

def plot_moment_of_inertia(path, time, moi):

    plt.figure('Moment of Inertia')

    plt.subplot(211)
    plt.plot(time, moi[:, 1], color=color[0], label='Pitch')
    plt.plot(time, moi[:, 2], color=color[1], label='Yaw')
    plt.xlim(left=0., right=time[-1])
    plt.ylabel('Moment of Inertia [$kg m^2$]')
    plt.minorticks_on()
    plt.legend()
    plt.grid(linestyle='--')
    
    plt.subplot(212)
    plt.plot(time, moi[:, 0], color=color[2], label='Roll')
    plt.xlim(left=0., right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Moment of Inertia [$kg m^2$]')
    plt.minorticks_on()
    plt.legend()
    plt.grid(linestyle='--')
    
    plt.savefig(path + os.sep + 'Moment_of_Inertia' + '.png')

def plot_omega_dot(path, time, omega_dot):

    plt.figure('Omega_dot')
    plt.plot(time, omega_dot[:, 0], color=color[0], label='Roll')
    plt.plot(time, omega_dot[:, 1], color=color[1], label='Pitch')
    plt.plot(time, omega_dot[:, 2], color=color[2], label='Yaw')
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
    ax1.plot(time   , coeff_A    , color=color[0], label='$C_D$')
    ax1.plot(time[0], coeff_A[0] , color=color[1], label='$C_{Na}$')
    ax2.plot(time   , coeff_Na   , color=color[1], label='$C_{Na}$')
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

def plot_vel_descent(path, time, vel):

    plt.figure('Static_Margin')
    plt.plot(time, vel, color=color[0])
    plt.xlim(left=time[0], right=time[-1])
    plt.xlabel('Time [sec]')
    plt.ylabel('Velocity [m/s]')
    plt.minorticks_on()
    plt.grid(linestyle='--')
    plt.savefig(path + os.sep + 'Velocity_descent' + '.png')
    plt.close()