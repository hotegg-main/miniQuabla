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
    grav, rho, cs  = param.atmos.get_atmosphere(alt_log)
    vel_NED     = np.array([d @ v for d, v in zip(dcm, vel)])
    wind_NED    = np.array([param.wind.get_wind_NED(alt) for alt in alt_log])
    vel_air     = np.array([d.T @ calc_air_speed(v, w) for d, v, w in zip(dcm, vel_NED, wind_NED)])
    vel_air_abs = np.array([np.linalg.norm(v) for v in vel_air])
    aoa         = np.array([calc_angle_of_attack(v) for v in vel_air])
    mach        = calc_mach_number(vel_air_abs, cs)
    dynamic_pressure = calc_dynamic_pressure(vel_air_abs, rho)
    coeff_A, coeff_Na = param.aero.get_coeffecient_force(mach)
    force_aero      = np.array([calc_aero_force(d, a, b, coeff_A, coeff_Na, param.geomet.area) for d, a, b in zip(dynamic_pressure, aoa[:, 0], aoa[:, 1])])
    force_thrust    = np.zeros((len(time), 3))
    force_thrust[:, 0] = param.engine.get_thrust(time)
    force_gravity   = ([d.T @ (m * np.array([0., 0., g])) for d, m, g in zip(dcm, mass, grav)])
    acc_body    = calc_acceleration(force_aero, force_thrust, force_gravity, np.array([mass, mass, mass]).T, vel, omega)
    lcg = param.geomet.get_Lcg(time)
    lcp = param.aero.get_Lcp(mach)
    Fst = (lcg - lcp) / param.geomet.length * 100.

    plot_euler(path, time, euler)
    plot_force_aero(path, time, force_aero)
    plot_acc_body(path, time, acc_body)

def plot_main_values(path, time, pos, vel, quat, omega, mass, param):

    plot_pos(path, time, pos)
    plot_vel(path, time, vel)
    plot_omega(path, time, omega)

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
