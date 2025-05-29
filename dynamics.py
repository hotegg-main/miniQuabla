import numpy as np
import quaternion
from Parameter.parameter import Parameter

def dynamics_trajectory(time, x, param:Parameter):
    '''
    Args:
        time    :時間
        pos     :位置(NED)
        vel     :速度(Body)
        quat    :クォータニオン
        omega   :角速度
        mass    :質量
    '''
    
    pos     = x[0:3]
    vel     = x[3:6]
    quat    = quaternion.from_float_array(x[6:10]).normalized()
    omega   = x[10:13]
    mass    = x[13]
    
    dcm = quaternion.as_rotation_matrix(quat)
    # altitude = np.min([0., - pos[2]])
    altitude = np.abs(pos[2])
    
    area_ref        = param.geomet.area
    length_ref      = param.geomet.length
    diameter_ref    = param.geomet.diameter
    mdot = param.engine.get_mass_flow_rate(time)
    lcg         = param.geomet.get_Lcg(time)
    Ij          = param.geomet.get_Ij(time)
    
    g, rho, cs  = param.atmos.get_atmosphere(altitude)
    wind_NED    = param.wind.get_wind_NED(altitude)
    
    vel_NED             = dcm @ vel
    vel_air_body        = dcm.T @ calc_air_speed(vel_NED, wind_NED)
    vel_air_abs         = np.linalg.norm(vel_air_body)
    alpha, beta         = calc_angle_of_attack(vel_air_body)
    mach                = calc_mach_number(vel_air_abs, cs)
    dynamic_pressure    = calc_dynamic_pressure(vel_air_abs, rho)

    coeff_axial, coeff_normal = param.aero.get_coeffecient_force(mach)
    coeff_lp = param.aero.coeff_lp
    coeff_mq = param.aero.coeff_mq
    coeff_nr = param.aero.coeff_nr

    lcp = param.aero.get_Lcp(mach)
    
    thrust = param.engine.get_thrust(time)
    
    # Force
    force_aero      = calc_aero_force(dynamic_pressure, alpha, beta, coeff_axial, coeff_normal, area_ref)
    force_thrust    = np.array([thrust, 0., 0.])
    force_gravity   = dcm.T @ (mass * np.array([0., 0., g]))

    # Moment
    moment_aero         = calc_aero_moment(lcg, lcp, force_aero)
    moment_aero_damp    = calc_aero_damping_moment(dynamic_pressure, vel_air_abs, omega, coeff_lp, coeff_mq, coeff_nr, area_ref, length_ref, diameter_ref)
    moment_gyro         = calc_gyro_moment(omega, Ij)

    acc_body    = calc_acceleration(force_aero, force_thrust, force_gravity, mass, vel, omega)
    omega_dot   = calc_omega_dot(moment_aero, moment_aero_damp, moment_gyro, Ij)
    quat_dot    = calc_quat_dot(omega, quat)

    if not is_launch_clear(pos, dcm, param.launch.length):

        if acc_body[0] < 0. and vel[0] < 0.:
            acc_body[0:3] = 0.

        acc_body[1:3] = 0.
        omega_dot = np.zeros(3)
        quat_dot = np.zeros(4)

    dxdt = np.zeros(14)
    dxdt[0:3]     = vel_NED
    dxdt[3:6]     = acc_body
    dxdt[6:10]    = quat_dot
    dxdt[10:13]   = omega_dot
    dxdt[13]      = mdot

    return dxdt

#####################################################################
# Subroutine 
#####################################################################

def calc_air_speed(vel: np.ndarray, wind: np.ndarray):
    '''対気速度計算'''

    return vel - wind

def calc_angle_of_attack(vel_air:np.ndarray):
    '''迎角・横滑り角計算'''

    norm = np.linalg.norm(vel_air)


    alpha = 0.
    beta  = 0.

    if norm > 0.:
        alpha = np.arctan2(vel_air[2], vel_air[0])
        beta  = np.arcsin(vel_air[1] / norm)

    return alpha, beta

def calc_mach_number(vel_air_abs, sound_speed):
    '''マッハ数計算'''

    return vel_air_abs / sound_speed

def calc_dynamic_pressure(vel_air_abs, air_density):
    '''動圧計算'''

    return 0.5 * air_density * vel_air_abs**2

def calc_aero_force(dynamic_pressure, alpha, beta, coeff_axial, coeff_normal, area_ref):
    '''空気力計算'''
    
    axial  = dynamic_pressure * coeff_axial  * area_ref
    normal = dynamic_pressure * coeff_normal * area_ref * alpha
    side   = dynamic_pressure * coeff_normal * area_ref * beta
    
    return np.array([- axial, - side, - normal])

def calc_acceleration(force_aero, force_thrust, force_gravity, mass, vel, omega):

    acc = force_aero + force_thrust + force_gravity
    acc /= mass
    acc += - np.cross(omega, vel)

    return acc

def calc_aero_moment(lcg, lcp, force_aero):
    '''空力モーメント（復元モーメント）'''

    arm = np.array([lcp - lcg, 0.0, 0.0])
    
    return np.cross(arm, force_aero)

def calc_aero_damping_moment(dynamic_pressure, vel_air_abs, omega, coeff_lp, coeff_mq, coeff_nr, area, length, diameter):
    '''空力的な減衰モーメント'''

    moment  = dynamic_pressure \
            * np.array([coeff_lp,
                        coeff_mq,
                        coeff_nr]) \
            * area \
            * np.array([.5 * diameter**2,
                        .5 * length**2,
                        .5 * length**2]) \
            / vel_air_abs \
            * omega

    return moment

def calc_gyro_moment(omega, Ij):

    tensor = np.array([
        [Ij[0]  , 0.    , 0.],
        [0.     , Ij[1] , 0.],
        [0.     , 0.    , Ij[2]]
    ])

    return - np.cross(omega, tensor @ omega)

def calc_omega_dot(moment_aero, moment_aero_damp, moment_gyro, Ij):

    return (moment_aero + moment_aero_damp + moment_gyro) / Ij

def calc_quat_dot(omega, quat):

    p = omega[0]
    q = omega[1]
    r = omega[2]

    tensor = np.array([
        [.0, -p, -q, -r],
        [ p, .0,  r, -q],
        [ q, -r, .0,  p],
        [ r,  q, -p, .0]
    ])

    return 0.5 * tensor @ quaternion.as_float_array(quat)

def is_launch_clear(pos, dcm, l_launcher):

    distance = (dcm.T @ pos)[0]

    if distance > l_launcher:
        return True
    else:
        return False

def event_land(time, x, param):

    if x[2] < 0.:
        return 1
    else:
        return 0

#### for Debug Function ###############################################

def __check_result(time, pos, vel, quat, omega, mass, param):

    import matplotlib.pyplot as plt

    plt.figure('Position')
    plt.plot(time, pos[0])
    plt.plot(time, pos[1])
    plt.plot(time, pos[2])
    plt.xlim(left=0., right=time[-1])
    plt.grid()
    plt.savefig('test/' + 'Position' + '.png')

    plt.figure('Velocity')
    plt.plot(time, vel[0])
    plt.plot(time, vel[1])
    plt.plot(time, vel[2])
    plt.xlim(left=0., right=time[-1])
    plt.grid()
    plt.savefig('test/' + 'Velocity' + '.png')
    
    plt.figure('Quaternion')
    plt.plot(time, quat[0])
    plt.plot(time, quat[1])
    plt.plot(time, quat[2])
    plt.plot(time, quat[3])
    plt.xlim(left=0., right=time[-1])
    plt.grid()
    plt.savefig('test/' + 'Quaternion' + '.png')

    quat_new = quaternion.from_float_array(quat.T)
    euler = np.rad2deg(quaternion.as_euler_angles(quat_new))

    plt.figure('Euler')
    plt.plot(time, euler[:, 0], label='Azimuth')
    plt.plot(time, euler[:, 1], label='Elevation')
    plt.plot(time, euler[:, 2], label='Roll')
    plt.xlim(left=0., right=time[-1])
    plt.legend()
    plt.grid()
    plt.savefig('test/' + 'Euler' + '.png')

    plt.figure('Omega')
    plt.plot(time, omega[0])
    plt.plot(time, omega[1])
    plt.plot(time, omega[2])
    plt.xlim(left=0., right=time[-1])
    plt.grid()
    plt.savefig('test/' + 'Omega' + '.png')
    
    plt.figure('Mass')
    plt.plot(time, mass)
    plt.xlim(left=0., right=time[-1])
    plt.grid()
    plt.savefig('test/' + 'Mass' + '.png')

    alt_log = np.abs(pos[2])
    g, rho, cs  = param.atmos.get_atmosphere(alt_log)
    plt.figure('Air Density')
    plt.plot(time, rho)
    plt.xlim(left=0., right=time[-1])
    plt.grid()
    plt.savefig('test/' + 'Air_density' + '.png')

    dcm = quaternion.as_rotation_matrix(quat_new)
    vel_NED = np.array([d @ v for d, v in zip(dcm, vel.T)])
    wind_NED = np.array([param.wind.get_wind_NED(alt) for alt in alt_log])
    vel_air = np.array([d.T @ calc_air_speed(v, w) for d, v, w in zip(dcm, vel_NED, wind_NED)])
    aoa = np.array([np.rad2deg(calc_angle_of_attack(v)) for v in vel_air])
    
    plt.figure('Velocity_NED')
    plt.plot(time, vel_NED[:, 0])
    plt.plot(time, vel_NED[:, 1])
    plt.plot(time, vel_NED[:, 2])
    plt.xlim(left=0., right=time[-1])
    plt.grid()
    plt.savefig('test/' + 'Velocity_NED' + '.png')
    
    plt.figure('Air_speed')
    plt.plot(time, vel_air[:, 0])
    plt.plot(time, vel_air[:, 1])
    plt.plot(time, vel_air[:, 2])
    plt.xlim(left=0., right=time[-1])
    plt.grid()
    plt.savefig('test/' + 'Air_speed' + '.png')
    
    plt.figure('AoA_AoS')
    plt.plot(time, aoa[:, 0])
    plt.plot(time, aoa[:, 1])
    plt.xlim(left=0., right=time[-1])
    plt.ylim(bottom=-15., top=15.)
    plt.grid()
    plt.savefig('test/' + 'AoA_AoS' + '.png')



def __check_value(time, pos, vel, quat, omega, mass):

    index       = np.argmax( - pos[2])
    alt_apogee  = - pos[2, index]
    time_apogee = time[index]

    print('Apogee:',time_apogee, alt_apogee)
    print('Landing Time:', time[-1])
    print('Mass:', mass[0], mass[-1])

def __debug():

    from scipy.integrate import solve_ivp
    from Parameter.parameter import Parameter

    param = Parameter('example/rocket_config.csv')
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
    pos_log   = result.y[0:3]
    vel_log   = result.y[3:6]
    quat_log  = result.y[6:10]
    omega_log = result.y[10:13]
    mass_log  = result.y[13]
    __check_result(time_log, pos_log, vel_log, quat_log, omega_log, mass_log, param)
    __check_value(time_log, pos_log, vel_log, quat_log, omega_log, mass_log)

if __name__=='__main__':

    print('Hello World!')

    __debug()
