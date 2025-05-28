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
    quat    = np.quaternion(x[6], x[7], x[8], x[9])
    omega   = x[10:13]
    mass    = x[13]
    
    dcm = quaternion.as_rotation_matrix(quat)
    altitude = - pos[2]
    
    area_ref        = param.geomet.area
    length_ref      = param.geomet.length
    diameter_ref    = param.geomet.diameter
    mdot = param.engine.get_mass_flow_rate(time)
    lcg         = param.geomet.get_Lcg(time)
    Ij          = param.geomet.get_Ij(time)
    # Ij_roll, Ij_pitch, Ij_yaw = Ij[0], Ij[1], Ij[2]
    
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
    force_gravity   = dcm.T @ np.array([0., 0., g])

    # Moment
    moment_aero         = calc_aero_moment(lcg, lcp, force_aero)
    moment_aero_damp    = calc_aero_damping_moment(dynamic_pressure, vel_air_abs, omega, coeff_lp, coeff_mq, coeff_nr, area_ref, length_ref, diameter_ref)
    moment_gyro         = calc_gyro_moment(omega, Ij)

    acc_body    = calc_acceleration(force_aero, force_thrust, force_gravity, mass, vel, omega)
    omega_dot   = calc_omega_dot(moment_aero, moment_aero_damp, moment_gyro, Ij)
    quat_dot    = calc_quat_dot(omega, quat)

    if not is_launch_clear(pos, dcm, param.launch.length):

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

#### Subroutine ###############################################
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

    return mass

def calc_aero_moment(lcg, lcp, force_aero):
    '''空力モーメント（復元モーメント）'''

    arm = np.array([lcg - lcp, 0.0, 0.0])
    
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
        [.0,  r, -q,  p],
        [-r, .0,  p,  q],
        [ q, -p, .0,  r],
        [-p, -q, -r, .0]
    ])

    return 0.5 * tensor @ quaternion.as_float_array(quat)

def is_launch_clear(pos, dcm, l_launcher):

    distance = (dcm.T @ pos)[0]

    if distance > l_launcher:
        return True
    else:
        return False

#### for Debug Function ###############################################

def __unit_test():

    vel_Body = np.array(
        [[1., 2., 3.],
         [1., 2., 3.],
         [1., 2., 3.],
         [1., 2., 3.],
         [1., 2., 3.],
         ]
    )

    wind = np.array(
        [[5., 2., 3.],
         [5., 2., 3.],
         [5., 2., 3.],
         [5., 2., 3.],
         [5., 2., 3.],
         ]
    )

    vel_NED = [0.2305403462, -12.57858105, -31.56811072]
    wind_NED = [0.2305403462, -12.57858105, -31.56811072]

    # quat = quaternion.from_euler_angles(np.deg2rad([-88.95,	70,	-9.30E-15]))
    quat = quaternion.from_euler_angles(np.deg2rad([-88.95, 68.24064192, 2.57E-14]))
    # quat = quaternion.from_euler_angles(np.deg2rad([0, 68.24064192, -88.95]))
    dcm = quaternion.as_rotation_matrix(quat)
    vel_Body = dcm.T @ vel_NED
    wind = dcm.T @ wind_NED

    # vel_air = calc_air_speed(vel_Body, wind)
    vel_air = np.array([35.37841342,	-1.59E-15,	3.478619408])
    alpha, beta = calc_angle_of_attack(vel_air)
    vel_air_abs = np.linalg.norm(vel_air)
    mach = calc_mach_number(vel_air_abs, 350)
    dynamic_pressure = calc_dynamic_pressure(vel_air_abs, 1.23)
    force_aero = calc_aero_force(dynamic_pressure, alpha, beta, .6, 7.2, 0.011)

    quatdot = calc_quat_dot(np.zeros(3), quat)

    print(vel_air)

def __debug_dynamics():

    from scipy.integrate import solve_ivp

    time_sta = 0.
    time_max = 200.

    pos0    = np.array([0., 0., 0.])
    vel0    = np.array([0., 0., 0.])
    quat0   = quaternion.from_euler_angles(np.deg2rad([-88.95, 70., 0.]))
    omega0  = np.array([0., 0., 0.])
    mass0    = 8.118825

    x0 = np.zeros(14)
    x0[0:3]     = pos0
    x0[3:6]     = vel0
    x0[6:10]    = quat0
    x0[10:13]   = omega0
    x0[13]      = mass0


    result = solve_ivp(dynamics_trajectory, (time_sta, time_max), x0, args=(), )

def __debug():

    time0 = 0.
    pos0 = np.array([0., 0., 0.])
    vel0 = np.array([0., 0., 0.])
    omega0 = np.array([0., 0., 0.])


    quat0 = quaternion.from_euler_angles(np.deg2rad([0., -70., 270.]))
    euler0 = np.rad2deg(quaternion.as_euler_angles(quat0))

    dynamics_trajectory(time0, pos0, vel0, quat0, omega0)

if __name__=='__main__':

    print('Hello World!')

    # __debug()
    __unit_test()
