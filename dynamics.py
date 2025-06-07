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
    
    # DCM_BODY->NED
    dcm = quaternion.as_rotation_matrix(quat)
    altitude = np.max([1.e-03, - pos[2]])
    
    area_ref        = param.geomet.area
    length_ref      = param.geomet.length
    diameter_ref    = param.geomet.diameter
    mdot            = param.engine.get_mass_flow_rate(time)
    lcg             = param.geomet.get_Lcg(time)
    Ij              = param.geomet.get_Ij(time)
    
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

    if not is_launch_clear(time, pos, dcm, param):

        if acc_body[0] < 0. and vel[0] < 0.:
            acc_body[0:3] = 0.

        acc_body[1:3] = 0.
        vel[1:3] = 0.
        omega_dot = np.zeros(3)
        quat_dot = np.zeros(4)

    dxdt        = np.zeros(14)
    dxdt[0:3]   = vel_NED
    dxdt[3:6]   = acc_body
    dxdt[6:10]  = quat_dot
    dxdt[10:13] = omega_dot
    dxdt[13]    = mdot

    return dxdt

def dynamics_parachute(time, x, param:Parameter):
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
    altitude = np.abs(pos[2])
    wind_NED = param.wind.get_wind_NED(altitude)
    vel_NED = np.zeros(3)
    vel_NED[0:2] = wind_NED[0:2]
    vel_NED[2] = param.para.vel_para_1st

    dxdt = np.zeros(3)
    dxdt[0:3] = vel_NED

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
    return - dynamic_pressure * \
            np.array([
                coeff_axial,
                coeff_normal,
                coeff_normal
            ]) \
            * area_ref \
            * np.array([
                1.,
                beta,
                alpha
            ])

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

def is_launch_clear(time, pos, dcm, param: Parameter):

    l_launcher = param.launch.length
    time_act   = param.engine.time_act
    distance = (dcm.T @ pos)[0] - param.geomet.get_Lcg(time)

    if distance <  l_launcher and time < time_act:
        return False
    else:
        return True

def event_land(time, x, param):

    if x[2] < 0.:
        return 1
    else:
        return 0

#### for Debug Function ###############################################


if __name__=='__main__':

    print('Hello World!')
