import numpy as np
from scipy.interpolate import interp1d
from scipy import integrate

class Engine:
    '''
    位置などは全て機体後端基準
    '''

    def __init__(self, config):

        self.mass_ox        = config['mass_ox']
        self.mass_fuel_bef  = config['mass_fuel_bef']
        self.mass_fuel_aft  = config['mass_fuel_aft']
        self.lcg_ox         = config['lcg_ox']
        self.lcg_fuel       = config['lcg_fuel']
        l_tank_cap          = config['l_tank_cap']
        self.l_fuel         = l_tank_cap
        # self.time_burn      = config['time_burn']
        self.delta_fuel = self.mass_fuel_bef - self.mass_fuel_aft
        
        self.l_tank = 2. * np.abs(l_tank_cap - self.lcg_ox)
        path = config['thrust_file']
        thrust_data = np.loadtxt(path, delimiter=',', skiprows=1, usecols=(0, 1))
        time_array = thrust_data[:, 0]
        thrust_array = thrust_data[:, 1]
        self.get_thrust = interp1d(time_array, thrust_array, kind='linear', bounds_error=False, fill_value=(0., 0.))
        #TODO: 推力最大値5%カット
        
        self.time_act = time_array[-1]

        total_impulse   = self.__get_total_impulse(time_array, thrust_array)
        thrust_ave      = total_impulse / self.time_act
        mdot_prop_ave   = - (self.delta_fuel + self.mass_ox) / self.time_act
        mdot_prop_log   = mdot_prop_ave * (thrust_array / thrust_ave)

        # 質量変化一定モデル
        # mdot_prop_const = - (self.delta_fuel + self.mass_ox) / self.time_act
        # mdot_prop_log = np.zeros(len(time_array))
        # mdot_prop_log[:] = mdot_prop_const
        
        self.get_mass_flow_rate = interp1d(time_array, mdot_prop_log, kind='linear', bounds_error=False, fill_value=(0., 0.))

        # lcg_prop_bef = self.mass_ox * self.lcg_ox + self.delta_fuel * self.lcg_fuel
        # lcg_prop_bef /= self.mass_ox + self.delta_fuel
        # # lcg_prop_aft = 0.
        # lcg_prop_log = np.array([lcg_prop_bef * (1. -  (time / self.time_act)) for time in time_array])
        # self.get_lcg_prop = interp1d(time_array, lcg_prop_log, kind='linear', bounds_error=False, fill_value=(lcg_prop_log[0], lcg_prop_log[-1]))

    def __get_total_impulse(self, time, thrust):

        return integrate.simpson(thrust, time)