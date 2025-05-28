import numpy as np
from scipy.interpolate import interp1d

class Engine:
    '''
    位置などは全て機体後端基準
    '''

    def __init__(self, config):

        self.mass_ox         = config['mass_ox']
        self.mass_fuel_bef   = config['mass_fuel_bef']
        self.mass_fuel_aft   = config['mass_fuel_aft']
        self.lcg_ox          = config['lcg_ox']
        self.lcg_fuel        = config['lcg_fuel']
        self.delta_fuel = self.mass_fuel_bef - self.mass_fuel_aft
        
        path = config['thrust_file']
        thrust_data = np.loadtxt(path, delimiter=',', skiprows=1, usecols=(0, 1))
        time_array = thrust_data[:, 0]
        thrust_array = thrust_data[:, 1]
        self.get_thrust = interp1d(time_array, thrust_array, kind='linear', bounds_error=False, fill_value=(0., 0.))
        
        self.time_act = time_array[-1]

        mdot_prop_const = - (self.delta_fuel + self.mass_ox) / self.time_act
        

    def get_mass_flow_rate(self, time):
        ''''''

    # def get_thrust(self, time):
    #     '''推力'''