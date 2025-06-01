import numpy as np
from scipy.interpolate import interp1d

class Aerodynamic:

    def __init__(self, config):
        
        self.lcp_const      = config['lcp']
        self.coeff_A_const  = config['CA']
        self.coeff_Na_const = config['CNa']
        self.coeff_lp       = config['Clp']
        self.coeff_mq       = config['Cmq']
        if self.coeff_mq > 0:
            self.coeff_mq*= -1
        self.coeff_nr   = self.coeff_mq

        mach_array = np.array([0., 0.3, 20.])
        self.get_Lcp      = interp1d(mach_array, self.lcp_const     *np.array([1., 1., 1.]), kind='linear', bounds_error=False, fill_value=(self.lcp_const, self.lcp_const))
        self.get_coeff_A  = interp1d(mach_array, self.coeff_A_const *np.array([1., 1., 1.]), kind='linear', bounds_error=False, fill_value=(self.coeff_A_const, self.coeff_A_const))
        self.get_coeff_Na = interp1d(mach_array, self.coeff_Na_const*np.array([1., 1., 1.]), kind='linear', bounds_error=False, fill_value=(self.coeff_Na_const, self.coeff_Na_const))

    def get_coeffecient_force(self, mach):
        '''無次元化安定微係数（力）'''

        return self.get_coeff_A(mach), self.get_coeff_Na(mach)