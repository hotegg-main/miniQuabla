import numpy as np
from scipy.interpolate import interp1d

class Aerodynamic:

    def __init__(self, config):
        
        self.lcp_const        = config['lcp']
        self.coeff_A    = config['CA']
        self.coeff_Na   = config['CNa']
        self.coeff_lp   = config['Clp']
        self.coeff_mq   = config['Cmq']
        if self.coeff_mq > 0:
            self.coeff_mq*= -1
        self.coeff_nr   = self.coeff_mq

        mach_array = np.array([0., 0.3, 20.])
        self.get_Lcp = interp1d(mach_array, self.lcp_const*np.array([1., 1., 1.]), kind='linear', bounds_error=False, fill_value=(self.lcp_const, self.lcp_const))

    # def get_Lcp(self, mach):
    #     '''圧力中心'''
        
    #     return self.lcp_const

    def get_coeffecient_force(self, mach):
        '''無次元化安定微係数（力）'''

        return self.coeff_A, self.coeff_Na