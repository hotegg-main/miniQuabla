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
        
        exixst_lcp_file    = config['exsist_lcp_file']
        exixst_CA_file     = config['exsist_CA_file']
        exixst_CNa_file    = config['exsist_CNa_file']
        lcp_file    = config['lcp_file']
        CA_file     = config['CA_file']
        CNa_file    = config['CNa_file']

        mach_array_const = np.linspace(0., 20., 80)
        
        if exixst_lcp_file:            
            data_lcp    = np.loadtxt(lcp_file, delimiter=',', skiprows=1, usecols=(0, 1))
            mach_array  = data_lcp[:, 0]
            lcp_array   = data_lcp[:, 1] * 1.E-03

        else:
            mach_array      = mach_array_const
            lcp_array       = np.zeros(len(mach_array))
            lcp_array[:]    = self.lcp_const

        self.get_Lcp = interp1d(mach_array, lcp_array, kind='linear', bounds_error=False, fill_value=(lcp_array[0], lcp_array[-1]))
        
        if exixst_CA_file:            
            data_CA     = np.loadtxt(CA_file, delimiter=',', skiprows=1, usecols=(0, 1))
            mach_array  = data_CA[:, 0]
            CA_array    = data_CA[:, 1]

        else:
            mach_array  = mach_array_const
            CA_array    = np.zeros(len(mach_array))
            CA_array[:] = self.coeff_A_const

        self.get_coeff_A  = interp1d(mach_array, CA_array, kind='linear', bounds_error=False, fill_value=(CA_array[0], CA_array[-1]))
        
        if exixst_CNa_file:            
            data_CNa    = np.loadtxt(CNa_file, delimiter=',', skiprows=1, usecols=(0, 1))
            mach_array  = data_CNa[:, 0]
            CNa_array   = data_CNa[:, 1]

        else:
            mach_array      = mach_array_const
            CNa_array       = np.zeros(len(mach_array))
            CNa_array[:]    = self.coeff_Na_const

        self.get_coeff_Na = interp1d(mach_array, CNa_array, kind='linear', bounds_error=False, fill_value=(CNa_array[0], CNa_array[-1]))

    def get_coeffecient_force(self, mach):
        '''無次元化安定微係数（力）'''

        return self.get_coeff_A(mach), self.get_coeff_Na(mach)