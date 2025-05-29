class Aerodynamic:

    def __init__(self, config):
        
        self.lcp        = config['lcp']
        self.coeff_A    = config['CA']
        self.coeff_Na   = config['CNa']
        self.coeff_lp   = config['Clp']
        self.coeff_mq   = config['Cmq']
        if self.coeff_mq > 0:
            self.coeff_mq*= -1
        self.coeff_nr   = self.coeff_mq

    def get_Lcp(self, mach):
        '''圧力中心'''
        
        return self.lcp

    def get_coeffecient_force(self, mach):
        '''無次元化安定微係数（力）'''

        return self.coeff_A, self.coeff_Na