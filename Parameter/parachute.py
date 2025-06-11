class Parachute:

    def __init__(self, config):
        
        self.vel_para_1st   = config['vel_para_1st']
        self.exist_2nd_para = config['exist_2nd_para']
        
        if self.exist_2nd_para:
            self.mode_timer_2nd = config['2nd_para_timer']
            self.vel_para_2nd   = config['vel_para_2nd']
            
            if self.mode_timer_2nd:
                self.alt_para_2nd   = 0.
                self.time_2nd       = config['time_2nd']  
            
            else:
                self.alt_para_2nd   = config['alt_para_2nd']
                self.time_2nd       = 1.0E+10

        else:
            self.mode_timer_2nd = False
            self.vel_para_2nd   = 0.
            self.alt_para_2nd   = 0.
            self.time_2nd       = 0.

    def get_velocity(self, time, altitude):
        '''
        降下速度の取得
        '''

        if self.exist_2nd_para and \
            ( \
                (self.mode_timer_2nd and time >= self.time_2nd) \
            or \
                (not self.mode_timer_2nd and altitude <= self.alt_para_2nd) \
                        ):
            return self.vel_para_2nd
        
        else:
            return self.vel_para_1st