import numpy as np

MODEL_POWER     = 'powewr'
MODEL_ORIGINAL  = 'original'

class Wind:

    def __init__(self, config):

        self.model          = config['model']
        self.speed_ref      = config['speed']
        self.azimuth_ref    = np.deg2rad(config['azimuth'])
        self.exponent       = config['power_coeff']
        self.altitude_ref   = config['altitude']
        self.mag_dec        = 0.
        
        self.path_file = ''
        if self.model == MODEL_ORIGINAL:
            self.path_file = config['file']

    def get_wind_NED(self, altitude):
        
        speed = self.__power_law(altitude)
        azimuth = self.azimuth_ref 
        azimuth -= self.mag_dec

        return - speed * np.array([np.cos(azimuth), np.sin(azimuth), 0.])

    def __power_law(self, altitude):

        return self.speed_ref * (altitude / self.altitude_ref)** (1. / self.exponent)
    
if __name__=='__main__':

    import matplotlib.pyplot as plt

    config = dict()
    config['model'] = 'power'
    config['speed'] = 3.
    config['azimuth']   = 0.
    config['power_coeff']   = 4.5
    config['altitude']  = 2.
    my = Wind(config)

    alt_src = np.arange(0., 200, 1.).tolist()
    speed = [my.get_wind_NED(alt)[0] for alt in alt_src]

    plt.figure()
    plt.plot(speed, alt_src)
    plt.show()
    
    print('End Debug')