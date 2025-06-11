import numpy as np
from scipy.interpolate import interp1d

MODEL_POWER     = 'power'
MODEL_ORIGINAL  = 'original'

class Wind:

    def __init__(self, config):

        self.model          = config['model']
        self.speed_ref      = config['speed']
        self.azimuth_ref    = np.deg2rad(config['azimuth'])
        self.exponent       = config['power_coeff']
        self.altitude_ref   = config['altitude']

        if self.model == MODEL_POWER:
            alt_array       = np.linspace(0.0, 30.e+03, 3000)
            speed_array     = np.array([self.__power_law(alt) for alt in alt_array])
            azimuth_array   = np.array([self.azimuth_ref]*len(alt_array))
        
        elif self.model == MODEL_ORIGINAL:
            path = config['file']
            data_wind = np.loadtxt(path, delimiter=',', skiprows=1, usecols=(0, 1, 2))
            alt_array       = data_wind[:, 0]
            speed_array     = data_wind[:, 1]
            azimuth_array   = np.deg2rad(data_wind[:, 2])
            
        self.get_speed      = interp1d(alt_array, speed_array, kind='linear', bounds_error=False, fill_value=(speed_array[0], speed_array[-1]))
        self.get_azimuth    = interp1d(alt_array, azimuth_array, kind='linear', bounds_error=False, fill_value=(azimuth_array[0], azimuth_array[-1]))

    def set_power_model(self, speed_ref, azimuth_ref):

        self.model          = MODEL_POWER
        self.speed_ref      = speed_ref
        self.azimuth_ref    = np.deg2rad(azimuth_ref)

        alt_array       = np.linspace(0.0, 30.e+03, 3000)
        speed_array     = np.array([self.__power_law(alt) for alt in alt_array])
        azimuth_array   = np.array([self.azimuth_ref]*len(alt_array))
        
        self.get_speed      = interp1d(alt_array, speed_array, kind='linear', bounds_error=False, fill_value=(speed_array[0], speed_array[-1]))
        self.get_azimuth    = interp1d(alt_array, azimuth_array, kind='linear', bounds_error=False, fill_value=(azimuth_array[0], azimuth_array[-1]))


    def get_wind_NED(self, altitude):
        
        speed   = self.get_speed(altitude)
        azimuth = self.get_azimuth(altitude)

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