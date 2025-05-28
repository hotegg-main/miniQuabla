import numpy as np

class Wind:

    def __init__(self):

        self.speed_ref      = 0.
        self.altitude_ref   = 0.
        self.azimuth_ref    = 0.
        self.exponent       = 6.
        self.mag_dec        = 0.

    def get_wind_NED(self, altitude):
        
        speed = self.__power_law(altitude)
        azimuth = self.azimuth_ref 
        azimuth -= self.mag_dec

        return - speed * np.array([np.cos(azimuth), np.sin(azimuth), 0.])

    def __power_law(self, altitude):

        return self.speed_ref * (altitude / self.altitude_ref)** (1. / self.exponent)
    
if __name__=='__main__':

    my = Wind()
    my.speed_ref      = 3.
    my.altitude_ref   = 2.
    my.azimuth_ref    = 0.
    my.exponent       = 4.5
    my.mag_dec        = 0.

    alt_src = np.arange(0., 200, 1.).tolist()
    speed = [my.get_wind_NED(alt)[0] for alt in alt_src]
    
    print('End Debug')