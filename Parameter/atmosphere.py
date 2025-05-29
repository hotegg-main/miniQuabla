import ussa1976
import numpy as np
from scipy.interpolate import interp1d

class Atmosphere:

    def __init__(self):

        alt_array = np.array([0.0, 11.000, 20.000, 32.000, 47.000, 51.000, 71.000, 84.852])
        alt_array *= 1e+03
        result = ussa1976.compute(alt_array)
        p_list      = result.p.values
        t_list      = result.t.values
        rho_list    = result.rho.values 
        cs_list     = result.cs.values

        self.get_pressure    = interp1d(alt_array, p_list, kind='linear', bounds_error=False, fill_value=(p_list[0], p_list[-1]))
        self.get_temperature = interp1d(alt_array, t_list, kind='linear', bounds_error=False, fill_value=(t_list[0], t_list[-1]))
        self.get_air_density = interp1d(alt_array, rho_list, kind='linear', bounds_error=False, fill_value=(rho_list[0], rho_list[-1]))
        self.get_sound_speed = interp1d(alt_array, cs_list, kind='linear', bounds_error=False, fill_value=(cs_list[0], cs_list[-1]))

    def get_atmosphere(self, altitude):

        air_density = self.get_air_density(altitude)
        sound_speed = self.get_sound_speed(altitude)

        gravity = 9.08665

        return gravity, air_density, sound_speed

    
if __name__=='__main__':

    atmos = Atmosphere()
    atmos.get_atmosphere(np.array([200]))

    print('Have a Good Day!')