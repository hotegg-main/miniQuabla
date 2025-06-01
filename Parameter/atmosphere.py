import ussa1976
import numpy as np
from scipy.interpolate import interp1d

class Atmosphere:

    def __init__(self):


        alt_array = np.array([0.0, 11.000, 20.000, 32.000, 47.000, 51.000, 71.000, 84.852])
        alt_array *= 1e+03
        # Re = 6378.137e3 # Earth Radius [m]
        # alt_array = alt_array * Re / (Re + alt_array) # geometric altitude => geopotential height
        result = ussa1976.compute(alt_array)
        p_list      = result.p.values
        t_list      = result.t.values
        rho_list    = result.rho.values 
        cs_list     = result.cs.values

        # def calc_gravity(altitude):
        #     Re = 6378.137e3 # Earth Radius [m]
        #     g0 = 9.80665
        #     return g0 * (Re / (Re + altitude)) ** 2 # [m/s^2]
        # g_list      = calc_gravity(alt_array * 1.e+03)
        g_list      = np.array([9.80665]*8)

        self.get_pressure    = interp1d(alt_array, p_list, kind='linear', bounds_error=False, fill_value=(p_list[0], p_list[-1]))
        self.get_temperature = interp1d(alt_array, t_list, kind='linear', bounds_error=False, fill_value=(t_list[0], t_list[-1]))
        self.get_air_density = interp1d(alt_array, rho_list, kind='linear', bounds_error=False, fill_value=(rho_list[0], rho_list[-1]))
        self.get_sound_speed = interp1d(alt_array, cs_list, kind='linear', bounds_error=False, fill_value=(cs_list[0], cs_list[-1]))
        self.get_gravity     = interp1d(alt_array, g_list, kind='linear', bounds_error=False, fill_value=(g_list[0], g_list[-1]))

    def get_atmosphere(self, altitude):

        air_density = self.get_air_density(altitude)
        sound_speed = self.get_sound_speed(altitude)
        gravity     = self.get_gravity(altitude)

        return gravity, air_density, sound_speed

def __plot(alt, g, rho, cs):
    
    import matplotlib.pyplot as plt

    plt.figure('Gravity')
    plt.plot(g, alt)
    plt.ylim(bottom=alt[0], top=alt[-1])
    plt.xlabel('Gravity [m/s^2]')
    plt.ylabel('Altitude [m]')
    plt.minorticks_on()
    plt.grid(linestyle='--')
    
    plt.figure('Air Density')
    plt.plot(rho, alt)
    plt.ylim(bottom=alt[0], top=alt[-1])
    plt.xlabel('Air Density [kg/m^3]')
    plt.ylabel('Altitude [m]')
    plt.minorticks_on()
    plt.grid(linestyle='--')
    
    plt.figure('Sound Speed')
    plt.plot(cs, alt)
    plt.ylim(bottom=alt[0], top=alt[-1])
    plt.xlabel('Sund Speed [m/s]')
    plt.ylabel('Altitude [m]')
    plt.minorticks_on()
    plt.grid(linestyle='--')

    plt.show()

def __debug():

    alt_array = np.linspace(0.1, 20.e+03, 500)

    atmos = Atmosphere()
    g_log, rho_log, cs_log = atmos.get_atmosphere(alt_array)
    __plot(alt_array, g_log, rho_log, cs_log)

if __name__=='__main__':

    __debug()
    print('Have a Good Day!')