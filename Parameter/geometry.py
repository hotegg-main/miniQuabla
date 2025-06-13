from scipy.interpolate import interp1d
import numpy as np
# from .engine import Engine

class Geometry:

    def __init__(self, config, engine):
        
        self.diameter   = config['diameter'] #
        self.length     = config['length'] #
        self.area       = 0.25 * np.pi * self.diameter**2

        time_sta = 0.
        time_list = np.array([time_sta, engine.time_act])

        mass_dry = config['mass_dry']
        self.mass_bef = mass_dry + engine.mass_ox
        self.mass_inert = mass_dry - engine.delta_fuel
        
        lcg_dry = config['lcg_dry']                 # 乾燥重心
        lcg_bef = mass_dry * lcg_dry \
                + engine.mass_ox * engine.lcg_ox
        lcg_bef /= (mass_dry + engine.mass_ox)      # 燃焼前重心
        lcg_aft = mass_dry * lcg_dry - engine.delta_fuel * engine.lcg_fuel
        lcg_aft /= (mass_dry - engine.delta_fuel)   # 燃焼後重心
        self.lcg_inert = mass_dry * lcg_dry - engine.delta_fuel * engine.lcg_fuel
        self.lcg_inert /= mass_dry - engine.delta_fuel
        lcg_array = np.array([lcg_bef, lcg_aft]) 
        self.get_Lcg = interp1d(time_list, lcg_array, kind='linear', bounds_error=False, fill_value=(lcg_array[0], lcg_array[-1]))

        Ij_roll_dry  = config['Ij_roll_dry']
        Ij_pitch_dry = config['Ij_pitch_dry']
        Ij_pitch_ox = engine.mass_ox * ( self.diameter**2. / 16. + engine.l_tank**2. / 12. + (engine.lcg_ox - lcg_bef)**2 )
        Ij_pitch_fuel = engine.delta_fuel * ( self.diameter**2. / 16. + engine.l_fuel**2. / 12. + (engine.lcg_fuel - lcg_aft)**2 )
        Ij_pitch_dry_bef = Ij_pitch_dry + mass_dry * np.abs(lcg_bef - lcg_dry) ** 2
        Ij_pitch_dry_aft = Ij_pitch_dry
        Ij_pitch_bef = Ij_pitch_dry_bef + Ij_pitch_ox 
        Ij_pitch_aft = Ij_pitch_dry_aft - Ij_pitch_fuel
        Ij_pitch_array = np.array([Ij_pitch_bef, Ij_pitch_aft])
        Ij_roll_bef = Ij_roll_dry #
        Ij_roll_aft = Ij_roll_dry #
        Ij_roll_array = np.array([Ij_roll_bef, Ij_roll_aft])
        self.calc_Ij_pitch  = interp1d(time_list, Ij_pitch_array, kind='linear', bounds_error=False, fill_value=(Ij_pitch_array[0], Ij_pitch_array[-1]))
        self.calc_Ij_roll   = interp1d(time_list, Ij_roll_array, kind='linear', bounds_error=False, fill_value=(Ij_roll_array[0], Ij_roll_array[-1]))

    # def get_Lcg(self, mass, lcg_prop):
    #     '''重心'''

    #     mass_prop = mass - self.mass_inert

    #     return (self.mass_inert * self.lcg_inert + mass_prop * lcg_prop) / mass
    
    def get_Ij(self, time):
        '''慣性モーメント'''

        Ij_pitch = self.calc_Ij_pitch(time)
        Ij_roll = self.calc_Ij_roll(time)

        return np.array([Ij_roll, Ij_pitch, Ij_pitch])