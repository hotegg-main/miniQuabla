import pandas as pd
import numpy as np
import quaternion
from setuptools._distutils.util import strtobool

class Parameter:

    def __init__(self, path_config):

        self.config = self.__allocate_df(self.__read_config(path_config))

        self.aero    = Aerodynamic(self.config['Aerodynamics'])
        self.atmos   = Atmosphere()
        self.wind    = Wind(self.config['Wind'])
        self.engine  = Engine(self.config['Engine'])
        self.geomet  = Geometry(self.config['Geometry'], self.engine)
        self.launch  = Launcher(self.config['Launcher'])
        self.para    = Parachute(self.config['Parachute'])
        self.payload = Payload(self.config['Payload'])

        self.name   = self.config['Solver']['name']
        self.dt     = self.config['Solver']['dt']
        self.t_max  = self.config['Solver']['t_max']

    def __read_config(self, path):
        '''
        諸元ファイルの読み込み
        Args:
            path: 諸元csvのファイルパス
        Return:
            config: 辞書型
        '''

        df_src = pd.read_csv(path, comment='$', names=('name', 'value'))
        df_name = df_src['name'].str.strip()
        df_value = df_src['value'].str.strip()

        config = dict()
        for name, value in zip(df_name, df_value):
            config[name] = value

        return config

    def __allocate_df(self, config_src):
        '''csvから読み込んだ諸元を分類'''

        config_dst = dict()
        config_dst['Geometry'] = {}
        config_dst['Aerodynamics'] = {}
        config_dst['Engine'] = {}
        config_dst['Wind'] = {}
        config_dst['Launcher'] = {}
        config_dst['Parachute'] = {}
        config_dst['Payload'] = {}
        config_dst['Solver'] = {}

        config_dst['Geometry']['diameter']      = float(config_src['diameter'])
        config_dst['Geometry']['length']        = float(config_src['length'])
        config_dst['Geometry']['mass_dry']      = float(config_src['mass_dry'])
        config_dst['Geometry']['lcg_dry']       = float(config_src['lcg_dry'])
        config_dst['Geometry']['Ij_roll_dry']   = float(config_src['Ij_dry_roll'])
        config_dst['Geometry']['Ij_pitch_dry']  = float(config_src['Ij_dry_pitch'])
        
        config_dst['Engine']['mass_ox']         = float(config_src['mass_ox'])
        config_dst['Engine']['mass_fuel_bef']   = float(config_src['mass_fuel_bef'])
        config_dst['Engine']['mass_fuel_aft']   = float(config_src['mass_fuel_aft'])
        config_dst['Engine']['lcg_ox']          = float(config_src['lcg_ox'])
        config_dst['Engine']['lcg_fuel']        = float(config_src['lcg_fuel'])
        config_dst['Engine']['l_tank_cap']      = float(config_src['l_tank_cap'])
        config_dst['Engine']['thrust_file']     = config_src['thrust_file']
        # config_dst['Engine']['time_burn']       = float(config_src['time_burn'])
        
        config_dst['Aerodynamics']['lcp']               = float(config_src['lcp'])
        config_dst['Aerodynamics']['CA']                = float(config_src['CA'])
        config_dst['Aerodynamics']['CNa']               = float(config_src['CNa'])
        config_dst['Aerodynamics']['Clp']               = float(config_src['Clp'])
        config_dst['Aerodynamics']['Cmq']               = float(config_src['Cmq'])
        config_dst['Aerodynamics']['exsist_lcp_file']   = strtobool(config_src['exsist_lcp_file'])
        config_dst['Aerodynamics']['exsist_CA_file']    = strtobool(config_src['exsist_CA_file'])
        config_dst['Aerodynamics']['exsist_CNa_file']   = strtobool(config_src['exsist_CNa_file'])
        config_dst['Aerodynamics']['lcp_file']          = str(config_src['lcp_file'])
        config_dst['Aerodynamics']['CA_file']           = str(config_src['CA_file'])
        config_dst['Aerodynamics']['CNa_file']          = str(config_src['CNa_file'])
        
        # config_dst['Wind']['model']         = config_src['wind_model']
        config_dst['Wind']['speed']         = float(config_src['wind_speed'])
        config_dst['Wind']['azimuth']       = float(config_src['wind_azimuth'])
        config_dst['Wind']['power_coeff']   = float(config_src['wind_power_coeff'])
        config_dst['Wind']['altitude']      = float(config_src['wind_alt_ref'])
        config_dst['Wind']['exist_file']    = strtobool(config_src['exist_wind_file'])
        config_dst['Wind']['file']          = config_src['wind_file']
        
        config_dst['Launcher']['elevation'] = float(config_src['launch_elevation'])
        config_dst['Launcher']['azimuth']   = float(config_src['launch_azimuth'])
        config_dst['Launcher']['length']    = float(config_src['rail_length'])
        config_dst['Launcher']['Latitude']  = float(config_src['lat'])
        config_dst['Launcher']['Longtitude']= float(config_src['lon'])
        config_dst['Launcher']['Height']    = float(config_src['height'])
        config_dst['Launcher']['mag_dec']   = float(config_src['mag_dec'])
        
        config_dst['Parachute']['vel_para_1st']     = float(config_src['vel_para_1st'])
        config_dst['Parachute']['exist_2nd_para']   = strtobool(config_src['exist_2nd_para'])
        config_dst['Parachute']['2nd_para_timer']   = strtobool(config_src['2nd_para_timer'])
        config_dst['Parachute']['vel_para_2nd']     = float(config_src['vel_para_2nd'])
        config_dst['Parachute']['alt_para_2nd']     = float(config_src['alt_para_2nd'])
        config_dst['Parachute']['time_2nd']         = float(config_src['time_2nd'])

        config_dst['Payload']['exist_payload'] = strtobool(config_src['exist_payload'])
        config_dst['Payload']['mass_payload']  = float(config_src['mass_payload'])
        config_dst['Payload']['vel_payload']   = float(config_src['vel_payload'])
        
        config_dst['Solver']['name']    = config_src['name']
        config_dst['Solver']['dt']      = float(config_src['dt'])
        config_dst['Solver']['t_max']   = float(config_src['t_max'])

        return config_dst
    
    def get_initial_param(self):

        vel = np.zeros(3)
        omega = np.zeros(3)
        
        elev    = np.deg2rad(self.launch.elevation)
        azim    = np.deg2rad(self.launch.azimuth)
        roll    = 0.
        quat    = quaternion.from_euler_angles(np.array([azim, elev, roll])).normalized()
        dcm     = quaternion.as_rotation_matrix(quat)
        quat    = quaternion.from_rotation_matrix(dcm)
        mass    = self.geomet.mass_bef
        # pos     = dcm @ np.array([self.geomet.get_Lcg(mass, self.engine.get_lcg_prop(0.)), 0., 0.])
        pos     = dcm @ np.array([self.geomet.get_Lcg(0.), 0., 0.])

        return pos, vel, quat, omega, mass
    
def __deb_plot(time, lcg, thrust):

    import matplotlib.pyplot as plt

    plt.figure('C.G.')
    plt.plot(time, lcg)
    plt.xlim(left=time[0], right=time[-1])
    plt.grid()

    plt.figure('Thrust')
    plt.plot(time, thrust)
    plt.xlim(left=time[0], right=time[-1])
    plt.grid()

    plt.show()

def __debug():

    import numpy as np

    my = Parameter('rocket_config.csv')

    time_list = np.arange(0., 30.01, 0.01)
    lcg_list = my.geomet.get_Lcg(time_list)
    thrust_list = my.engine.get_thrust(time_list)
    __deb_plot(time_list, lcg_list, thrust_list)

    print('End Debug')
    # config_src = my.read_config('rocket_config.csv')
    # config = my.allocate_df(config_src)


if __name__=='__main__':

    from aerodynamic import Aerodynamic
    from atmosphere import Atmosphere
    from wind import Wind
    from engine import Engine
    from geometry import Geometry
    from launcher import Launcher
    from payload import Payload
    
    __debug()
    print('Have a Good Day!')

else:
    from .aerodynamic import Aerodynamic
    from .atmosphere import Atmosphere
    from .wind import Wind
    from .engine import Engine
    from .geometry import Geometry
    from .launcher import Launcher
    from .parachute import Parachute
    from .payload import Payload