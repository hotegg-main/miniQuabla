import pandas as pd


class Parameter:

    def __init__(self, path_config):

        self.config = self.__allocate_df(self.__read_config(path_config))

        self.aero   = Aerodynamic()
        self.atmos  = Atmosphere()
        self.wind   = Wind()
        self.engine = Engine(self.config['Engine'])
        self.geomet = Geometry(self.config['Geometry'], self.engine)
        self.launch = Launcher()

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
        config_dst['Engine']['thrust_file']     = config_src['thrust_file']
        
        config_dst['Aerodynamics']['lcp']   = float(config_src['lcp'])
        config_dst['Aerodynamics']['CA']    = float(config_src['CA'])
        config_dst['Aerodynamics']['CNa']   = float(config_src['CNa'])
        config_dst['Aerodynamics']['Clp']   = float(config_src['Clp'])
        config_dst['Aerodynamics']['Cmq']   = float(config_src['Cmq'])
        
        config_dst['Launcher']['elevation'] = float(config_src['launch_elevation'])
        config_dst['Launcher']['azimuth']   = float(config_src['launch_azimuth'])
        config_dst['Launcher']['length']    = float(config_src['rail_length'])
        
        config_dst['Solver']['dt']      = float(config_src['dt'])
        config_dst['Solver']['t_max']   = float(config_src['t_max'])

        return config_dst

def __debug():

    my = Parameter('rocket_config.csv')

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
    
    __debug()
    print('Have a Good Day!')

else:
    from .aerodynamic import Aerodynamic
    from .atmosphere import Atmosphere
    from .wind import Wind
    from .engine import Engine
    from .geometry import Geometry
    from .launcher import Launcher