import numpy as np

class Launcher:

    def __init__(self, config):
        
        self.elevation  = config['elevation']
        self.azimuth    = config['azimuth']
        self.LLH        = np.array([config['Latitude'],
                                    config['Longtitude'],
                                    config['Height']])
        self.length     = config['length']
        self.pos_LLH = np.zeros(3)
        self.mag_dec    = - np.abs(config['mag_dec'])