import numpy as np

class Launcher:

    def __init__(self, config):
        
        self.elevation  = config['elevation']
        self.azimuth    = config['azimuth']
        if self.azimuth > 90.:
            self.azimuth += 360
        elif self.azimuth < - 90.:
            self.azimuth -= 360
        self.length     = config['length']
        self.pos_LLH = np.zeros(3)