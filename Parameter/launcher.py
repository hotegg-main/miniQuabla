import numpy as np

class Launcher:

    def __init__(self, config):
        
        self.elevation  = config['elevation']
        self.azimuth    = config['azimuth']
        self.length     = config['length']
        self.pos_LLH = np.zeros(3)