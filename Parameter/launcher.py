import numpy as np

class Launcher:

    def __init__(self):
        
        self.length = 0.
        self.azimuth = 0.
        self.elevation = 0.
        self.pos_LLH = np.zeros(3)