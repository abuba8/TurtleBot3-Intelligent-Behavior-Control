import numpy as np

class InputSpace:
    def __init__(self, name):
        self.name = name
        self.c_range = np.arange(0.0, 0.41, 0.1)
        self.close_centroid = np.round(np.mean(self.c_range), decimals=2)
        self.m_range = np.arange(self.close_centroid, 0.651, 0.1)
        self.medium_centroid = np.round(np.mean(self.m_range), decimals=2)
        self.f_range = np.arange(self.medium_centroid, 0.81, 0.1)
        self.far_centroid = np.round(np.mean(self.f_range), decimals=2)
