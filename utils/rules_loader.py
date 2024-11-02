import numpy as np

def loadRules(path):
    return np.genfromtxt(path, delimiter=',', dtype=str)
