import numpy as np

class CoordLayer:
    def __init__(self, start_threshold, end_threshold, centroid):
        self.end = end_threshold
        self.start = start_threshold
        self.m_range = np.arange(self.start, self.end, 0.1)
        self.member_centroid = centroid
        self.m_value = None

    def getMembershipLow(self, d_value):
        if d_value <= self.member_centroid:
            self.m_value = 1
            print('Left shoulder, value = 1')
        elif d_value > self.member_centroid and d_value < self.end:
            self.m_value = (self.end - d_value)/(self.end - self.member_centroid)
            print(f'{d_value}, falling edge for close, rising edge for medium')
        elif d_value > self.end:
            self.m_value = 0.0001
            print('Greater than, value = 0')  
        else:
            print('Value Exceeded')

    def getMembershipHigh(self, d_value):
        if d_value >= self.member_centroid:
            self.m_value = 1
            print('Right shoulder, value = 1')
        elif d_value < self.member_centroid and d_value > self.start:
            self.m_value = (self.end - d_value)/(self.end - self.member_centroid)
            print(f'{d_value}, falling edge for close, rising edge for medium')
        elif d_value < self.start:
            self.m_value = 0.0001
            print('Greater than, value = 0')  
        else:
            print('Value Exceeded')

