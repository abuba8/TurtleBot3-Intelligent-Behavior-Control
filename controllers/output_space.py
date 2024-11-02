class OutputSpace:
    def __init__(self, out):
        if out == 'x':
            self.slow = 0.06
            self.medium = 0.09
            self.fast = 0.12
        elif out == 'z':
            self.right = -0.5
            self.forward = 0.0
            self.left = 0.5
        else:
            raise ValueError('Pass the correct argument')
