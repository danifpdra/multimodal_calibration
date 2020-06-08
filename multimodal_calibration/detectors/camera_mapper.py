import numpy as np

class CameraMapper:

    def __init__(self, f, c, ball_diameter):
        fx, fy = f
        cx, cy = c
        
        self.fx, self.fy = f
        self.cx, self.cy = c
        self.favg = (f[0] + f[1]) / 2
        self.K = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]])
        
        self.ball_diameter = ball_diameter
    
    def transform(self, detection):
        (x, y), r = detection

        d = self.favg * self.ball_diameter / r / 2
        p = np.array([x, y, 1], dtype='f4')

        return d * np.linalg.inv(self.K).dot(p)
