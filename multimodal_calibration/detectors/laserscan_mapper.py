import numpy as np

class LaserscanMapper:

    def __init__(self, ball_diameter):
        self.ball_diameter = ball_diameter

    def transform(self, detection):
        center, radius = detection

        x, y = center
        z = np.sqrt(self.ball_diameter ** 2 / 4 - radius ** 2)

        return np.array([x, y, z])