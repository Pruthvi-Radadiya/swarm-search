import numpy as np


class IMU:
    def __init__(self, initial_pos, drift_std=0.002):
        self.believed_pos = np.array(initial_pos, dtype=np.float32)
        self.true_pos_prev = np.array(initial_pos, dtype=np.float32)
        self.drift_std = drift_std

    def update(self, true_pos):
        self.true_pos = np.array(true_pos, dtype=np.float32)
        delta = self.true_pos - self.true_pos_prev
        noise = np.random.normal(0, self.drift_std, size=3)

        self.believed_pos = self.believed_pos + noise + delta
        self.true_pos_prev = self.true_pos

        return self.believed_pos
