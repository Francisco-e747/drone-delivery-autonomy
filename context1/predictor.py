import numpy as np

class ConstantVelocityPredictor:
    def __init__(self, dt=0.5):
        self.dt = dt  # prediction horizon in seconds

    def predict(self, track_history):
        """
        track_history: list of (x, y, w, h) tuples, most recent last
        returns predicted (x, y, w, h) at t + dt
        """
        if len(track_history) < 2:
            return track_history[-1]

        prev = np.array(track_history[-2][:2])
        curr = np.array(track_history[-1][:2])
        velocity = curr - prev  # pixels per frame

        # assume ~20fps -> dt=0.5s = 10 frames ahead
        frames_ahead = self.dt * 20
        predicted_xy = curr + velocity * frames_ahead

        w, h = track_history[-1][2], track_history[-1][3]
        return (predicted_xy[0], predicted_xy[1], w, h)
