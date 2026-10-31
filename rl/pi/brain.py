"""The trained brain in plain numpy (made by export_brain.py). The Pi needs only numpy for it."""
import numpy as np


class Brain:
    def __init__(self, path):
        z = np.load(path)
        self.c = {k: z[k] for k in z.files}
        n = int(self.c["n_layers"])
        self.layers = [(self.c[f"w{i}"].astype(np.float64), self.c[f"b{i}"].astype(np.float64)) for i in range(n)]

    def act(self, obs):
        x = np.asarray(obs, dtype=np.float64)
        for i, (w, b) in enumerate(self.layers):
            x = w @ x + b
            if i < len(self.layers) - 1:
                x = np.tanh(x)
        return np.clip(x, -1.0, 1.0)      # the same clip as the trained brain's predict()

    def stand_hips(self, height):
        """Hip angles (f, r, f, r) for a leg height, from the simulator's five-bar table."""
        h, p = self.c["pose_h"], self.c["pose_hips"]
        return np.array([np.interp(height, h, p[:, i]) for i in range(4)])
