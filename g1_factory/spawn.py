"""Seeded spawn perturbation shared by the launch file and the experiment runner (stdlib only)."""
from __future__ import annotations

import math
import random


def perturbed_spawn(spawn, seed: int, noise_m: float = 0.3, noise_deg: float = 20.0):
    """seed 0 -> the task's exact spawn; other seeds move it <= noise_m and turn it <= noise_deg."""
    x, y, yaw = (float(v) for v in spawn)
    if seed:
        rng = random.Random(seed)
        radius = noise_m*math.sqrt(rng.random())
        angle = rng.uniform(-math.pi, math.pi)
        x, y = x+radius*math.cos(angle), y+radius*math.sin(angle)
        yaw += math.radians(rng.uniform(-1, 1)*noise_deg)
    return [x, y, yaw]
