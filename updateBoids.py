## ICA 9 Test File

# Paige Jewell, Peter Lie, Caleb Nalley

# Boiding and avoidance

# Collision avoidance
    # Follow boids algorithms
    # Sense other boids in close proximity
    # Can define a view angle in here if we want - but unlimited orientation in space
    # Steer away from other boids in close proximity

import numpy as np
from genSatellitePos import genSatellitePos

# Boid parameters
SENSE_RADIUS = 15  # km, distance to consider another satellite as a neighbor
AVOID_GAIN = 0.2   # steering strength to avoid nearby satellites

# Simple boid update to steer away from nearby satellites
def update_boids(pos, vel, dt=1.0):
    numSats = pos.shape[1]
    new_vel = vel.copy()

    for i in range(numSats):
        steer = np.zeros(3)
        count = 0

        for j in range(numSats):
            if i == j:
                continue
            offset = pos[:, i] - pos[:, j]
            dist = np.linalg.norm(offset)
            if dist < SENSE_RADIUS and dist > 0:
                steer += offset / dist  # normalized avoidance vector
                count += 1

        if count > 0:
            steer = steer / count
            new_vel[:, i] += AVOID_GAIN * steer

    pos += new_vel * dt
    return pos, new_vel

# Example usage
satsPos = genSatellitePos(5)
satsVel = [0, 0, 0]

for step in range(10):
    satsPos, satsVel = update_boids(satsPos, satsVel)
    print(f"Step {step}:\n{satsPos}\n")

