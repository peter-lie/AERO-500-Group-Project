## ICA 9 Test File

# Paige Jewell, Peter Lie, Caleb Nalley

# Boiding and avoidance

# Collision avoidance
    # Follow boids algorithms
    # Sense other boids in close proximity
    # Can define a view angle in here if we want - but unlimited orientation in space
    # Steer away from other boids in close proximity

from vpython import *
import numpy as np
from genSatellitePos import genSatellitePos

# Boid parameters
SENSE_RADIUS = 15  # km, distance to consider another satellite as a neighbor
AVOID_GAIN = 0.2   # steering strength to avoid nearby satellites


new_vel = np.zeros((3,5))
# Simple boid update to steer away from nearby satellites
def update_boids(pos, vel, dt=1.0):
    pos = np.array(pos)
    vel = np.array(vel)
    numSats = pos.shape[1]
    # new_vel

    for i in range(numSats):
        steer = np.zeros((3,))
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
            # print(new_vel)
            # new_vel[:, i] = vel[:, i] + AVOID_GAIN * steer
            new_vel[:, i] = AVOID_GAIN * steer

    pos += new_vel * dt
    return pos, new_vel


# Example usage
satsPos = genSatellitePos(5)
satsVel = [0, 0, 0]

for step in range(10):
    satsPos, satsVel = update_boids(satsPos, satsVel)
    print(f"Step {step}:{satsPos}\n")


# Example usage and VPython visualization
satsPos = genSatellitePos(5)
satsVel = [0, 0, 0]

scene = canvas(title="Boid Satellite Avoidance", width=800, height=600, range=100)
scene.background = color.black

satellite_spheres = []
for i in range(satsPos.shape[1]):
    sphere_i = sphere(pos=vector(*satsPos[:, i]), radius=2, color=color.red, make_trail=True)
    satellite_spheres.append(sphere_i)

while True:
    rate(20)
    satsPos, satsVel = update_boids(satsPos, satsVel)
    for i, s in enumerate(satellite_spheres):
        s.pos = vector(*satsPos[:, i])
