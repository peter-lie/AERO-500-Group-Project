## Updating satellite boids to get to avoid and hold position

# Paige Jewell, Peter Lie, Caleb Nalley

# Boiding and avoidance
# Collision avoidance
    # Follow boids algorithms
    # Sense other boids in close proximity
    # Can define a view angle in here if we want - but unlimited orientation in space
    # Steer away from other boids in close proximity

from vpython import *
import numpy as np
import random as rnd
from genSatellitePos import genSatellitePos
from debris import generate_objects
from debris import visualize_objects
import time

# Boid parameters
SENSE_RADIUS = 10       # km, distance to consider another satellite as a neighbor
AVOID_GAIN = 0.01       # steering strength to avoid nearby satellites
FORMATION_GAIN = 0.04   # strength of position-holding force
DAMPING = 0.15           # Tune between 0 and 1 (0 = no damping, 1 = heavy damping)
DODGE_RADIUS = 15.0     # [km] Distance to react to falling debris
DODGE_GAIN = 0.07       # Strength of dodge reaction
NUM_DEBRIS = 250        # Amount of debris

SAT_SPACING = 20        # km apart in x from the leader

# Bounding box dimensions
X_MAX, Y_MAX, Z_MAX = 50, 50, 50
CENTER = vector(0,0,0)


def get_nominal_positions_from_initial(satPos, spacing=SAT_SPACING):
    """
    Create nominal positions for formation keeping based on initial sorted order.
    Leader is the leftmost (largest x value after sorting).
    """
    numSats = satPos.shape[1]
    leader_x = satPos[0, 0]  # leftmost (after sorting in genSatellitePos)
    nominal = np.zeros_like(satPos)
    for i in range(numSats):
        nominal[0, i] = leader_x - i * spacing
        nominal[1, i] = satPos[1, 0]  # same y as leader
        nominal[2, i] = satPos[2, 0]  # same z as leader
    return nominal


# Simple boid update to steer away from nearby satellites
def update_boids(pos, vel, nominal, debris_spheres, dt=1.0):
    pos = np.array(pos)
    vel = np.array(vel)
    nominal = np.array(nominal)
    numSats = pos.shape[1]
    new_vel = vel.copy()

    for i in range(numSats):
        steer = np.zeros((3,))
        count = 0

        # --- Satellite avoidance ---
        for j in range(numSats):
            if i == j:
                continue
            offset = pos[:, i] - pos[:, j]
            dist = np.linalg.norm(offset)
            if 0 < dist < SENSE_RADIUS:
                steer += offset / dist
                count += 1

        if count > 0:
            steer /= count
            new_vel[:, i] += AVOID_GAIN * steer

        # --- Formation holding ---
        formation_error = nominal[:, i] - pos[:, i]
        new_vel[:, i] += FORMATION_GAIN * formation_error

        # --- Debris avoidance ---
        for d in debris_spheres:
            debris_pos = np.array([d.pos.x, d.pos.y, d.pos.z])
            rel_pos = pos[:, i] - debris_pos
            dist = np.linalg.norm(rel_pos)
            # d.size = debris_objects[d]["size"]
            # print(d.size)
            if dist < DODGE_RADIUS + d.size.x:
                dodge_dir = np.cross(np.array([0, -1, 0]), rel_pos)  # lateral to falling direction
                if np.linalg.norm(dodge_dir) > 0:
                    dodge_dir = dodge_dir / np.linalg.norm(dodge_dir)
                    new_vel[:, i] += DODGE_GAIN * dodge_dir

        # --- Damping ---
        new_vel[:, i] -= DAMPING * vel[:, i]

    pos += new_vel * dt
    return pos, new_vel

# Example usage and VPython visualization
satsPos = genSatellitePos(5)
satsVel = np.zeros((3,5))
# print(satsPos)
# print(satsVel)
nominalPos = get_nominal_positions_from_initial(satsPos)


scene = canvas(title="Boid Satellite Avoidance", width=800, height=600, center=CENTER, background=color.black)
scene.range = max(X_MAX, Y_MAX, Z_MAX)

# Draw bounding box
# box_frame = box(pos=CENTER, size=vector(X_MAX, Y_MAX, Z_MAX), opacity=0.1, color=color.white)

satellite_spheres = []
for i in range(satsPos.shape[1]):
    sphere_i = sphere(pos=vector(*satsPos[:, i]), radius=1, color=color.red, make_trail=True, retain=300)
    satellite_spheres.append(sphere_i)

# Initialize debris
debris_objects = generate_objects(n=NUM_DEBRIS)
moving_spheres = visualize_objects(debris_objects)

# Main simulation loop
t_prev = time.perf_counter()
while True:
    rate(40)
    t_now = time.perf_counter()
    dt = t_now - t_prev
    t_prev = t_now

    # Update debris positions
    for s in moving_spheres:
        s.pos += s.velocity * dt

    # Update satellites with boids + debris avoidance
    satsPos, satsVel = update_boids(satsPos, satsVel, nominalPos, moving_spheres, dt)
    for i, s in enumerate(satellite_spheres):
        s.pos = vector(*satsPos[:, i])

