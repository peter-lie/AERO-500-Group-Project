# AERO 500 Group Project

# Paige Jewell, Peter Lie, Caleb Nalley

# Clear terminal
import os
clear = lambda: os.system('clear')
clear()

from vpython import *
import numpy as np
# import COE2RV

# 3D Boids algorithm - be super cool if applicable on satellites
# Start with orbital motion
# Implement collision avoidance - like geese agents

# Orbital motion goals:
    # Start scenario similar to rocket release
    # Achieve even spacing - phasing
    # Correct orbit
    # Collision avoidance
        # Follow boids algorithms
        # Sense other boids in close proximity
        # Can define a view angle in here if we want - but unlimited orientation in space
        # Steer away from other boids in close proximity

    # Avoid a rocket
    # Track total delta-v if possible
    # Plot in vpython

# Parameters (adjustable)
mu = 398600             # Earth's gravitational parameter (m^3/s^2)
a = 6978                # semi-major axis (m)
e = 0                   # eccentricity
inc = np.pi/8                 # inclination in rads
raan = -np.pi/2                # right ascension of ascending node in rads
argp = 0                # argument of periapsis in rads

from COE2RV import coe_to_rv

r0, v0 = coe_to_rv(a, e, inc, raan, argp, 0)

# Scene setup
scene = canvas(title="Satellite Orbit in ECI Frame",
               width=800, height=600, range=2 * a, autoscale=False)
scene.forward = vector(0, 2, -1)  # view from above
scene.up = vector(0, 1, 0)

# Earth
earth_radius = 6378
earth = sphere(pos=vector(0,0,0), radius=earth_radius, texture=textures.earth, make_trail=False)
earth.rotate(angle=np.pi/2, axis=vector(1,0,0))

# Satellite
sat = sphere(radius=earth_radius*0.03, color=color.red, make_trail=True, trail_type='curve', retain=200)

print("r0: ", r0)

sat.pos = vector(*r0)
sat.velocity = vector(*v0)

# Time integration loop
dt = 10
while True:
    rate(100)
    r_vec = sat.pos
    r_mag = mag(r_vec)
    a_vec = -mu * r_vec / r_mag**3
    sat.velocity += a_vec * dt
    sat.pos += sat.velocity * dt

