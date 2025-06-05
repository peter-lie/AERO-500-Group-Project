# AERO 500 Group Project

# Paige Jewell, Peter Lie, Caleb Nalley

from vpython import *
import numpy as np

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


# VPython orbital simulation: Earth in ECI + differential equation-based satellite motion
from vpython import *
import numpy as np

# Parameters (adjustable)
mu = 398600             # Earth's gravitational parameter (m^3/s^2)
a = 6878                # semi-major axis (m)
e = 0                   # eccentricity
inc = 0                 # inclination in degrees
raan = 0                # right ascension of ascending node in degrees
argp = 0                # argument of periapsis in degrees

# Scene setup
scene = canvas(title="Satellite Orbit in ECI Frame (Differential Equation Method)",
               width=800, height=600, range=1.5 * a, autoscale=False)
scene.forward = vector(-1, -1, -1)  # Set to default 3D view for interaction

# Earth
earth_radius = 6378
earth = sphere(pos=vector(0,0,0), radius=earth_radius, texture=textures.earth, make_trail=False)

# Satellite
sat = sphere(radius=earth_radius*0.05, color=color.red, make_trail=True, trail_type='curve', retain=200)

# Initial orbital conditions in perifocal frame
p = a * (1 - e**2)
r0_pf = vector(p / (1 + e * cos(0)), 0, 0)
v0_pf = vector(0, np.sqrt(mu / p), 0)

# Convert angles to radians
inc_rad = np.radians(inc)
raan_rad = np.radians(raan)
argp_rad = np.radians(argp)

# Rotation matrices (corrected order: Rz(RAAN) * Rx(inc) * Rz(argp))
def rotation_matrix(omega, inc, RAAN):
    R_RAAN = np.array([[np.cos(RAAN), -np.sin(RAAN), 0],
                       [np.sin(RAAN),  np.cos(RAAN), 0],
                       [0, 0, 1]])
    R_inc = np.array([[1, 0, 0],
                      [0, np.cos(inc), -np.sin(inc)],
                      [0, np.sin(inc),  np.cos(inc)]])
    R_argp = np.array([[np.cos(omega), -np.sin(omega), 0],
                       [np.sin(omega),  np.cos(omega), 0],
                       [0, 0, 1]])
    return R_RAAN @ R_inc @ R_argp

# Initial conditions in ECI frame
R = rotation_matrix(argp_rad, inc_rad, raan_rad)
r0 = vector(*((R @ np.array([r0_pf.x, r0_pf.y, r0_pf.z])).tolist()))
v0 = vector(*((R @ np.array([v0_pf.x, v0_pf.y, v0_pf.z])).tolist()))

sat.pos = r0
sat.velocity = v0

# Time integration loop
dt = 10
while True:
    rate(100)
    r_vec = sat.pos
    r_mag = mag(r_vec)
    a_vec = -mu * r_vec / r_mag**3
    sat.velocity += a_vec * dt
    sat.pos += sat.velocity * dt

