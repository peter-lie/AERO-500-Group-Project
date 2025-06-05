#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 27 21:21:42 2025

@author: paigejewell
"""

import numpy as np
from scipy.optimize import newton

def mean_to_true_anomaly(M_rad, e, tol=1e-8):
    """
    Solve Kepler's equation M = E - e*sin(E) for E (eccentric anomaly),
    then find true anomaly from E.
    """
    # Define the function for Newton's method
    def kepler_eq(E):
        return E - e * np.sin(E) - M_rad

    # Initial guess for E
    if e < 0.8:
        E0 = M_rad
    else:
        E0 = np.pi

    # Solve for eccentric anomaly E
    E = newton(kepler_eq, E0, tol=tol)

    # Now compute true anomaly
    true_anomaly = 2 * np.arctan2(np.sqrt(1+e) * np.sin(E/2),
                                  np.sqrt(1-e) * np.cos(E/2))
    
    # Make sure it's between 0 and 2*pi
    true_anomaly = true_anomaly % (2*np.pi)

    return true_anomaly

def coe_to_rv(a, e, i, RAAN, arg_periapsis, anomaly, anomaly_type='true', mu=398600.4418):
    """
    Converts COEs to r and v vectors.
    
    anomaly_type can be 'true' or 'mean'
    """

    if anomaly_type == 'true':
        true_anomaly = anomaly
    elif anomaly_type == 'mean':
        M = anomaly
        true_anomaly = mean_to_true_anomaly(M, e)
    else:
        raise ValueError("anomaly_type must be either 'true' or 'mean'")

    # 1. Compute the distance r
    p = a * (1 - e**2)
    r_mag = p / (1 + e * np.cos(true_anomaly))

    # 2. Perifocal frame (PQW)
    r_pqw = np.array([
        r_mag * np.cos(true_anomaly),
        r_mag * np.sin(true_anomaly),
        0
    ])

    v_pqw = np.array([
        -np.sqrt(mu/p) * np.sin(true_anomaly),
         np.sqrt(mu/p) * (e + np.cos(true_anomaly)),
         0
    ])

    # 3. Rotation matrix PQW → ECI
    cos_O = np.cos(RAAN)
    sin_O = np.sin(RAAN)
    cos_w = np.cos(arg_periapsis)
    sin_w = np.sin(arg_periapsis)
    cos_i = np.cos(i)
    sin_i = np.sin(i)

    R = np.array([
        [cos_O*cos_w - sin_O*sin_w*cos_i, -cos_O*sin_w - sin_O*cos_w*cos_i,  sin_O*sin_i],
        [sin_O*cos_w + cos_O*sin_w*cos_i, -sin_O*sin_w + cos_O*cos_w*cos_i, -cos_O*sin_i],
        [sin_w*sin_i,                      cos_w*sin_i,                     cos_i]
    ])

    # 4. Rotate to ECI
    r_eci = R @ r_pqw
    v_eci = R @ v_pqw

    return r_eci, v_eci

# Example usage
if __name__ == "__main__":
    # Example 1: true anomaly given
    a = 7000          # km
    e = 0.001
    i = 98.6          # degrees
    RAAN = 40.0       # degrees
    arg_periapsis = 30.0  # degrees
    anomaly = 45.0       # degrees (true anomaly)

    r_true, v_true = coe_to_rv(a, e, i, RAAN, arg_periapsis, anomaly, anomaly_type='true')

    print("With TRUE anomaly:")
    print("Position (km):", r_true)
    print("Velocity (km/s):", v_true)

    # Example 2: mean anomaly given
    anomaly_mean = 50.0  # degrees (mean anomaly)

    r_mean, v_mean = coe_to_rv(a, e, i, RAAN, arg_periapsis, anomaly_mean, anomaly_type='mean')

    print("\nWith MEAN anomaly:")
    print("Position (km):", r_mean)
    print("Velocity (km/s):", v_mean)
