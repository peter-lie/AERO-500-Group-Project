# Caleb Nalley
# AERO 500 Agent Based Algorithm

import numpy as np
from vpython import *

# Parameters
N = 40
a = 1
gamma = 0.05
dx, dy = 0.05, 0.05
d_nom = np.sqrt(2*dx**2 + dx**1.1 + (np.pi/4)**2)

# Randomly generate 2D position
x_bar_k = np.zeros((2, N))
for i in range(N):
    x_k = np.random.uniform(-10, 10)
    y_k = np.random.uniform(-10, 10)
    x_bar_k[:, i] = [x_k, y_k]

# Sort by y (descending) and identify leader as highest y value
sort_indices = np.argsort(-x_bar_k[1, :])
x_bar_k = x_bar_k[:, sort_indices]
front_leader_idx = 0

# Rendering Setup
scene = canvas(title="Goose V-Formation", width=1000, height=600)
scene.range = 30
bodies = []
for i in range(N):
    color_i = color.yellow if i == front_leader_idx else color.white
    body = sphere(pos=vector(*x_bar_k[:, i], 0), radius=0.4, make_trail=False, color=color_i)
    bodies.append(body)


# Functions
def update_pos(i, x_bar, leader_pos, front_leader_pos):
    xi = x_bar[:, i]
    delta_r = leader_pos - xi
    dist = np.linalg.norm(delta_r)

    # far field
    if dist > d_nom:
        delta = gamma * delta_r  # delta r
        # Logic vector F^i
        F = np.zeros(4)
        side = np.sign(xi[0] - front_leader_pos[0])
        if side == -1:
            F[0] = 1  # move left
        elif side == 1:
            F[1] = 1  # move right
        else:
            F[np.random.choice([0, 1])] = 1

        F[2] = 1  # always move back
        if np.isclose(xi[0], leader_pos[0], atol=0.5) or np.isclose(xi[1], leader_pos[1], atol=0.5):
            F[3] = 1

        # B^i matrix (2x4)
        B = np.array([
            [-dx if F[0] else 0, dx if F[1] else 0, 0, 0],
            [0, 0, -dy if F[2] else 0, -dy if F[3] else 0]
        ])
        delta = delta + B @ F  # total movement

    # near field
    else:
        if xi[0] > leader_pos[0]:  # to the right of leader
            delta = gamma * (delta_r - np.array([-0.75 * a, 0.75 * a]))
        elif xi[0] == front_leader_pos[0]:  # possible for goose to be in line with front leader, so add this exception
            delta = gamma * (delta_r - np.array([1 * a, 1 * a]))
        else:  # to the front left
            delta = gamma * (delta_r - np.array([0.75 * a, 0.75 * a]))
    return delta


def get_relative_leader(i, x_bar):
    xi = x_bar[:, i]
    min_dist = np.inf
    rel_leader = None
    for j in range(N):
        if j == i:
            continue
        xj = x_bar[:, j]
        if xj[1] > xi[1]:  # must be in front in y
            dist = np.linalg.norm(xj - xi)
            if dist < min_dist:
                min_dist = dist
                rel_leader = j
    return rel_leader  # may return None if no leader


# Simulation Loop
while True:
    rate(30)
    front_leader_pos = x_bar_k[:, front_leader_idx]
    rel_leader_idx = get_relative_leader(i, x_bar_k)
    if rel_leader_idx is None:
        continue  # skip update if no leader ahead
    leader_pos = x_bar_k[:, rel_leader_idx].copy()

    for i in range(N):
        if i == front_leader_idx:
            continue

        rel_leader_idx = get_relative_leader(i, x_bar_k)
        if rel_leader_idx is None:
            continue
        leader_pos = x_bar_k[:, rel_leader_idx].copy()
        update = update_pos(i, x_bar_k, leader_pos, front_leader_pos)
        x_bar_k[:, i] += update

    # Update visualization
    for i in range(N):
        bodies[i].pos = vector(*x_bar_k[:, i], 0)
