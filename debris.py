import numpy as np
import time
from vpython import *

# ---------------------- configuration ----------------------
NUM_OBJECTS = 100          # number of objects
SIZE_MEAN = 1.0          # mean size of sphere (km)
SIZE_STD = 0.5          # size standard deviation
POS_MEAN = 0.0           # mean position coordinate
POS_STD = 25.0            # position standard deviation

# ---------------------- data generation --------------------
def generate_objects(n=NUM_OBJECTS):
    sizes = np.random.normal(loc=SIZE_MEAN, scale=SIZE_STD, size=n)
    # ensure positive sizes (take absolute value of any negatives)
    sizes = np.abs(sizes)
    positions = np.random.normal(loc=POS_MEAN, scale=POS_STD, size=(n, 3))

    objects = []
    for idx in range(n):
        objects.append(
            {
                "id": idx,
                "size": sizes[idx],               # scalar
                "pos": positions[idx].reshape(3)  # 1-D array [x, y, z]
            }
        )
    return objects

# ---------------------- visualisation ----------------------
def visualize_objects(objects):
    # scene = canvas(title="Randomly-sized Objects", width=800, height=600, background=color.black)

    # wire-frame cube (12 edges)
    half = 50
    w = 0.05
    corners = [vector(x, y, z)
               for x in (-half, half)
               for y in (-half, half)
               for z in (-half, half)]
    edges = [(0,1),(1,3),(3,2),(2,0),    # bottom
             (4,5),(5,7),(7,6),(6,4),    # top
             (0,4),(1,5),(2,6),(3,7)]    # pillars
    for i, j in edges:
        curve(pos=[corners[i], corners[j]], color=color.white, radius=w)

    # create spheres and store references
    spheres = []
    palette = [color.cyan, color.magenta, color.yellow,
               color.orange, color.white]
    for obj in objects:
        x, y, z = obj["pos"]
        y += 100               # start above the cube
        s = sphere(pos=vector(x, y, z),
                   radius=obj["size"] / 2.0,
                   color=palette[obj["id"] % len(palette)],
                   make_trail=False)
        s.velocity = vector(0, -10, 0)   # 5 units / s in −y
        spheres.append(s)
    return spheres

# ---------------------- main -------------------------------
if __name__ == "__main__":
    objs = generate_objects()
    moving_spheres = visualize_objects(objs)

    t_prev = time.perf_counter()
    while True:
        rate(60)                                # 60 fps limiter
        t_now = time.perf_counter()
        dt = t_now - t_prev
        t_prev = t_now

        for s in moving_spheres:                # update each sphere
            s.pos += s.velocity * dt