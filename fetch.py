import swift
import spatialgeometry as sg
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import qpsolvers as qp
import math

from environment import stuff


# Launch the simulator Swift
env = swift.Swift()
env.launch()

# Create a Fetch and Camera robot object
fetch = rtb.models.Fetch()
fetch_camera = rtb.models.FetchCamera()
print(fetch.grippers)

# Set joint angles to zero configuration
fetch.q = fetch.qz
print(fetch.n)
fetch_camera.q = fetch_camera.qz
print(fetch_camera.n)

# ADDING ENVIRONMENT STUFF
env_stuff = stuff()

# Add the Fetch and other shapes to the simulator
env.add(fetch)
env.add(fetch_camera)
env_stuff.add_to_env(env)


env.hold()