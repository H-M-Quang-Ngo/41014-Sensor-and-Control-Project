# #!/usr/bin/env python
# """
# @ base on Jesse Haviland
# """

import swift
import roboticstoolbox as rtb
import numpy as np
import time
from environment import stuff

# Launch the simulator Swift
env = swift.Swift()
env.launch()

# Make a fetch robot and add it to Swift
fetch = rtb.models.Fetch()
fetch.q = fetch.qr
env.add(fetch)

fetch.links[1].qlim = [-2*np.pi, 2*np.pi]
fetch.links[2].qlim = [-10, 10]

# Add some objects
env_stuff = stuff()
env_stuff.add_to_env(env)


for link in fetch.links:
    print(link.name)
# This is our callback funciton from the sliders in Swift which set
# the joint angles of our robot to the value of the sliders
def set_joint(j, value):
    if j == 0:
        # print("a", base)
        # baselink_first = fetch.fkine(fetch.q, end= 'base_link')
        fetch.q[j] = np.deg2rad(float(value))
        # baselink_second  = fetch.fkine(fetch.q, end= 'base_link')
        
        print(fetch.fkine(fetch.q, end= 'base_link'))
        # print("b",fetch.base)
    else:
        fetch.q[j] = np.deg2rad(float(value))
    

# Loop through each link in the fetch and if it is a variable joint,
# add a slider to Swift to control it
j = 0
for link in fetch.links:
    if link.isjoint:
        # We use a lambda as the callback function from Swift
        # j=j is used to set the value of j rather than the variable j
        # We use the HTML unicode format for the degree sign in the unit arg
        try:
            env.add(
                swift.Slider(
                    lambda x, j=j: set_joint(j, x),
                    min=np.round(np.rad2deg(link.qlim[0]), 2),
                    max=np.round(np.rad2deg(link.qlim[1]), 2),
                    step=1,
                    value=np.round(np.rad2deg(fetch.q[j]), 2),
                    desc="Fetch Joint " + str(j),
                    unit="&#176;",
                )
            )
        except:
            env.add(
                swift.Slider(
                    lambda x, j=j: set_joint(j, x),
                    min=-360,
                    max=360,
                    step=1,
                    value=np.round(np.rad2deg(fetch.q[j]), 2),
                    desc="Fetch Joint " + str(j),
                    unit="&#176;",
                )
            )

        j += 1

while True:
    # Process the event queue from Swift, this invokes the callback functions
    # from the sliders if the slider value was changed
    # env.process_events()

    # Update the environment with the new robot pose
    env.step(0)

    time.sleep(0.01)