"""Script demonstrating the joint use of simulation and control.

The simulation is run by a `CtrlAviary` or `VisionAviary` environment.
The control is given by the PID implementation in `DSLPIDControl`.

Example
-------
In a terminal, run as:

    $ python fly.py

Notes
-----
The drones move, at different altitudes, along cicular trajectories
in the X-Y plane, around point (0, -.3).

"""
import os
import time
import swarm_control.Graph as g
from swarm_control import Utils
from swarm_control.Utils import gaussian_roll
import argparse
from datetime import datetime
import pdb
import math
import random
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt

from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.envs.VisionAviary import VisionAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.control.SimplePIDControl import SimplePIDControl
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool

if __name__ == "__main__":

    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(
        description='Helix flight script using CtrlAviary or VisionAviary and DSLPIDControl')
    parser.add_argument('--drone', default="cf2x", type=DroneModel, help='Drone model (default: CF2X)', metavar='',
                        choices=DroneModel)
    parser.add_argument('--num_drones', default=4, type=int, help='Number of drones (default: 3)', metavar='')
    parser.add_argument('--physics', default="pyb", type=Physics, help='Physics updates (default: PYB)', metavar='',
                        choices=Physics)
    parser.add_argument('--vision', default=False, type=str2bool, help='Whether to use VisionAviary (default: False)',
                        metavar='')
    parser.add_argument('--gui', default=True, type=str2bool, help='Whether to use PyBullet GUI (default: True)',
                        metavar='')
    parser.add_argument('--record_video', default=False, type=str2bool,
                        help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--plot', default=True, type=str2bool,
                        help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--user_debug_gui', default=False, type=str2bool,
                        help='Whether to add debug lines and parameters to the GUI (default: False)', metavar='')
    parser.add_argument('--aggregate', default=False, type=str2bool,
                        help='Whether to aggregate physics steps (default: False)', metavar='')
    parser.add_argument('--obstacles', default=False, type=str2bool,
                        help='Whether to add obstacles to the environment (default: True)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=240, type=int, help='Simulation frequency in Hz (default: 240)',
                        metavar='')
    parser.add_argument('--control_freq_hz', default=48, type=int, help='Control frequency in Hz (default: 48)',
                        metavar='')
    parser.add_argument('--duration_sec', default=5, type=int,
                        help='Duration of the simulation in seconds (default: 5)', metavar='')
    ARGS = parser.parse_args()

    #### Initialize the simulation #############################
    H = 0.1
    H_STEP = 0.02
    H_GOAL = 2
    R = 0.5
    # Initialised positions based on number of drones
    INIT_XYZS = np.array([[R * i, 0, H] for i in range(
        ARGS.num_drones)])  #np.array([[R*np.cos((i/6)*2*np.pi+np.pi/2), R*np.sin((i/6)*2*np.pi+np.pi/2), H] for i in range(ARGS.num_drones)])
    #Aggregation of physics steps (not used by default)
    AGGR_PHY_STEPS = int(ARGS.simulation_freq_hz / ARGS.control_freq_hz) if ARGS.aggregate else 1

    #### Create the environment with or without video capture ## Create sim window here
    if ARGS.vision:
        env = VisionAviary(drone_model=ARGS.drone,
                           num_drones=ARGS.num_drones,
                           initial_xyzs=INIT_XYZS,
                           physics=ARGS.physics,
                           neighbourhood_radius=10,
                           freq=ARGS.simulation_freq_hz,
                           aggregate_phy_steps=AGGR_PHY_STEPS,
                           gui=ARGS.gui,
                           record=ARGS.record_video,
                           obstacles=ARGS.obstacles
                           )
    else:
        env = CtrlAviary(drone_model=ARGS.drone,
                         num_drones=ARGS.num_drones,
                         initial_xyzs=INIT_XYZS,
                         physics=ARGS.physics,
                         neighbourhood_radius=10,
                         freq=ARGS.simulation_freq_hz,
                         aggregate_phy_steps=AGGR_PHY_STEPS,
                         gui=ARGS.gui,
                         record=ARGS.record_video,
                         obstacles=ARGS.obstacles,
                         user_debug_gui=ARGS.user_debug_gui
                         )

    #### Obtain the PyBullet Client ID from the environment ####
    PYB_CLIENT = env.getPyBulletClient()

    #### Initialize a takeoff path ######################
    PERIOD = 10
    NUM_WP = ARGS.control_freq_hz * PERIOD  # Number of waypoints in simulation
    TARGET_POS = np.zeros((NUM_WP, 3))  # TARGET_POS maps the XYZ positions to be followed broadly by the swarm
    for i in range(NUM_WP):
        height = i * H_STEP
        if height > H_GOAL:
            height = H_GOAL
        TARGET_POS[i, :] = (INIT_XYZS[0, 0],
                            INIT_XYZS[0, 1],
                            height)
    wp_counters = np.array([0 for i in
                            range(ARGS.num_drones)])  # value for each drone indicating pos in circle
    print("WP Counters: ", wp_counters)
    print("Target POS: ", TARGET_POS.shape)

    #### Initialize the logger #################################
    logger = Logger(logging_freq_hz=int(ARGS.simulation_freq_hz / AGGR_PHY_STEPS),
                    num_drones=ARGS.num_drones
                    )

    #### Create graph ##########################
    edges = (0, 3), (0, 1), (2, 3)
    swarm = g.Graph(edges, ARGS.num_drones, 0)
    delay_register = np.zeros(swarm.get_size(), float)

    #### Initialize the controllers ############################
    ctrl = [DSLPIDControl(env) for i in range(ARGS.num_drones)]
    # ctrl = [SimplePIDControl(env) for i in range(ARGS.num_drones)]

    #### !! SIMULATION START !! ####
    #### Run the simulation ####################################
    CTRL_EVERY_N_STEPS = int(np.floor(env.SIM_FREQ / ARGS.control_freq_hz))  #with preset values, =5
    action = {str(i): np.array([0, 0, 0, 0]) for i in
              range(ARGS.num_drones)}  #each drone given a goal position array spot
    START = time.time()
    for i in range(0, int(ARGS.duration_sec * env.SIM_FREQ),
                   AGGR_PHY_STEPS):  #loops through for duration * frequency iterations

        #### Make it rain rubber ducks #############################
        # (part of original code, but contains useful instructions for events occuring at a given time)
        # if i/env.SIM_FREQ>5 and i%10==0 and i/env.SIM_FREQ<10: p.loadURDF("duck_vhacd.urdf", [0+random.gauss(0, 0.3),-0.5+random.gauss(0, 0.3),3], p.getQuaternionFromEuler([random.randint(0,360),random.randint(0,360),random.randint(0,360)]), physicsClientId=PYB_CLIENT)

        #### Step the simulation ###################################
        obs, reward, done, info = env.step(action)

        #### Compute control at the desired frequency ##############
        if i % CTRL_EVERY_N_STEPS == 0:  # ie every five steps. At default, this is every ~20ms
            print("--20ms--")
            # This part is likely to be the one to get modified for swarm control messaging. WP iterator increases here

            # Check connectivity status
            for j in range(ARGS.num_drones):  # for each drone
                # find its neighbours
                neighbours = swarm.get_neighbours(j)
                # find distance between them
                drone1 = env.pos[j]
                for k in neighbours:
                    if k > j:  # If we've already visited that drone, (j > k) we don't need to test connection twice
                        # find drone positions
                        drone2 = env.pos[k]
                        # pythag theorem to find straight line between them
                        side_x = drone1[0] - drone2[0]
                        side_y = drone1[1] - drone2[1]
                        side_z = drone1[2] - drone2[2]
                        distance = np.sqrt(side_x**2 + side_y**2 + side_z**2)
                        print("Distance between drone ", j, " and drone ", k, ": ", distance)
                        # check connectivity to each one
                        if not Utils.check_connected(distance, L_s=gaussian_roll(3.0, 1.0),
                                                     L_misc=gaussian_roll(1.0, 0.5)):
                            # Link broken, remove edge
                            print("Link broken, drone ", j, " and ", k, " disconnected")
                            swarm.delete_edge(j, k)
                        # calc delay to send along this line
                        Utils.find_delay(distance, processing=gaussian_roll(0.020, 0.005))
                        # Insert into table (numdrones x numdrones with link delays in each cell)
                        # Change find ultimate links so it makes a tuple of all nodes it visits in order



            # get the connectivity table indicating how far away everything is from each other
            connections = swarm.find_ultimate_links(swarm.get_head(), np.zeros(swarm.SIZE,int))
                # Ping every drone, print how long it took for last drone to receive
            # print(connections)
            # Change so that the CH receives the first command, then in order across the swarm the rest
            # Head no delay
            # for j in connections: # each element in check_array
            #     drone_delay = connections[j]  # multiplied by


            # next one needs delay calcd
            # receive and follow. Need a bit to dynamically find delay for each drone as well.
            
            #### Compute control for the current way point #############
            for j in range(ARGS.num_drones):  # for each drone, give them a command to follow
                action[str(j)], _, _ = ctrl[j].computeControlFromState(
                    control_timestep=CTRL_EVERY_N_STEPS * env.TIMESTEP,
                    state=obs[str(j)]["state"],
                    target_pos=np.hstack([INIT_XYZS[j, 0], INIT_XYZS[j, 1], TARGET_POS[wp_counters[j], 2]])  #H+j*H_STEP
                    )  # individually assigned movements to each drone
                # Should be noted that this section should not just be TARGET_POS, else drones will certainly collide.

            #### Go to the next way point and loop #####################
            for j in range(ARGS.num_drones):  #for each drone
                wp_counters[j] = wp_counters[j] + 1 if wp_counters[j] < (
                            NUM_WP - 1) else 0  #increment the counter tracker

        #### Log the simulation ####################################
        for j in range(ARGS.num_drones):  #for each drone
            logger.log(drone=j,
                       timestamp=i / env.SIM_FREQ,
                       state=obs[str(j)]["state"],
                       control=np.hstack([TARGET_POS[wp_counters[j], 0:2], H + j * H_STEP, np.zeros(9)])
                       )  #make a log entry with its state and goal positions

        #### Printout ##############################################
        if i % env.SIM_FREQ == 0:
            env.render()
            #### Print matrices with the images captured by each drone #
            if ARGS.vision:
                for j in range(ARGS.num_drones):
                    print(obs[str(j)]["rgb"].shape, np.average(obs[str(j)]["rgb"]),
                          obs[str(j)]["dep"].shape, np.average(obs[str(j)]["dep"]),
                          obs[str(j)]["seg"].shape, np.average(obs[str(j)]["seg"])
                          )

        #### Sync the simulation ###################################
        if ARGS.gui:
            sync(i, START, env.TIMESTEP)  # Syncs the stepped simulation with the wall clock

    #### Close the environment #################################
    env.close()

    #### Save the simulation results ###########################
    logger.save()

    #### Plot the simulation results ###########################
    if ARGS.plot:
        logger.plot()
