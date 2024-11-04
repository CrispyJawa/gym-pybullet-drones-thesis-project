"""Script for drones operating in a swarm together

The simulation is run by a `CtrlAviary` or `VisionAviary` environment.
The control is given by the PID implementation in `DSLPIDControl`.

Example
-------
In a terminal, run as:

    $ python thesis_demo.py

Notes
-----
The drones move in a user-specified direction,
with various parameter able to be adjusted

"""
import ast
import os
import time
import swarm_control.Graph as g
from swarm_control import Utils
from swarm_control.Utils import gaussian_roll
import argparse
from datetime import datetime
import pdb
import math
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


def list_of_floats(arg):
    return arg.split(',')


def parse_tuples(tuple_str):
    # Use literal_eval to safely parse the input string into actual tuple objects
    try:
        return ast.literal_eval(tuple_str)
    except ValueError:
        return tuple(Utils.optimal_topology_pattern(INIT_XYZS))


def make_folder(path, foldername):
    try:
        os.mkdir(path + foldername)
        print(f"Directory '{foldername}' created successfully.")
    except FileExistsError:
        print(f"Directory '{foldername}' already exists.")
    except PermissionError:
        print(f"Permission denied: Unable to create '{foldername}'.")
    except Exception as e:
        print(f"An error occurred: {e}")


if __name__ == "__main__":

    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(
        description='Helix flight script using CtrlAviary or VisionAviary and DSLPIDControl')
    parser.add_argument('--drone', default="cf2x", type=DroneModel, help='Drone model (default: CF2X)', metavar='',
                        choices=DroneModel)
    parser.add_argument('--num_drones', default=8, type=int, help='Number of drones (default: 3)', metavar='')
    parser.add_argument('--physics', default="pyb", type=Physics, help='Physics updates (default: PYB)',
                        metavar='', choices=Physics)
    parser.add_argument('--vision', default=False, type=str2bool, help='Whether to use VisionAviary (default: False)',
                        metavar='')
    parser.add_argument('--gui', default=True, type=str2bool, help='Whether to use PyBullet GUI (default: True)',
                        metavar='')
    parser.add_argument('--record_video', default=False, type=str2bool,
                        help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--plot', default=False, type=str2bool,
                        help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--user_debug_gui', default=False, type=str2bool,
                        help='Whether to add debug lines and parameters to the GUI (default: False)', metavar='')
    parser.add_argument('--aggregate', default=False, type=str2bool,
                        help='Whether to aggregate physics steps (default: False)', metavar='')
    parser.add_argument('--obstacles', default=False, type=str2bool,
                        help='Whether to add obstacles to the environment (default: True)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=240, type=int,
                        help='Simulation frequency in Hz (default: 240)', metavar='')
    # Control frequency: affects number of waypoints, frequency of motion updates
    parser.add_argument('--control_freq_hz', default=48, type=int,
                        help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec', default=10, type=int,
                        help='Duration of the simulation in seconds (default: 5)', metavar='')
    # Positioning parameters
    parser.add_argument('--swarm_direction', default=(0.0, 0.0, 1.0), type=list_of_floats,
                        help='Direction the drones will all try to fly in')
    parser.add_argument('--movement_rate', default=0.02, type=float,
                        help='Rate at which the drones move in the swarm_direction')
    parser.add_argument('--starting_height', default=0.3, type=float,
                        help='Initial height of all drones')
    parser.add_argument('--range_goal', default=50.0, type=float,
                        help='How far the drones will fly at the specified rate')
    parser.add_argument('--drone_spacing', default=5.0, type=float,
                        help='Distance between each drone in swarm setup')
    parser.add_argument('--swarm_topology', default="((0,1),(0,2),(0,3),(0,4),(0,5),(0,6),(0,7))",
                        help='A list of all connections between drones, in tuple form')
    # (0, 1), (1, 2), (0, 3), (0, 7), (1, 6), (2, 5), (3, 4), (4, 5)
    parser.add_argument('--swarm_head', default=0, type=int,
                        help='Which drone in the swarm shall receive communications and transmit to the rest')
    # Losses and delays
    parser.add_argument('--system_losses', default=(3.0, 0.7), type=float,
                        help='The mean and S.D. of system losses in communications as a tuple')
    parser.add_argument('--miscellaneous_losses', default=(1.0, 0.1), type=float,
                        help='The mean and S.D. of any other losses in communications as a tuple')
    parser.add_argument('--processing_delays', default=(0.025, 0.001), type=float,
                        help='The mean and S.D. of any processing delays between sending messages as a tuple')
    # Simulation control
    parser.add_argument('--simulation_iterations', default=1, type=int,
                        help='Number of simulation iterations')
    parser.add_argument('--simulation_title', default='default', type=str,
                        help='Title of csv files outputted at end')
    parser.add_argument('--output_data', default=True, type=str2bool,
                        help='Whether to save delay and position results (default: True)', metavar='')

    ARGS = parser.parse_args()
    filepath = "C:\\Users\\Signature Edition\\Documents\\PyGymDrones\\gym-pybullet-drones-0.5.2\\files\\outputs\\"
    #### Initialize the simulation #############################

    # Initialise physical positions and goals
    H = ARGS.starting_height
    R = ARGS.drone_spacing
    R_STEP = ARGS.movement_rate
    R_GOAL = ARGS.range_goal

    flight_vector = np.array(ARGS.swarm_direction, dtype=float)
    for i in range(len(flight_vector)):
        flight_vector[i] = flight_vector[i] * R_STEP

    # Initialised positions based on number of drones
    INIT_XYZS = np.array(
        [[R * (i % 4), R * (i // 4), H] for i in range(ARGS.num_drones)])  #sets drones in a rectangular pattern
    # np.array([[R*np.cos((i/6)*2*np.pi+np.pi/2), R*np.sin((i/6)*2*np.pi+np.pi/2), H] for i in range(ARGS.num_drones)]) #sets drones in a circle
    current_pos = INIT_XYZS

    # Aggregation of physics steps (not used by default)
    AGGR_PHY_STEPS = int(ARGS.simulation_freq_hz / ARGS.control_freq_hz) if ARGS.aggregate else 1  # default 1

    for sim_iter in range(ARGS.simulation_iterations):
        print("SIMULATION ITERATION: " + str(sim_iter))
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
        PERIOD = ARGS.duration_sec  # 10
        NUM_WP = ARGS.control_freq_hz * PERIOD  # Number of waypoints in simulation
        TARGET_POS = np.zeros((NUM_WP, 3))  # TARGET_POS maps the XYZ positions to be followed broadly by the swarm
        movement_step = np.zeros(3, dtype=float)
        for i in range(NUM_WP):
            for j in range(len(flight_vector)):
                movement_step[j] = i * flight_vector[j]  # Create a value for the motion of each drone from origin point
            distance = Utils.pythag_3D(movement_step[0], movement_step[1], movement_step[2])
            if distance < R_GOAL:
                TARGET_POS[i, :] = (movement_step[0],
                                    movement_step[1],
                                    movement_step[2])
            else:
                TARGET_POS[i, :] = TARGET_POS[i - 1, :]  # Fix to R_GOAL that has the drones stop here
        wp_counters = np.array([0 for i in range(ARGS.num_drones)])  # value for each drone indicating pos flight paths
        # print("WP Counters: ", wp_counters)
        # print("Target POS: ", TARGET_POS.shape)

        #### Initialize the logger #################################
        logger = Logger(logging_freq_hz=int(ARGS.simulation_freq_hz / AGGR_PHY_STEPS),
                        num_drones=ARGS.num_drones
                        )

        #### Create graph ##########################
        temp = ARGS.swarm_topology
        edges = parse_tuples(temp)
        head = ARGS.swarm_head
        swarm = g.Graph(edges, ARGS.num_drones, head)
        delay_register = np.zeros(swarm.get_size(), float)
        # swarm.print_graph()

        #### Initialize the controllers ############################
        ctrl = [DSLPIDControl(env) for i in range(ARGS.num_drones)]
        # ctrl = [SimplePIDControl(env) for i in range(ARGS.num_drones)]

        #### !! SIMULATION START !! ####
        #### Run the simulation ####################################
        position_results = np.zeros((swarm.get_size(), PERIOD, 3))  # swarm size x time x 3 array for tracking positions
        delay_results = np.zeros((swarm.get_size(), PERIOD))  #smaller array for tracking delays
        # link_quality_results =
        for i in range(ARGS.num_drones):
            position_results[i][0] = env.pos[i]  # Starting Position
            delay_results[i][0] = delay_register[i]
        CTRL_EVERY_N_STEPS = int(np.floor(env.SIM_FREQ / ARGS.control_freq_hz))  #with preset values, =5
        action = {str(i): np.array([0, 0, 0, 0]) for i in
                  range(ARGS.num_drones)}  #each drone given a goal position array spot
        START = time.time()
        for i in range(0, int(ARGS.duration_sec * env.SIM_FREQ),
                       AGGR_PHY_STEPS):  #loops through for duration * frequency iterations (1200 at default (240*5)) (step = 0.0041667 ms)

            #### Make it rain rubber ducks #############################
            # (part of original code, but contains useful instructions for events occuring at a given time)
            # if i/env.SIM_FREQ>5 and i%10==0 and i/env.SIM_FREQ<10: p.loadURDF("duck_vhacd.urdf", [0+random.gauss(0, 0.3),-0.5+random.gauss(0, 0.3),3], p.getQuaternionFromEuler([random.randint(0,360),random.randint(0,360),random.randint(0,360)]), physicsClientId=PYB_CLIENT)

            #### Step the simulation ###################################
            obs, reward, done, info = env.step(action)

            #### Compute control at the desired frequency ##############
            if i % CTRL_EVERY_N_STEPS == 0:  # At default, this is every ~20.83ms
                # print("--4ms--")

                #### Compute control for the current way point #############
                for j in range(ARGS.num_drones):  # for each drone, give them an action to follow
                    action[str(j)], _, _ = ctrl[j].computeControlFromState(
                        control_timestep=CTRL_EVERY_N_STEPS * env.TIMESTEP,
                        state=obs[str(j)]["state"],
                        target_pos=np.hstack(INIT_XYZS[j] + TARGET_POS[wp_counters[j]])
                        # target_pos=np.hstack([INIT_XYZS[j, 0], INIT_XYZS[j, 1], TARGET_POS[wp_counters[j], 2]])
                        # H+j*H_STEP
                    )  # individually assigned movements to each drone
                    # Should be noted that this section should not just be TARGET_POS, else drones will certainly collide.

                #### Go to the next way point and loop #####################
                for j in range(ARGS.num_drones):  # for each drone
                    if swarm.linked_to_head(j):
                        wp_counters[j] = wp_counters[j] + 1 if wp_counters[j] < (NUM_WP - 1) else 0
                        # increment the counter tracker, wrapping around if it's gone beyond

            #### Log the simulation ####################################
            for j in range(ARGS.num_drones):  #for each drone
                logger.log(drone=j,
                           timestamp=i / env.SIM_FREQ,
                           state=obs[str(j)]["state"],
                           control=np.hstack([TARGET_POS[wp_counters[j], 0:2], H + j * R_STEP, np.zeros(9)])
                           )  #make a log entry with its state and goal positions

            #### Printout ##############################################
            if i % env.SIM_FREQ == 0:
                env.render()
                print("SIMULATION ITERATOR: ",sim_iter)

                #CALCULATING SWARM DELAY
                # reset delay register
                delay_register.fill(0)

                # This part is likely to be the one to get modified for swarm control messaging. WP iterator increases here
                connections = swarm.find_ultimate_links(
                    swarm.get_head())  # find connections in dictionary starting from head
                # Check connectivity status
                for node in connections:  # find connection distance and delay for indexed node
                    # print(node)
                    last_item = swarm.get_head()
                    for path_item in connections[node]:
                        if path_item != swarm.get_head():
                            drone1 = last_item
                            drone2 = path_item
                            # print("    ", drone1, "->", drone2)
                            pos1 = env.pos[drone1]
                            pos2 = env.pos[drone2]
                            side_x = pos1[0] - pos2[0]
                            side_y = pos1[1] - pos2[1]
                            side_z = pos1[2] - pos2[2]
                            distance = Utils.pythag_3D(side_x, side_y, side_z)
                            # print("    Distance between drone ", drone1, " and drone ", drone2, ": ", distance)
                            # Verify connection status
                            if not Utils.check_connected(distance, L_s=gaussian_roll(ARGS.system_losses[0],
                                                                                     ARGS.system_losses[1]),
                                                         L_misc=gaussian_roll(ARGS.miscellaneous_losses[0],
                                                                              ARGS.miscellaneous_losses[1])):
                                # Link broken, remove edge
                                print("---LINK BROKEN--- Drone ", drone1, " and ", drone2, " disconnected")
                                swarm.delete_edge(drone1, drone2)
                            found_delay = Utils.find_delay(distance,
                                                           processing=Utils.gaussian_roll(ARGS.processing_delays[0],
                                                                                          ARGS.processing_delays[1]))
                            # add and record delays
                            # if delay_register[node] < found_delay:
                            delay_register[node] = delay_register[node] + found_delay
                        last_item = path_item
                # print(delay_register)

                # find time between each step (i iteration)
                step_time = 1 / env.SIM_FREQ

                for j in range(len(delay_register)):  # for each drone
                    if swarm.linked_to_head(j):  #check if connected to head
                        wp_counters[j] = wp_counters[0]  # bring time in line with head
                        # divide each delay by the time between each step
                        step_setback = delay_register[j] / step_time
                        setback = math.ceil(step_setback)
                        # print("Setback for drone ", j, " is ", setback)
                        wp_counters[j] -= setback
                        if wp_counters[j] < 0:
                            wp_counters[j] = 0  # prevent index error

                #### Save current results #
                curr_time = math.floor(i / env.SIM_FREQ)
                for j in range(ARGS.num_drones):
                    position_results[j][curr_time] = env.pos[j]
                    delay_results[j][curr_time] = delay_register[j]

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
        # swarm.print_graph()
        # print(swarm.find_ultimate_links(swarm.get_head()))

        #### Save the simulation results ###########################
        logger.save()

        # Additional calculations
        error_pos = np.zeros((swarm.get_size() * PERIOD, 3))  # percentage error
        # Find positional error (where it should be if there was zero delay)
        # abs(curr_pos - (Init_xyzs + Mvmntvector*time step))
        for j in range(ARGS.num_drones):
            for t in range(PERIOD):
                indexed_pos = position_results[j][t]
                initial_pos = INIT_XYZS[j]
                indexed_time = flight_vector * (t * ARGS.control_freq_hz)
                percent_error = 100 * ((np.abs(indexed_pos - (initial_pos + indexed_time))) /
                                                   (initial_pos + indexed_time))
                percent_error[np.isnan(percent_error)] = 0.0  # Check for NaN values
                error_pos[j * PERIOD + t] = percent_error

        if ARGS.output_data:
            # Save arrays into a CSV in corresponding directories
            delayDataFPath = ARGS.simulation_title + "DelayData_s" + str(ARGS.movement_rate)
            posDataFPath = ARGS.simulation_title + "PosData_s" + str(ARGS.movement_rate)
            posErrorFPath = ARGS.simulation_title + "PosError_s" + str(ARGS.movement_rate)

            # Ensure folders are created
            make_folder(filepath, delayDataFPath)
            make_folder(filepath + delayDataFPath+"\\", "S" + str(ARGS.movement_rate) + "_R" + str(ARGS.drone_spacing))
            make_folder(filepath, posDataFPath)
            make_folder(filepath + posDataFPath+"\\", "S" + str(ARGS.movement_rate) + "_R" + str(ARGS.drone_spacing))
            make_folder(filepath, posErrorFPath)
            make_folder(filepath + posErrorFPath+"\\", "S" + str(ARGS.movement_rate) + "_R" + str(ARGS.drone_spacing))

            #Reshape output
            output_pos = position_results.reshape((position_results.shape[0] * position_results.shape[1],
                                                   position_results.shape[2]))

            # Save CSV
            np.savetxt(filepath + delayDataFPath + "\\" + "S" + str(ARGS.movement_rate) + "_R" +
                       str(ARGS.drone_spacing) + "\\" + ARGS.simulation_title + "_Delay_Data_" + str(sim_iter) + "_s" +
                       str(ARGS.movement_rate) + "_r" + str(ARGS.drone_spacing) + ".csv", delay_results, delimiter=",")
            np.savetxt(filepath + posDataFPath + "\\" + "S" + str(ARGS.movement_rate) + "_R" + str(ARGS.drone_spacing)
                       + "\\" + ARGS.simulation_title + "_Pos_Data_" + str(sim_iter) + "_s"
                       + str(ARGS.movement_rate) + "_r" + str(ARGS.drone_spacing) + ".csv", output_pos, delimiter=",")
            np.savetxt(filepath + posErrorFPath + "\\" + "S" + str(ARGS.movement_rate) + "_R" +
                       str(ARGS.drone_spacing) + "\\" + ARGS.simulation_title + "_Pos_error_" + str(sim_iter) + "_s" +
                       str(ARGS.movement_rate) + "_r" + str(ARGS.drone_spacing) + ".csv", error_pos, delimiter=",")

        #### Plot the simulation results ###########################
        if ARGS.plot:
            logger.plot()
