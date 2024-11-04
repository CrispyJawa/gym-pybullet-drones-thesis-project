from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import pandas as pd
import os
import numpy as np
import glob


# Function to load and process multiple CSV files
def load_and_average_positions(file_paths):
    # Create an array to hold the sum of the XYZ positions for each node across all files
    position_sum = np.zeros((8, 10, 3))  # 8 nodes, 10 seconds, 3 (XYZ)

    # Loop through each CSV file and accumulate positions
    for file in file_paths:
        df = pd.read_csv(file, header=None)  # File read

        df.columns = ['x', 'y', 'z']
        # Assume the CSV has 80 rows (8 nodes * 10 seconds)
        for node in range(8):
            for second in range(10):
                # Extract the x, y, z values for the node at this second
                idx = node * 10 + second  # each node takes ten rows, iterate across seconds
                x = df['x'][idx]
                y = df['y'][idx]
                z = df['z'][idx]

                # Accumulate positions
                position_sum[node, second, 0] += x
                position_sum[node, second, 1] += y
                position_sum[node, second, 2] += z

    # Calculate the average position by dividing by the number of files
    avg_position = position_sum / len(file_paths)

    return avg_position


def load_and_average_error(file_paths):
    # Create an array to hold the sum of the XYZ positions for each node across all files
    position_sum = np.zeros((8, 3))  # 8 nodes, 3 (XYZ)
    node_count = 8
    time_steps = 10

    # Loop through each CSV file and accumulate positions
    for file in file_paths:
        # Read CSV without headers
        df = pd.read_csv(file, header=None)

        # Check if the file has 80 rows as expected (8 nodes * 10 seconds)
        if len(df) != node_count * time_steps:
            raise ValueError(f"CSV file {file} does not have {node_count * time_steps} rows. It has {len(df)} rows.")

        # Manually assign the column names to 'x', 'y', 'z' without overwriting any data
        df.columns = ['x', 'y', 'z']

        # Accumulate x, y, z positions for each node over the 10 seconds
        for node in range(node_count):  # 8 nodes
            node_data = df.iloc[node * time_steps:(node + 1) * time_steps]  # Selects all xyz error data for one node

            # Sum up the x, y, and z for this node across all time steps
            position_sum[node, 0] += node_data['x'].sum()  # Sum x values
            position_sum[node, 1] += node_data['y'].sum()  # Sum y values
            position_sum[node, 2] += node_data['z'].sum()  # Sum z values

    # Calculate the overall average for each node by dividing by (number of files * time steps)
    avg_position = position_sum / (len(file_paths) * time_steps)

    return avg_position


def load_and_average_delays(file_paths):
    # Create an array to hold the sum of the averages for each node across all files
    node_averages_sum = np.zeros(8)  # 8 nodes

    # Loop through each CSV file
    for file in file_paths:
        # Read the CSV assuming it has 8 rows (nodes) and 10 columns (time steps)
        df = pd.read_csv(file, header=None)

        # Check if the file has 8 rows and 10 columns
        if df.shape != (8, 10):
            raise ValueError(f"CSV file {file} does not have 8 rows and 10 columns. It has {df.shape}.")

        # Calculate the average across the columns for each row (node)
        # node_averages = df.mean(axis=1)  # Averages across the 10 columns (time steps)
        final_delays = df[:][9]
        # Accumulate the node averages across CSVs for all 8 drones
        node_averages_sum += final_delays

    # Calculate the final average by dividing the sum by the number of files
    overall_average = node_averages_sum / len(file_paths)

    return overall_average


if __name__ == "__main__":
    avg_positions = None
    colours = ['r', 'b', 'g', 'y', 'c', 'm', 'blueviolet', 'k']
    speeds = [0.015, 0.02, 0.025, 0.03]
    ranges = [5.0, 20.0]
    topologies = ["BUS", "OPTIMAL", "TREE"]

    # Positional display
    for topology in topologies:
        fig, axes = plt.subplots(len(speeds), len(ranges), figsize=(len(ranges) * 6, len(speeds) * 4),
                                 subplot_kw={'projection': '3d'})
        plot_idx = 0
        for speed in range(len(speeds)):
            for distance in range(len(ranges)):
                plot_idx += 1
                # Skim through all PosData files
                csv_files = glob.glob(
                    'C:\\Users\\Signature Edition\\Documents\\PyGymDrones\\gym-pybullet-drones-0.5.2\\files'
                    '\\outputs\\' + topology + '_Diagonal_flightPosData_s' + str(speeds[speed]) + '\\S' +
                    str(speeds[speed]) + '_R' + str(ranges[distance]) + '\\*Pos_Data*.csv')

                if len(csv_files) == 0:
                    print('No csv files found.')
                else:
                    avg_positions = load_and_average_positions(csv_files)  # sum all values, then divide
                    # print(avg_positions)

                for drone in range(8):
                    x = avg_positions[drone, :, 0]
                    y = avg_positions[drone, :, 1]
                    z = avg_positions[drone, :, 2]

                    axes[speed, distance].plot3D(x, y, z, c=colours[drone], marker='o')
                axes[speed, distance].set_title("Positions    Speed: " + str(speeds[speed]) + "    Range: " + str(ranges[distance]),
                    fontdict={'fontsize': 14, 'fontweight': 'bold'})
                # axes[speed, distance].title.set_text(
                #     "Speed: " + str(speeds[speed]) + "    Distance between Drones: " + str(ranges[distance]),
                #     fontdict={'fontsize': 16, 'fontweight': 'bold'})
                axes[speed, distance].set_xlabel('X (m from origin)', fontweight='bold')
                axes[speed, distance].set_ylabel('Y (m from origin)', fontweight='bold')
                axes[speed, distance].set_zlabel('Z (m from origin)', fontweight='bold')
        if True:
            plt.subplots_adjust(left=0.07, right=0.93, top=0.98, bottom=0.03, wspace=0.2, hspace=0.3)
            plt.savefig(
                "C:\\Users\\Signature Edition\\Documents\\PyGymDrones\\gym-pybullet-drones-0.5.2\\files\\outputs\\AAA_Images\\AVG_XYZ_" + topology + ".png")
            plt.show()


    # Delays
    for topology in topologies:
        plot_idx = 0
        fig = plt.figure(figsize=(len(ranges)*8, len(speeds)*4))
        for speed in speeds:
            for distance in ranges:
                plot_idx += 1
                # print(topology, speed, distance)
                # Take pos_error files
                csv_files = glob.glob(
                    'C:\\Users\\Signature Edition\\Documents\\PyGymDrones\\gym-pybullet-drones-0.5.2\\files'
                    '\\outputs\\' + topology + '_Diagonal_flightDelayData_s' + str(speed) + '\\S' +
                    str(speed) + '_R' + str(distance) + '\\*Delay_Data*.csv')

                if len(csv_files) == 0:
                    print("No files found")
                else:
                    avg_error = load_and_average_delays(csv_files)
                    # print(avg_error)
                    ax = fig.add_subplot(len(speeds), len(ranges), int(plot_idx))

                    cpick = 'g'
                    if topology == topologies[0]:
                        cpick = 'r'
                    elif topology == topologies[1]:
                        cpick = 'b'
                    ax.bar(range(len(avg_error)), avg_error, color=cpick)
                    ax.set_xticks(np.arange(0,8,1))

                    # Find the index of the highest bar
                    max_value = max(avg_error)
                    max_index = np.argmax(avg_error)

                    # Annotate the highest bar with its value
                    ax.text(float(max_index), max_value - 0.01, f'{max_value:.8f}', ha='center', va='bottom', fontsize=12,
                             color='white',
                            bbox=dict(facecolor='black', alpha=0.8, boxstyle='round,pad=0.3'))

                    ax.set_title("Messaging delays    Speed: " + str(speed) + " Range: " + str(distance), fontdict={'fontsize': 14, 'fontweight': 'bold'})
                    ax.set_xlabel("Drone ID", fontweight='bold')
                    ax.set_ylabel("Average Delay (s)", fontweight='bold')
        plt.subplots_adjust(left=0.1, right=0.9, top=0.94, bottom=0.05, wspace=0.2, hspace=0.3)
        plt.savefig("C:\\Users\\Signature Edition\\Documents\\PyGymDrones\\gym-pybullet-drones-0.5.2\\files\\outputs\\AAA_Images\\DELAY_XYZ_" + topology + ".png")
        plt.show()

    # Positional error
    for topology in topologies:
        plot_idx = 0
        fig = plt.figure(figsize=(len(ranges)*8, len(speeds)*4))
        for speed in speeds:
            for distance in ranges:
                plot_idx += 1
                # print(topology, speed, distance)
                # Take pos_error files
                csv_files = glob.glob(
                    'C:\\Users\\Signature Edition\\Documents\\PyGymDrones\\gym-pybullet-drones-0.5.2\\files'
                    '\\outputs\\' + topology + '_Diagonal_flightPosError_s' + str(speed) + '\\S' +
                    str(speed) + '_R' + str(distance) + '\\*Pos_Error*.csv')

                if len(csv_files) == 0:
                    print("No files found")
                else:
                    avg_error = load_and_average_error(csv_files).mean(axis=1)
                    # print("AVG_ERROR\r\n",avg_error)
                    ax = fig.add_subplot(len(speeds), len(ranges), int(plot_idx))
                    cpick='g'
                    if topology == topologies[0]:
                        cpick='r'
                    elif topology == topologies[1]:
                        cpick='b'
                    ax.bar(range(len(avg_error)), avg_error, color=cpick)
                    ax.set_xticks(np.arange(0,8,1))

                    ax.set_title(
                        "Positioning errors    Speed: " + str(speed) + "   Range: " + str(distance), fontdict={'fontsize': 14, 'fontweight': 'bold'})
                    ax.set_xlabel("Drone ID", fontweight='bold')
                    ax.set_ylabel("Average Error (%)", fontweight='bold')
                    ax.set_ylim(0, 25)
        plt.subplots_adjust(left=0.1, right=0.9, top=0.94, bottom=0.05, wspace=0.2, hspace=0.3)
        plt.savefig("C:\\Users\\Signature Edition\\Documents\\PyGymDrones\\gym-pybullet-drones-0.5.2\\files\\outputs\\AAA_Images\\ERR_XYZ_" + topology + ".png")
        plt.show()
