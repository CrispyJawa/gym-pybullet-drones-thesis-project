import numpy as np


# pick a random value from a normal distribution
def gaussian_roll(mu: float = 0.0,
                  sigma: float = 1.0):
    return np.random.normal(mu, sigma)


# decibel-milli-watts to milli-watts
def dbm_to_mw(dbm: float):
    return 10.0 ** (dbm / 10.0)


# milli-watts to decibel-milli-watts
def mw_to_dbm(mw: float):
    return 10.0 * np.log10(mw)


def pythag_3D(a: float, b: float, c: float):
    return np.sqrt(a ** 2 + b ** 2 + c ** 2)


# Free space path loss calculation
def calc_fspl(distance: float, frequency: float):
    return 20 * np.log10(distance) + 20 * np.log10(frequency) - 147.55


# Verify connectivity between two drones. Parameters are: Distance (m), frequency (Hz), transmit power (dBm),
# transmit gain (dBi), receiver gain (dBi), system loss (dB), miscellaneous loss (dB), receiver sensitivity (dBm),
# bandwidth (Hz), SNR Threshold (dB) and Fade Margin (dB)
def check_connected(dist: float = 50.0, freq: float = 2450000000.0, P_t: float = 5, G_t: float = 2.8,
                    G_r: float = 2.8, L_s: float = 3.0, L_misc: float = 1.0, recv_sens: float = -100.0,
                    bandwidth: float = 2000000.0, SNR_threshold: float = 20.0, fade_margin: float = 15.0):
    link_budget = find_Link_Budget(dist, freq, P_t, G_t, G_r, L_s, L_misc)
    # print("Link budget (dB): ", link_budget)
    P_r = dbm_to_mw(link_budget)  # Received power
    # print("Received Power (dBm): ", P_r)
    SNR = SNR_calc(P_r, bandwidth)
    # Above receiver sensitivity (with fade margin) and above SNR threshold
    if (recv_sens + fade_margin < link_budget) & (SNR > SNR_threshold):
        return True
    else:
        return False


def SNR_calc(P_r: float, bandwidth: float):
    T = 290
    noise = (((1.380649 * (10 ** -23)) * T * bandwidth) * 10 ** 6)  # measure in mW
    # print("Noise: ", noise, "mW")
    SNR = 20 * np.log10(P_r / noise) - gaussian_roll(1, 1)  # Gaussian on noise amount
    # print("SNR: ", SNR, "dB")
    return SNR


def find_Link_Budget(dist: float = 50.0, freq: float = 2450000000.0, P_t: float = 5, G_t: float = 2.8,
                     G_r: float = 2.8, L_s: float = 3.0, L_misc: float = 1.0):
    L_p = calc_fspl(dist, freq)  # Path Loss
    link_budget = P_t + G_t + G_r - L_s - L_p - L_misc  # Link Budget
    # print("Link budget (dB): ", link_budget)
    P_r = dbm_to_mw(link_budget)  # Received power
    # print("Received Power (dBm): ", P_r)
    return link_budget


# Find total delay of a signal. distance (m), processing delay (s), transmission rate (bytes/s), message size (bytes)
def find_delay(distance: float = 50.0, processing: float = 0.030, trans_rate: float = 250000.0,
               msg_size: float = 250.0):
    c = 299792458.0
    delay = distance / c + processing + (msg_size / trans_rate)
    return delay


# Needs gauss, needs some mechanic for queuing of messages


def optimal_topology_pattern(positions, system_loss: float = 3.0, misc_loss: float = 1.0, bandwidth: float = 2000000.0):
    num_drones = len(positions)
    optimal_edges = []
    for j in range(num_drones): # for every drone
        # array ranking best connections to other drones
        sorted_connections = find_best_links(j, positions, system_loss, misc_loss, bandwidth)

        # Pick the top two unconnected, and connect them
        top_links = sort_indices_by_values(sorted_connections)

        # print("TOP LINKS FOR DRONE ", j, " is ", top_links)
        for k in range(2):
            idx = 0
            while link_exists(optimal_edges, [j, top_links[idx]]):
                # print("Link exists!")
                idx += 1
                if idx == num_drones:
                    break
            optimal_edges.append([j, top_links[idx]])

            # If everything is already connected, pick top two and make them redundant connections

    print(optimal_edges)
    return optimal_edges


def find_best_links(droneID, positions, system_loss, misc_loss, bandwidth):
    swarm_size = len(positions)
    connections_sens = np.zeros(swarm_size)
    for k in range(swarm_size):
        if droneID != k:  # for every other drone
            dist = positions[k] - positions[droneID]
            range_between = pythag_3D(*dist[0:3])
            # print("\n",droneID,": ",dist," ", range_between)
            LB = find_Link_Budget(range_between, L_s=system_loss, L_misc=misc_loss)
            SNR = SNR_calc(dbm_to_mw(LB), bandwidth)
            conn_quality = LB/SNR
            connections_sens[k] = conn_quality # RX POWER / SNR
            # print(connections_sens)
        else:
            connections_sens[droneID] = -900.0
    return connections_sens


# GPT generated
def sort_indices_by_values(arr):
    # Create a list of (index, value) pairs
    indexed_arr = list(enumerate(arr))

    # Sort the list by the value in descending order
    sorted_indexed_arr = sorted(indexed_arr, key=lambda x: x[1], reverse=True)

    # Extract and return the indices from the sorted list
    sorted_indices = [index for index, value in sorted_indexed_arr]
    return sorted_indices


# GPT Generated
def link_exists(links, new_link):
    # Sort the new link to treat [a, b] the same as [b, a]
    sorted_new_link = sorted(new_link)

    for link in links:
        # Sort each existing link and compare with the new one
        if sorted(link) == sorted_new_link:
            return True

    return False
