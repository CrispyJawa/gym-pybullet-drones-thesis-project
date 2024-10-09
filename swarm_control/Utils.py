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
    return np.sqrt(a**2 + b**2 + c**2)


# Free space path loss calculation
def calc_fspl(distance: float, frequency: float):
    return 20 * np.log10(distance) + 20 * np.log10(frequency) - 147.55


# Verify connectivity between two drones. Parameters are: Distance (m), frequency (Hz), transmit power (dBm),
# transmit gain (dBi), receiver gain (dBi), system loss (dB), miscellaneous loss (dB), receiver sensitivity (dBm),
# bandwidth (Hz), SNR Threshold (dB) and Fade Margin (dB)
def check_connected(dist: float = 50.0, freq: float = 2450000000.0, P_t: float = 5, G_t: float = 2.8,
                    G_r: float = 2.8, L_s: float = 3.0, L_misc: float = 1.0, recv_sens: float = -100.0,
                    bandwidth: float = 2000000.0, SNR_threshold: float = 20.0, fade_margin: float = 15.0):
    L_p = calc_fspl(dist, freq)  # Path Loss
    # print("Path loss (dB): ", L_p)
    link_budget = P_t + G_t + G_r - L_s - L_p - L_misc  # Link Budget
    # print("Link budget (dB): ", link_budget)
    P_r = dbm_to_mw(link_budget)  # Received power
    # print("Received Power (dBm): ", P_r)
    T = 290  # Kelvin
    noise = (((1.380649 * (10 ** -23)) * T * bandwidth) * 10 ** 6) # measure in mW NEEDS GAUSS
    # print("Noise: ", noise, "mW")
    SNR = 20 * np.log10(P_r / noise) - gaussian_roll(1,1)
    # print("SNR: ", SNR, "dB")
    # Above receiver sensitivity (with fade margin) and above SNR threshold
    if (recv_sens + fade_margin < link_budget) & (SNR > SNR_threshold):
        return True
    else:
        return False


# Find total delay of a signal. distance (m), processing delay (s), transmission rate (bytes/s), message size (bytes)
def find_delay(distance: float = 50.0, processing: float = 0.030, trans_rate: float = 250000.0,
               msg_size: float = 250.0):
    c = 299792458.0
    delay = distance / c + processing + (msg_size / trans_rate)
    return delay
# Needs gauss, needs some mechanic for queuing of messages
