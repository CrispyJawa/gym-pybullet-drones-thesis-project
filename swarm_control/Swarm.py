import string
import Graph
import numpy as np


class Swarm:
    def __init__(self,
                 num_drones: int,
                 comm_range: float,
                 pkg_size: int,
                 send_rate: int,
                 topography: string = "star"):
        self.NUM_DRONES = num_drones
        self.COMM_RANGE = comm_range
        self.PKG_SIZE = pkg_size
        self.SEND_RATE = send_rate
        self.TOPOGRAPHY = topography  # determine node topography based on preset. If unrecognised, should use default
        if(self.TOPOGRAPHY=="star"):
            self.SWARM_MAP=Graph()
            for i in range(self.NUM_DRONES):
                self.SWARM_MAP.add_node(i)


