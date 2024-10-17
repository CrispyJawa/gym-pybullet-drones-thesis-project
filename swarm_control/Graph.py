import numpy as np
from swarm_control import Utils
from swarm_control.Utils import gaussian_roll


# Graph is a 2D list of all connections between nodes. In a swarm context, this effectively represents links between
# drones
class Graph:
    # Edges parameter is a list of tuples, indicating links between nodes.
    def __init__(self,
                 edges,
                 size: int = 0,
                 head: int = 0):
        self.SIZE = size
        self.HEAD = head
        self.GRAPH = [[] for _ in range(size)]
        self.VISITED = []
        self.QUEUE = []
        for u, v in edges:
            # Since the graph is undirected, push the edges in both directions
            self.GRAPH[u].append(v)
            self.GRAPH[v].append(u)

    def delete_node_edges(self, droneID):
        # deletes all links on a given node, effectively disconnecting it
        for i in range(self.SIZE):
            try:
                self.delete_edge(droneID, i)
                print("deleted edge", i)
            except ValueError:
                pass  # Not an existing link

    def add_edge(self, droneID1, droneID2):
        if droneID1 == droneID2:  # if the ends point to the same place, ignore
            pass
        else:
            # if the end tag isn't already listed
            if droneID2 not in self.GRAPH[droneID1]:
                self.GRAPH[droneID1].append(droneID2)
            # if the start tag isn't already listed
            if droneID1 not in self.GRAPH[droneID2]:
                self.GRAPH[droneID2].append(droneID1)

    def delete_edge(self, droneID1, droneID2):
        try:
            self.GRAPH[droneID1].remove(droneID2)
            self.GRAPH[droneID2].remove(droneID1)
        except ValueError:
            print("Indicated connection doesn't exist")

    def print_graph(self):
        for i in range(self.SIZE):
            print(str(i) + ": " + str(self.GRAPH[i]))

    def get_neighbours(self, droneID):
        neighbours = []
        for i in range(len(self.GRAPH[droneID])):
            neighbours.append(self.GRAPH[droneID][i])
        return neighbours

    def find_neighbour(self, droneID1, droneID2):
        found_neighbour = False
        row = self.GRAPH[droneID1]  # Define which drone needs to find neighbour
        for i in row:
            if i == droneID2:  # if found the defined neighbour
                found_neighbour = True
                break
        return found_neighbour

    # Check to see if the drone has any links (ie is it connected)
    def has_links(self, droneID1):
        # links_avail = np.zeros(self.SIZE)
        link_check = False
        for i in range(self.SIZE):
            if i != droneID1:  # for each other drone...
                if self.find_neighbour(droneID1, i):  # ...see if indexed drone is a neighbour
                    # links_avail[i] = 1
                    link_check = True
                    break
        return link_check

    def find_ultimate_links(self, head):
        paths = {head: [head]}
        self.QUEUE.append(head)

        while self.QUEUE:
            curr = self.QUEUE.pop(0)
            for neighbour in self.get_neighbours(curr):
                if neighbour not in paths:
                    paths[neighbour] = paths[curr] + [neighbour]
                    self.QUEUE.append(neighbour)

        return paths

    def linked_to_head(self, droneID1):
        qLinked = False
        ultimate_links = self.find_ultimate_links(self.get_head())
        if droneID1 in ultimate_links:
            qLinked = True
        return qLinked

    def get_head(self):
        return self.HEAD

    def get_size(self):
        return self.SIZE


if __name__ == '__main__':
    edges = (0, 2), (0, 7), (0, 8), (1, 2), (1, 3), (2, 4), (2, 7), (3, 6), (3, 8), (6, 8)
    # #(0, 1), (1, 2), (0, 3), (0, 7), (1, 6), (2, 5), (3, 4), (4, 5) #(0, 1), (1, 2), (2, 3)
    # print("Graph")
    g = Graph(edges, 9, 0)
    # g.print_graph()
    # print("Nodes connected to 0")
    # print(g.get_neighbours(0))
    # print("Cluster head:", g.get_head())
    # print("Cluster size:", g.get_size())
    # print("Find connection between 2 and 0: ", (g.find_neighbour(2, 0)))
    # print("Nodes connected to each other through the swarm:", g.find_ultimate_links(g.get_head()))
    # if Utils.check_connected(dist=60.0, L_s=gaussian_roll(3.0, 1.0), L_misc=gaussian_roll(1.0, 1.0)):
    #     print("Drones are in range and talking clearly")
    # else:
    #     print("Drones are not in range or there is too much noise")
    # print("Estimated delay time:", Utils.find_delay(processing=gaussian_roll(0.025, 0.01)), "s")
    # g.delete_edge(1, 2)
    # g.delete_edge(0, 2)
    # paths_taken = g.find_ultimate_links(g.get_head())
    # print("Is 5 here? ", g.linked_to_head(5))
    # print("Is 7 here? ", g.linked_to_head(7))
    # print("check array again", paths_taken)
    # print("Length of path to node 2: ", len(paths_taken[2]))  # use len and indexed node to find "depth"
    # for node in paths_taken:
    #     print(node)
    #     last_item = g.get_head()
    #     for path_item in paths_taken[node]:
    #         if path_item != g.get_head():
    #             drone1 = last_item
    #             drone2 = path_item
    #             print("    ", drone1, "->", drone2)
    #         last_item = path_item

    drone_positions = np.array([[0, 0, 1],
                                [5, 0, 1],
                                [10, 0, 1],
                                [15, 0, 1],
                                [0, 5, 1],
                                [5, 5, 1],
                                [10, 5, 1],
                                [15, 5, 1]], dtype=float)
    swarm_pattern = Utils.optimal_topology_pattern(drone_positions)
    SWARM = Graph(swarm_pattern, len(drone_positions), head=0)
    print("Graph generated with receiver sensitivity optimal strategy")
    SWARM.print_graph()
    print(SWARM.find_ultimate_links(SWARM.get_head()))


