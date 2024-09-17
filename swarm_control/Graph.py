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

    # def add_node(self, droneID, source: int, dest: int):
    #     # Specify new node's existence, and two links it connects to
    #     if droneID not in self.GRAPH:  # try/catch find_node_idx
    #         print("Node added")
    #         self.SIZE += 1
    #     self.add_edge(source, droneID)
    #     self.add_edge(dest, droneID)

    def delete_node_edges(self, droneID):
        # deletes all links on a given node, effectively disconnecting it
        for i in range(self.SIZE):
            try:
                self.delete_edge(droneID, i)
                print("deleted edge", i)
            except ValueError:
                pass  # Not an existing link

    # def find_node_idx(self, droneID):
    #     try:
    #         output = self.GRAPH.index(droneID)
    #     except ValueError:
    #         print("Did not find the drone", droneID)
    #         output = 99 #error code
    #     return output

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

    # def find_ultimate_links(self, node, check_array, depth=0):
    #     # # check_array = np.zeros(self.SIZE)  # Create this array of each node. 0 if disconnected, 1 if connected.
    #     # # Verify connecting line to a node exists, recursive function
    #     # check_array[node] = depth  # Head always connected to itself
    #     # neighbours = self.GRAPH[node]  # list of neighbours connected directly to this node
    #     # for i in range(len(neighbours)):
    #     #     # skip if already in the check_array
    #     #     curr_node = neighbours[i]
    #     #     if check_array[curr_node] == 0:
    #     #         check_array = self.find_ultimate_links(curr_node, check_array, depth+1)
    #     # return check_array
    #
    #     # QUEUE is a list of drones waiting to be checked
    #     # VISITED is a list of drones already checked
    #     # Source for BFS: https://www.artofcse.com/learning/implement-breadth-first-search-%28bfs%29-graph-algorithm-in-python-using-recursion
    #     check_array[node] = depth  # each value in the check array represents its depth, and position is droneID
    #     if len(self.QUEUE) == 0:  # separate head into its own level (queue empty means only head)
    #         self.QUEUE.append(-1)  # -1 represents a level break
    #
    #     if node not in self.VISITED:
    #         self.VISITED.append(node)  # visit node
    #     for neighbour in self.GRAPH[node]:
    #         if neighbour not in self.VISITED:  # add every node not visited and connected to this one
    #             self.QUEUE.append(neighbour)
    #             self.VISITED.append(neighbour)
    #     if self.QUEUE[0] == -1:  # once you're at the end of the level...
    #         self.QUEUE.append(-1)  # ...add break to indicate level
    #
    #     if len(self.QUEUE) != 0:  # if the queue has values left, start following them in the graph
    #         vertex = self.QUEUE.pop(0)
    #         if vertex == -1:  # if you've reached a break, try to skip it
    #             try:
    #                 while vertex == -1:
    #                     vertex = self.QUEUE.pop(0)  # pass break and increment level depth
    #                 self.find_ultimate_links(vertex, check_array, depth + 1)
    #                 for i in range(len(self.VISITED)):
    #                     self.VISITED.pop(0)
    #             except IndexError:
    #                 pass  # do nothing if break is last item in queue
    #         else:
    #             self.find_ultimate_links(vertex, check_array, depth)
    #
    #     else:  #finished, clear all stacks
    #         for i in range(len(self.VISITED)):
    #             self.VISITED.pop(0)
    #
    #     return check_array
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

    def get_head(self):
        return self.HEAD

    def get_size(self):
        return self.SIZE


if __name__ == '__main__':
    edges = (0, 2), (0, 7), (0, 8), (1, 2), (1, 3), (2, 4), (2, 7), (3, 5), (3, 6), (3, 8), (4, 5), (5, 6), (6, 8)
    #(0, 1), (1, 2), (0, 3), (0, 7), (1, 6), (2, 5), (3, 4), (4, 5) #(0, 1), (1, 2), (2, 3)
    print("Graph")
    g = Graph(edges, 9, 0)
    g.print_graph()
    print("Nodes connected to 0")
    print(g.get_neighbours(0))
    print("Cluster head:", g.get_head())
    print("Cluster size:", g.get_size())
    print("Find connection between 2 and 0: ", (g.find_neighbour(2, 0)))
    print("Nodes connected to each other through the swarm:", g.find_ultimate_links(g.get_head()))
    if Utils.check_connected(dist=60.0, L_s=gaussian_roll(3.0, 1.0), L_misc=gaussian_roll(1.0, 1.0)):
        print("Drones are in range and talking clearly")
    else:
        print("Drones are not in range or there is too much noise")
    print("Estimated delay time:", Utils.find_delay(processing=gaussian_roll(0.025, 0.01)), "s")
    g.delete_edge(1, 2)
    g.delete_edge(0, 2)
    paths_taken = g.find_ultimate_links(g.get_head())
    print("check array again", paths_taken)
    print("Length of path to node 2: ", len(paths_taken[2]))  # use len and indexed node to find "depth"
    for node in paths_taken:
        print(node)
        last_item = g.get_head()
        for path_item in paths_taken[node]:
            if path_item != g.get_head():
                drone1 = last_item
                drone2 = path_item
                print("    ", drone1, "->", drone2)
            last_item = path_item
