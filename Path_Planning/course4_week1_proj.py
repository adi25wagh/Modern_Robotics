import csv
import os.path

from collections import defaultdict

"""
This script solves the A-star path planning problem using two files:
nodes.csv, edges.csv
Nodes is given  of the format:
ID | x | y | heuristic cost

Edges is given of the format:
ID(start) | ID(next) | edge cost

Paths are checked to be collision free and then optimized
Output is saved as path.csv

Written by Aditya Wagh (adi25wagh@gmail.com) on 23/12/2023
"""

# Read path files (edges, nodes) and define output as results
Path_folder = os.path.join(os.path.split(__file__)[0], 'Course4_week1_solution')

def read_nodes():
    """ Read the nodes file with the heuristic cost.
    XY locations will be used to check for collisions in loop"""
    with open(os.path.join(Path_folder, 'nodes.csv')) as nodes_file:
        nodes = csv.reader(nodes_file)
        res = []

        for node in nodes:
            ID, x, y, heu = node
            res.append([int(ID), float(x), float(y), float(heu)])

        return res
    
def read_edges():
    """ Read edges files to process data as matrices"""
    with open(os.path.join(Path_folder, 'edges.csv')) as edges_file:
        edges = csv.reader(edges_file)
        res = []

        for edge in edges:
            ID1, ID2, cost = edge
            res.append([int(ID1), int(ID2), float(cost)])

        return res
    
def read_obstacles():
    """ Read file with Obstacle locations """
    with open(os.path.join(Path_folder, 'obstacles.csv')) as obstacles_file:
        obs = csv.reader(obstacles_file)
        res = []

        for o in obs:
            xo, yo, circ = o
            res.append([float(xo), float(yo), float(circ)])

        return res
    
def write_path_file(result_path):
    """ Write resulting path list to file """
    with open(os.path.join(Path_folder, 'path.csv'), 'w') as path_file:
        path_csv = csv.writer(path_file)
        path_csv.writerow(result_path)


class OpenSet:
    """ Nodes that can still be reached"""
    def __init__(self):
        self.dict = {}

    def insert_node(self, value, cost_to_reach):
        """ Nodes are stored with total cost to get.
        Unsorted, sorting conducted when retrieving"""
        self.dict[value] = cost_to_reach

    def get_first(self):
        """ Get the first node in the open set.
        Delete from dictionary when extracted"""
        ID_out = None
        cost_out = None
        
        # Here sorting the IDs to get minimum cost
        for ID, cost in self.dict.items():
            if not cost_out or cost < cost_out:
                ID_out = ID
                cost_out = cost
        
        del self.dict[ID_out]
        return ID_out

    def check_if_empty(self):
        """ Check if open is empty """
        return not bool(self.dict)
    
class A_star:

    """ Start with initial conditions """
    def __init__(self, edges, nodes):
        self.open = OpenSet()
        self.closed = set()
        self.edges = self.init_edges(edges)
        self.nodes = self.init_nodes(nodes)

        # Initial conditions
        self.start = nodes[0][0] #ID of start node
        self.end = nodes[-1][0] #ID of end node

        # Initialize minimum cost to go to high value
        self.past_cost = defaultdict(lambda: 1000)
        self.parent = {}

        # add start node to open_list
        self.open.insert_node(self.start, self.nodes[self.start][-1])
        self.past_cost[self.start] = 0

    def init_edges(self, edges):
        """ Initialize costs per edge """
        res = defaultdict(lambda: defaultdict(lambda: None))
        for edge in edges:
            start, stop, cost = edge
            res[start][stop] = cost

        # Eliminate multiple copies of same costs 
        res_cp = res.copy()
        for i in res_cp.keys():
            for j in res_cp[i].keys():
                if i != j:
                    if res_cp[i][j]:
                        res[j][i] = res[i][j]
        
        return res
    
    def init_nodes(self,nodes):
        """ Heuristics included in the nodes file,
        Needed to know XY location for checking with each obstacle."""
        res = {}
        for n in nodes:
            node, xloc, yloc, cost = n
            res[node] = [xloc, yloc, cost]

        return res 

    
    def solve_A_star(self):
        """ Find path with A*"""
        while not self.open.check_if_empty():
            current = self.open.get_first()
            if current == self.end:
                return self.get_path(self.end)
            self.closed.add(current)

            for nbr in self.get_neighbors(current):
                self.process_neighbor(current, nbr)
        return None
    
    def get_neighbors(self, node):
        """ Find Neighbors"""
        res = set()
        for k1, v1 in self.edges.items():
            for k2, v2 in v1.items():
                if v1 and v2 and node in (k1, k2):
                    res.add(k1)
                    res.add(k2)
        
        return [n for n in res if n not in self.closed]


    def process_neighbor(self, node, nbr):
        """ Process neighbor using edge cost 
        (NOT Heuristic)"""
        past_cost = self.past_cost[node] + self.edges[node][nbr]
        if past_cost < self.past_cost[nbr]:
            self.update_path(node, nbr, past_cost)

    def update_path(self, node, nbr, past_cost):
        """ Update the best path found so far """
        self.past_cost[nbr] = past_cost
        self.parent[nbr] = node

        # Here heuristics are included
        total_cost = self.past_cost[nbr] + self.nodes[nbr][-1]
        self.open.insert_node(nbr, total_cost)

    def get_path(self, node):
        path = []
        itr = node
        while itr != self.start:
            path = [itr] + path
            itr = self.parent[itr]
        path = [self.start] + path

        return path
        


""" Run the files and extract the path """
def main():
    # Nodes file
    nodes = read_nodes()
    # Edges file
    edges = read_edges()
    # Obstacles
    obstacles = read_obstacles()


    a_star = A_star(edges, nodes, obstacles)
    path = a_star.solve_A_star()

    assert path == [1, 2, 5, 7, 10, 12], print('Wrong result: Check code')
    write_path_file(path)

if __name__ == "__main__":
    main()