import csv
import os.path
from numpy import sqrt, random
from re import findall
from course4_week1_proj import A_star
from Plot_tree_c4w2 import Plot_tree

# Tuning parameters
MAX_NODES = 100
START_NODE_LOC = [-0.5,-0.5]
END_NODE_LOC = [0.5, 0.5]
MAX_PATHS = 5

# Characteristic
ROBOT_DIAMETER = 0.08
SEARCH_SPACE = [START_NODE_LOC, END_NODE_LOC]

""" This script solves the RRT problem it requires input file:
     1. obstacles.csv
     x | y | diameter
     
     It gives output files:
     2. edges.csv
     ID(start) | ID(next) | edge cost
     3. nodes.csv
     ID | x | y | heuristic_cost

     Paths are checked to be collision free
     path.csv solution is found using Astar (class)
     Nodes inside obstacles and coinciding with obstacles are removed

     Written by Aditya Wagh (adi25wagh@gmail.com) on 13/01/2024
"""

# Path files will be stored in the folder:
path_folder = os.path.join(os.path.split(__file__)[0], "Course4_week2_solution")

def read_obstacles():
    """ Place obstacles file in the solution folder"""
    with open(os.path.join(path_folder, "obstacles.csv"), "r") as obstacles_file:
        file = csv.reader(obstacles_file)
        res = []
        for line in file:
            x = findall("^#", line[0])
            if not x:
                res.append([float(line[0]), float(line[1]), float(line[2])])

    return res

def write_file(edge_list, name_file):
    """ Write Edges/Nodes file from computed edges struct"""
    with open(os.path.join(path_folder, name_file),"w", newline = "") as edge_file:
        file = csv.writer(edge_file)
        for edge in edge_list:
            file.writerow(edge)

def write_path(result_path):
    """Write path to pathfile """
    with open(os.path.join(path_folder, "path.csv"),"w", newline = "") as pathf:
        path = csv.writer(pathf)
        path.writerow(result_path)


""" NODE CLASS, useful for representing current node
    Finds nodes using uniform random distribution and
    computes heuristic for node"""
class Node:
    def __init__(self, ID, obstacles):
        self.id = ID
        self.obstacles = obstacles
        self.point = self.get_loc()
        self.heu = self.get_heuristic(self.point)

    def represent_node(self):
        """ Write node as a list """
        return [self.id, self.point[0], self.point[1], self.heu]

    def get_loc(self):
        """ Get location of node using a uniform random distribution over the C-space """
        xmin, ymin = SEARCH_SPACE[0]
        xmax, ymax = SEARCH_SPACE[1]

        search_flag = True
        while search_flag != False:
            x_loc = (xmax-xmin)*random.random_sample() + xmin
            y_loc = (ymax-ymin)*random.random_sample() + ymin
            search_flag = self.check_obstacles(x_loc, y_loc)
        
        return [round(x_loc,5), round(y_loc,5)]

    def check_obstacles(self, x, y):
        """ Check if any node appears in an obstacle """
        obstacle_flags = []
        for obs in self.obstacles:
            obstacle_flags.append( sqrt( (x - obs[0])**2 + (y - obs[1])**2 )
                                  < obs[2] / 2 + ROBOT_DIAMETER / 2)
        return any(obstacle_flags)
    
    def get_heuristic(self, point):
        """ Heuristic is eucledian distance of node from goal node """
        x, y = point
        x_end, y_end = END_NODE_LOC
        dx = x - x_end
        dy = y - y_end

        return sqrt(dx*dx + dy*dy)
    

""" PRM:
    Takes start point and end point, computes intermediate nodes and edges
    outputs them as lists """
class PRM:
    def __init__(self, obstacles):
        self.path = None
        self.start = START_NODE_LOC
        self.end = END_NODE_LOC
        self.max_nodes = MAX_NODES
        self.max_paths = MAX_PATHS
        self.obstacles = obstacles
        self.nodes = self.get_nodes()
        self.edges = self.get_edges(self.nodes.copy())

    def output_tree(self):
        """Output nodes and edges as lists """
        return self.nodes, self.edges

    def get_nodes(self):
        """ Add star node and end node to randomly generated nodes """
        nodes = []
        nodes.append([1, self.start[0], self.start[1], 2.0])
        for i in range(2,self.max_nodes):
            node_instance = Node(i,self.obstacles)
            nodes.append(node_instance.represent_node())

        nodes.append([self.max_nodes, self.end[0], self.end[1], 0.0])
        return nodes
    
    def get_edges(self, nodes):
        """ Compute edges by finding possible paths wrt. obstacles """
        edges = []
        for node_start in nodes:
            targets = self.find_paths(nodes.copy(), node_start, self.max_paths)

            for target in targets:
                edges.append([node_start[0], target[0], self.get_dist(node_start, target)])

        return edges

    def find_paths(self, nodes, current, max_paths):
        """ Finds paths from each node to 'max paths' number of nearby nodes and removes paths
            within obstacles  """
        sorted_nodes = self.sort_nodes(nodes, current)
        target_nodes = []

        i = 0
        while (i < max_paths-1):
            flag = False
            itr = 1
            while flag != True:
                flag = self.check_obstacle_paths(current, sorted_nodes[itr])
                if itr > max_paths:
                    itr += 1
                    break
                itr += 1
            
            target_nodes.append(sorted_nodes[itr-1])
            sorted_nodes.remove(sorted_nodes[itr-1])
            i += 1

        return target_nodes

    def sort_nodes(self, nodes, current):
        """ select nearby nodes based on euclidean distance wrt. current node """
        x = current[1]
        y = current[2]
        res = []
        while len(nodes) > 0:
            dist = 15
            for n in nodes:
                dx = n[1] - x
                dy = n[2] - y
                euc = sqrt(dx*dx + dy*dy)
                if euc < dist:
                    dist = euc
                    nfin = n

            res.append(nfin)
            nodes.remove(nfin)

        return res
    
    def check_obstacle_paths(self, current, target):
        """ Check if a line segment intersects a circle:
            Let:
            ## Triangle formed from 3 known points in space
            points are of form: P(x,y)
            vectors are of form: v(x,y) = xi + yj 
            (i,j, are unit vectors parallel to x,y axes) 
            
            A = point of origin
            B = point of destination
            C = Centre of circle

            u = direction vector for line from A->B
            t = parameter to localize E on line AB
            E = point of intersection (occurs on u)

            l(AB) = L = sqrt((A(x)-B(x))^2 + (A(y) - B(y))^2)

            u(x) = (A(x)-B(x)) / L
            u(y) = (A(y)-B(y)) / L

            t = u(x)*(C(x)-A(x)) + u(y)*(C(y)-A(y))
            E(x) = t*u(x) + A(x)
            E(y) = t*u(y) + A(y)

            l(EC) = sqrt((E(x)-C(x))^2 + (E(y) - C(y))^2)

        """
        flags = []
        Ax = current[1]
        Ay = current[2]
        Bx = target[1]
        By = target[2]

        L = sqrt( (Ax-Bx)**2 + (Ay-By)**2 )
        ux = (Bx - Ax) / L
        uy = (By - Ay) / L
        
        for obs in self.obstacles:
            Cx, Cy, circle_diameter = obs

            t = ux*(Cx-Ax) + uy*(Cy-Ay)

            Ex = t*ux + Ax
            Ey = t*uy + Ay

            LEC = sqrt((Ex - Cx)**2 + (Ey-Cy)**2)
            flags.append(LEC < (circle_diameter + ROBOT_DIAMETER)/2)
        
        return any(flags)

    
    def get_dist(self, node1, node2):
        return sqrt( (node1[1] - node2[1])**2 + (node1[2] - node2[2])**2 ) 
    
### Execution script
def main():
    # input is obstacles.csv
    obstacles = read_obstacles()
    tree = PRM(obstacles)
    nodes, edges = tree.output_tree()

    # find path using A*
    a_star = A_star(edges, nodes)
    path = a_star.solve_A_star()
    print(path)

    Plot_tree(edges, nodes, obstacles)

    ## Output everything into CSV file
    write_file(nodes, "nodes.csv")
    write_file(edges, "edges.csv")
    write_path(path)

if __name__ == "__main__":
    main()





        


