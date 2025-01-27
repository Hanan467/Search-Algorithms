import queue
import math
import csv


class Node:
    def __init__(self, val):
        self.val = val
        self.nbs = []

# Romania map
Arad = Node('Arad')
Bucharest = Node('Bucharest')
Sibiu = Node('Sibiu')
Timisoara = Node('Timisoara')
Lugoj = Node('Lugoj')
Mehadia = Node('Mehadia')
Drobeta = Node('Drobeta')
Rimnicu = Node('Rimnicu')
Pitesti = Node('Pitesti')
Fagaras = Node('Fagaras')
Craiova = Node('Craiova')
Zerind = Node('Zerind')  
Oradea = Node('Oradea') 

Arad.nbs = [Sibiu, Zerind, Timisoara]
Zerind.nbs = [Arad, Oradea]  
Oradea.nbs = [Zerind, Sibiu] 
Sibiu.nbs = [Arad, Fagaras, Rimnicu, Oradea] 
Timisoara.nbs = [Arad, Lugoj]
Lugoj.nbs = [Timisoara, Mehadia]
Mehadia.nbs = [Lugoj, Drobeta]
Drobeta.nbs = [Craiova, Mehadia]
Fagaras.nbs = [Sibiu, Bucharest]
Rimnicu.nbs = [Sibiu, Pitesti, Craiova]
Pitesti.nbs = [Rimnicu, Craiova, Bucharest]
Craiova.nbs = [Rimnicu, Pitesti, Drobeta]
Bucharest.nbs = [Fagaras, Pitesti]

# Example usage
problem = {
    "initial": "Arad",
    "goal": "Bucharest",
    "max_iterations": 1000
}

# Cooling schedule
def schedule(t):
    return max(1e-3, 100 / (1 + t))  # Example schedule

def load_city_coords(filename):
    city_coords = {}
    with open(filename, mode='r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            city = row['City']
            latitude = float(row['Latitude'])
            longitude = float(row['Longitude'])
            city_coords[city] = (latitude, longitude)
    return city_coords

city_coords = load_city_coords('city_coords.csv')

class GraphSearch:
    #Implements graph search algorithms.
    def __init__(self, start_node, target_val):
        self.start_node = start_node
        self.target_val = target_val

    def depth_first_search(self):
        stack = [self.start_node]
        visited = set([self.start_node.val])
        path_map = {}

        while stack:
            node = stack.pop()
            if node.val == self.target_val:
                return self.reconstruct_path(path_map, node)
            for neighbor in node.nbs:
                if neighbor.val not in visited:
                    visited.add(neighbor.val)
                    stack.append(neighbor)
                    path_map[neighbor.val] = node

        raise ValueError("Target not found")

    def breadth_first_search(self):
        queue = [self.start_node]
        visited = set([self.start_node.val])
        path_map = {}

        while queue:
            node = queue.pop(0) #pop the 1st node FIFO
            if node.val == self.target_val:
                return self.reconstruct_path(path_map, node)
            for neighbor in node.nbs:
                if neighbor.val not in visited:
                    visited.add(neighbor.val)
                    queue.append(neighbor)
                    path_map[neighbor.val] = node

        raise ValueError("Target not found")

    def uniform_cost_search(self, cost_map):
        from heapq import heappop, heappush

        priority_queue = [(0, self.start_node)]  # (cost, node)
        visited = set()
        path_map = {}

        while priority_queue:
            cost, node = heappop(priority_queue)
            if node.val in visited:
                continue
            visited.add(node.val)

            if node.val == self.target_val:
                return self.reconstruct_path(path_map, node, cost)

            for neighbor in node.nbs:
                if neighbor.val not in visited:
                    edge_cost = cost_map.get((node.val, neighbor.val), float('inf'))
                    heappush(priority_queue, (cost + edge_cost, neighbor))
                    path_map[neighbor.val] = node

        raise ValueError("Target not found")
    
# A* search algorithm
    def a_star_search(self, cost_map, heuristic_costs):
        from heapq import heappop, heappush

        # Priority queue for A* (f_cost, node)
        start_heuristic = heuristic_costs.get((self.start_node.val, self.target_val), float('inf'))
        priority_queue = [(start_heuristic, 0, self.start_node)]  # (f_cost, g_cost, node)

        visited = set()
        best_costs = {self.start_node.val: 0}  # Track best cost to reach each node
        path_map = {}
        while priority_queue:
            f_cost, g_cost, node = heappop(priority_queue)
            if node.val in visited:
                continue
            visited.add(node.val)
            if node.val == self.target_val:
                return self.reconstruct_path(path_map, node, g_cost)

            for neighbor in node.nbs:
                if neighbor.val not in visited:
                    edge_cost = cost_map.get((node.val, neighbor.val), float('inf'))
                    h_cost = heuristic_costs.get((neighbor.val, self.target_val), float('inf'))
                    #print("hcost between",neighbor.val,"and",self.target_val,h_cost)
                    new_g_cost = g_cost + edge_cost
                    new_f_cost = new_g_cost + h_cost
                    # If we have not visited the neighbor or found a cheaper path to it
                    if neighbor.val not in visited or new_g_cost < best_costs.get(neighbor.val, float('inf')):
                         heappush(priority_queue, (new_f_cost, new_g_cost, neighbor))
                         path_map[neighbor.val] = node
                         best_costs[neighbor.val] = new_g_cost  # Update best cost for the neighbor

        raise ValueError("Target not found")
    
# simulated_annealing
    def simulated_annealing(self, problem, schedule):
        import random

        current = problem["initial"]
        path = [current]
        total_cost = 0

        for t in range(1, problem["max_iterations"]):
            T = schedule(t)
            if T == 0:
                # Ensure the goal node is added to the path if not already reached
                if current != problem["goal"]:
                    #path.append(problem["goal"])
                    # Add the cost to travel to the goal node (if it exists in cost_map)
                    travel_cost = cost_map.get((current, problem["goal"]), cost_map.get((problem["goal"], current), float('inf')))
                    total_cost += travel_cost
                return path, total_cost

            # Randomly select a successor
            neighbors = [key[1] for key in cost_map.keys() if key[0] == current]
            if not neighbors:
                # Ensure the goal node is added to the path if no neighbors are found
                if current != problem["goal"]:
                    #path.append(problem["goal"])
                    travel_cost = cost_map.get((current, problem["goal"]), cost_map.get((problem["goal"], current), float('inf')))
                    total_cost += travel_cost
                return path, total_cost  # No neighbors, return current path and cost
            
            next_city = random.choice(neighbors)
            
            # Calculate cost between current and next city
            travel_cost = cost_map.get((current, next_city), cost_map.get((next_city, current), float('inf')))
            
            # Compute change in value (heuristic difference)
            delta_e = heuristic_costs[(current, problem["goal"])] - heuristic_costs[(next_city, problem["goal"])]
            
            # Transition based on delta_e and temperature
            if delta_e > 0 or random.random() < math.exp(delta_e / T):
                current = next_city
                path.append(current)
                total_cost += travel_cost

            # If we reached the goal, return the path
            if current == problem["goal"]:
                return path, total_cost

        # Ensure the goal node is added to the path if the maximum iterations are reached
        if current != problem["goal"]:
            #path.append(problem["goal"])
            travel_cost = cost_map.get((current, problem["goal"]), cost_map.get((problem["goal"], current), float('inf')))
            total_cost += travel_cost

        return path, total_cost

    # Example usage
    problem = {
        "initial": "Arad",
        "goal": "Bucharest",
        "max_iterations": 1000
    }

    # Cooling schedule
    def schedule(t):
        return max(1e-3, 100 / (1 + t))  # Example schedule

    def reconstruct_path(self, path_map, node, cost=None):
        path = [node.val]
        while node.val in path_map:
            node = path_map[node.val]
            path.insert(0, node.val)
        if cost is not None:
            return path, cost
        return path

# cost map for UCS
cost_map = {
    ('Arad', 'Sibiu'): 140, ('Arad', 'Zerind'): 75, ('Arad', 'Timisoara'): 118,
    ('Zerind', 'Oradea'): 71, ('Oradea', 'Sibiu'): 151, ('Sibiu', 'Fagaras'): 99,
    ('Sibiu', 'Rimnicu'): 80, ('Timisoara', 'Lugoj'): 111, ('Lugoj', 'Mehadia'): 70,
    ('Mehadia', 'Drobeta'): 75, ('Drobeta', 'Craiova'): 120, ('Fagaras', 'Bucharest'): 211,
    ('Rimnicu', 'Pitesti'): 97, ('Rimnicu', 'Craiova'): 146, ('Pitesti', 'Bucharest'): 101,
    ('Pitesti', 'Craiova'): 138
}


# Function to compute Euclidean distance
def euclidean_distance(city_a, city_b):
    lat_a, long_a = city_coords[city_a]
    lat_b, long_b = city_coords[city_b]
    return math.sqrt((lat_b - lat_a)**2 + (long_b - long_a)**2)

# Generate heuristic costs (Euclidean distances)
heuristic_costs = {}
for city_a in city_coords:
    for city_b in city_coords:
        if city_a == city_b:
            heuristic_costs[(city_a, city_b)] = 0  # Heuristic cost to self is 0
        else:
            heuristic_costs[(city_a, city_b)] = euclidean_distance(city_a, city_b)
search = GraphSearch(Arad, 'Bucharest')

# DFS
dfs_result = search.depth_first_search()
print("DFS Result:", dfs_result)

# BFS
bfs_result = search.breadth_first_search()
print("BFS Result:", bfs_result)

# UCS
ucs_result, total_cost = search.uniform_cost_search(cost_map)
print("UCS Result:", ucs_result, "with cost", total_cost)

# A*
a_star_result, a_star_cost = search.a_star_search(cost_map, heuristic_costs)
print("A* Result:", a_star_result, "with cost", a_star_cost)

#Simulated Annealing
sa_result, sa_cost = search.simulated_annealing(problem, schedule)
print("Simulated Annealing Result:", sa_result, "with cost", sa_cost)
