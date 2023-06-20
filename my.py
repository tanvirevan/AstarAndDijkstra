import heapq
import math
import time

class Vertex:
    def __init__(self, name):
        self.name = name
        self.adjacent = {}
        self.distance = float('inf')
        self.visited = False
        self.previous = None

    def add_neighbor(self, neighbor, distance):
        self.adjacent[neighbor] = distance

    def __lt__(self, other):
        return self.distance < other.distance

def heuristic(coord1,coord2):
    x1, y1, z1 = coord1
    x2, y2, z2 = coord2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

def read_graph(distances_file, coordinates_file):
    graph = {}
    
    # Read distances file
    with open(distances_file, 'r') as f:
        for line in f:
            source, destination, distance = line.strip().split(',')
            distance = float(distance)
            
            if source not in graph:
                graph[source] = Vertex(source)
            
            if destination not in graph:
                graph[destination] = Vertex(destination)
            
            graph[source].add_neighbor(graph[destination], distance)
            graph[destination].add_neighbor(graph[source], distance)
    
    # Read coordinates file
    with open(coordinates_file, 'r') as f:
        for line in f:
            name, x, y, z = line.strip().split(',')
            x, y, z = float(x), float(y), float(z)
            
            if name in graph:
                graph[name].coordinates = (x, y, z)
    
    return graph

total1 = 0 ; 
def dijkstra(start):
    start1 = time.time()
    start.distance = 0
    queue = [(0, start)]

    while queue:
        current_distance, current_vertex = heapq.heappop(queue)

        if current_vertex.visited:
            continue

        current_vertex.visited = True

        for neighbor, distance in current_vertex.adjacent:
            new_distance = current_distance + distance

            if new_distance < neighbor.distance:
                neighbor.distance = new_distance
                neighbor.previous = current_vertex
                heapq.heappush(queue, (new_distance, neighbor))
    end1 = time.time()
    total1 = (end1 - start1)*1000
    
total2 = 0
def a_star(start, destination, heuristic):
    start2 = time.time()
    start.distance = 0
    queue = [(start.distance, start)] 
    while queue: 
        _, current_vertex = heapq.heappop(queue)

        if current_vertex.visited:
            continue

        current_vertex.visited = True

        if current_vertex == destination:
            break

        for neighbor, distance in current_vertex.adjacent.items():
            new_distance = current_vertex.distance + distance

            if new_distance < neighbor.distance:
                neighbor.distance = new_distance
                neighbor.previous = current_vertex
                total_cost = new_distance + heuristic(neighbor, destination)
                heapq.heappush(queue, (total_cost, neighbor))
    end2 = time.time()
    total2 = (end2-start2)*1000

def shortest_path(destination):
    path = []
    current_vertex = destination

    while current_vertex is not None:
        path.append(current_vertex.name)
        current_vertex = current_vertex.previous

    return path[::-1]

# File paths
distances_file = 'distances.csv'
coordinates_file = 'coordinates.csv'

# Read the graph from files
graph = read_graph(distances_file, coordinates_file)

# Set the start and destination vertices
start_vertex = graph['TRAPPIST-1']
destination_vertex = graph['55 Cancri']

# Execute A* search algorithm
a_star(start_vertex, destination_vertex, heuristic)
dijkstra(start_vertex)
# Get the shortest path from start to destination
path = shortest_path(destination_vertex)
path1 = shortest_path(destination_vertex)
# Print the path
print("Shortest path:", path)
print("Time need A* algorithm: ",total2," milliseconds")
print("Shortest path:",path1)
print("Time need dijkstra algorithm: ",total1," milliseconds")

