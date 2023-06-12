import heapq
import math
import csv
from collections import defaultdict

COORDINATES_FILE = "Coordinates.csv"
DISTANCE_FILE  = "distances.csv"

class Vertex:
    def __init__(self, name):
        self.name = name
        self.adjacent = []
        self.distance = float('inf')
        self.visited = False
        self.previous = None

    def add_neighbor(self, neighbor, distance):
        self.adjacent.append((neighbor, distance))

    def __lt__(self, other):
        return self.distance < other.distance

class Graph:
    def __init__(self):
        self.vertices = {}

    def add_vertex(self, vertex):
        self.vertices[vertex.name] = vertex

    def get_vertex(self, name):
        return self.vertices.get(name)

def euclidean_distance(coord1, coord2):
    x1, y1, z1 = coord1
    x2, y2, z2 = coord2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

def a_star(graph, start, heuristic):
    start.distance = 0
    queue = [(0, start)]

    while queue:
        current_distance, current_vertex = heapq.heappop(queue)

        if current_vertex.visited:
            continue

        current_vertex.visited = True

        if current_vertex == destination_vertex:
            break

        for neighbor, distance in current_vertex.adjacent:
            new_distance = current_distance + distance

            if new_distance < neighbor.distance:
                neighbor.distance = new_distance
                neighbor.previous = current_vertex
                total_cost = new_distance + heuristic(neighbor, destination_vertex)
                heapq.heappush(queue, (total_cost, neighbor))

def shortest_path(destination):
    path = []
    current_vertex = destination

    while current_vertex is not None:
        path.append(current_vertex.name)
        current_vertex = current_vertex.previous

    return path[::-1]

# Create the graph and add vertices and edges based on "distances.csv" file
graph = Graph()
# Add vertices and edges based on the information in the file

# Read the coordinates from "Coordinates.csv" file and store them in a dictionary
coordinates = {}
# Read and parse the coordinates from the file

# Define the heuristic function based on the Euclidean distance
def heuristic(vertex, destination):
    coord1 = coordinates[vertex.name]
    coord2 = coordinates[destination.name]
    return euclidean_distance(coord1, coord2)

# Execute A* search algorithm
source_vertex = graph.get_vertex("Sun")
destination_vertex = graph.get_vertex("Upsilon Andromedae")
a_star(graph, source_vertex, heuristic)

# Get the shortest path from Sun to Upsilon Andromedae
path = shortest_path(destination_vertex)
print("Shortest path from Sun to Upsilon Andromedae:", path)
