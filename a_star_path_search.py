# A* search for finding the shortest path in a map

# It uses a function f, where f = g + h. g is the distance covered so far and h is the estimated distance to the goal.
# The calculation is based on the Euclidean distance (it could also be based on other calculations as the Manhattan distance, but Euclidean distance is the most accurate here).
# The total cost is stored in a min priority queue, so the current element with the min cost can be stored and retrieved in log(n) time, each.
# When the min element is the goal, the goal check is successful. It returns the path, which is backtracked by links to predecessors -> This requires just constant O(1) additional storage. 
# A visited set keeps track of intersections explored so far. 
# Altogether space requirements are: O(n) for priority queue + O(n) for explored set = O(n)
# Altogether the runtime for shortest_path is: O(V) for frontier while loop * O(E) for checking outgoing roads from intersection (duplicate checks when updating elements)
# .. + O(V) for building backtracking path. This is: O(V) * O(E) + O(V) = O(E*V), where E are roads (edges) and V are intersections (vertices)

from math import sqrt
from queue import PriorityQueue

class Intersection:
    def __init__(self, number, predecessor = None, g_cost = 0, h_cost = 0):
        self.number = number # id of intersection
        self.predecessor = predecessor # link to predecessor for backtracking path
        self.g_cost = g_cost # distance covered so far
        self.h_cost = h_cost # distance to the goal

    def cost(self):
        return self.g_cost + self.h_cost

    # Comparison function for priority queue
    def __lt__(self, other):
        return self.g_cost + self.h_cost < other.g_cost + other.h_cost

def shortest_path(map, start, goal):
    frontier = PriorityQueue() #frontier is a priority queue
    explored = set() #marker for already explored
    #The unexplored part is discovered by examining neighbours on the frontier, no need to keep track of it in a separate data structure!
    
    #push start element on queue (as Intersection)
    frontier.put(Intersection(start))

    while frontier.qsize() > 0:
        #pop min element
        intersection = frontier.get()

        #if element = goal: terminate
        if intersection.number == goal:
            return terminate_and_build_path(intersection)

        explored.add(intersection)

        #check neighbors .. map.roads[xx]
        for road in map.roads[intersection.number]:
            
            #calc g and h cost (based on euclidean distance)
            g_cost = intersection.g_cost + path_cost(map, intersection.number, road)
            h_cost = estimated_distance_to_goal(map, road, goal)
            cost = g_cost + h_cost

            element_already_explored = False
            for element in explored:
                if element.number == road:
                    if cost < element.cost(): #already explored & cost lower: update frontier element
                        element.g_cost = g_cost
                        element.h_cost = h_cost
                        element.path.append(road)
                    #else: discard (cost higher)
                    element_already_explored = True
                    break
                
            if not element_already_explored:
                #if not already explored: put on frontier
                new_intersection = Intersection(road, intersection, g_cost, h_cost)
                frontier.put(new_intersection)

    return None

def path_cost(map, intersection, goal): #g function
    return euclidean_distance(map.intersections[goal][0], map.intersections[intersection][0], map.intersections[goal][1], map.intersections[intersection][1])

def estimated_distance_to_goal(map, intersection, goal): #h function
     return euclidean_distance(map.intersections[goal][0], map.intersections[intersection][0], map.intersections[goal][1], map.intersections[intersection][1])

def euclidean_distance(x1, y1, x2, y2):
    return sqrt((x1 - y1) ** 2 + (x2 - y2) ** 2)

def terminate_and_build_path(final_intersection):
    # build path from predecessors
    path = [final_intersection.number]
    current_intersection = final_intersection
    while current_intersection.predecessor != None:
        path.append(current_intersection.predecessor.number)
        current_intersection = current_intersection.predecessor

    reversed_path = []
    for reversed_intersection in reversed(path):
        reversed_path.append(reversed_intersection)

    return reversed_path
