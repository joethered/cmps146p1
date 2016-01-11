from p1_support import load_level, show_level, save_level_costs
from math import inf, sqrt
from heapq import heappop, heappush




def dijkstras_shortest_path(initial_position, destination, graph, adj):
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.

    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        If a path exits, return a list containing all cells from initial_position to destination.
        Otherwise, return None.

    """
    
    Q = [(0, initial_position)]
    dist = {}
    prev = {}
    
    dist[initial_position] = 0
    prev[initial_position] = [initial_position]
    
    while Q:
        current_cost, current_node = heappop(Q)
        if current_node == destination:
            return prev[current_node]
        else:
            adjlst = adj(graph, current_node)
            for cost, node in adjlst:
                pathcost = cost + current_cost
                if node not in prev:
                    prev[node] = []
                    prev[node].extend(prev[current_node])
                    prev[node].append(node)
                    dist[node] = pathcost
                    heappush(Q, (pathcost, node))
                elif pathcost < dist[node]:
                    prev[node].extend(prev[current_node])
                    prev[node].append(node)
                    dist[node] = pathcost
                    
  
    
    return None

def dijkstras_path_costs(initial_position, destination, graph, adj):
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.

    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        If a path exits, return a list containing the cost of the paths to all cells from initial_position to destination.
        Otherwise, return None.

    """
    
    Q = [(0, initial_position)]
    dist = {}
    prev = {}
    
    dist[initial_position] = 0
    prev[initial_position] = [initial_position]
    
    while Q:
        current_cost, current_node = heappop(Q)
        if current_node == destination:
            return dist[current_node]
        else:
            adjlst = adj(graph, current_node)
            for cost, node in adjlst:
                pathcost = cost + current_cost
                if node not in prev:
                    prev[node] = []
                    prev[node].extend(prev[current_node])
                    prev[node].append(node)
                    dist[node] = pathcost
                    heappush(Q, (pathcost, node))
                elif pathcost < dist[node]:
                    prev[node].extend(prev[current_node])
                    prev[node].append(node)
                    dist[node] = pathcost
                    
  
    
    return None
            

def dijkstras_shortest_path_to_all(initial_position, graph, adj):
    """ Calculates the minimum cost to every reachable cell in a graph from the initial_position.

    Args:
        initial_position: The initial cell from which the path extends.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        A dictionary, mapping destination cells to the cost of a path from the initial_position.
    """
    paths = {}
    for dest in graph['spaces']:
        if dest != initial_position:
            paths[dest] = dijkstras_path_costs(initial_position, dest, graph, adj)
            
            
    return paths

def navigation_edges(level, cell):
    """ Provides a list of adjacent cells and their respective costs from the given cell.

    Args:
        level: A loaded level, containing walls, spaces, and waypoints.
        cell: A target location.

    Returns:
        A list of tuples containing an adjacent cell's coordinates and the cost of the edge joining it and the
        originating cell.

        E.g. from (0,0):
            [((0,1), 1),
             ((1,0), 1),
             ((1,1), 1.4142135623730951),
             ... ]
    """

    adjlst = []
    for i in range(cell[0]-1, cell[0]+2):
        for j in range(cell[1]-1, cell[1]+2):
            #print ("curr_node = " + str((i,j)))
            if (i,j) not in level['spaces'] or (i,j) == cell:
                #print ((i,j))
                continue
            curr_cell = (i,j)
            half_dist = distance(cell, curr_cell) * .5
    
                
            cost1 = level['spaces'][cell]
            cost2 = level['spaces'][curr_cell]
            final_cost = half_dist * (cost1 + cost2)
            heappush(adjlst, (final_cost, curr_cell))
    
    
    #[(0.0, (3, 2)), (1.0, (3, 1)), (1.5, (2, 2)), (2.121320343559643, (2, 1))]
    #print (adjlst)
    return adjlst
    

def distance (p1, p2):
        firstsum = (p2[1] - p1[1]) ** 2
        secsum = (p2[0] - p1[0]) ** 2
        total = firstsum + secsum
        return sqrt(total)
    
def test_route(filename, src_waypoint, dst_waypoint):
    """ Loads a level, searches for a path between the given waypoints, and displays the result.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        dst_waypoint: The character associated with the destination waypoint.

    """

    # Load and display the level.
    level = load_level(filename)
    show_level(level)

    # Retrieve the source and destination coordinates from the level.
    src = level['waypoints'][src_waypoint]
    dst = level['waypoints'][dst_waypoint]

    # Search for and display the path from src to dst.
    path = dijkstras_shortest_path(src, dst, level, navigation_edges)
    if path:
        show_level(level, path)
    else:
        print("No path possible!")


def cost_to_all_cells(filename, src_waypoint, output_filename):
    """ Loads a level, calculates the cost to all reachable cells from 
    src_waypoint, then saves the result in a csv file with name output_filename.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        output_filename: The filename for the output csv file.

    """
    
    # Load and display the level.
    level = load_level(filename)
    show_level(level)

    # Retrieve the source coordinates from the level.
    src = level['waypoints'][src_waypoint]
    
    # Calculate the cost to all reachable cells from src and save to a csv file.
    costs_to_all_cells = dijkstras_shortest_path_to_all(src, level, navigation_edges)
    save_level_costs(level, costs_to_all_cells, output_filename)


if __name__ == '__main__':
    filename, src_waypoint, dst_waypoint = 'my_maze.txt', 'a','c'

    # Use this function call to find the route between two waypoints.
    test_route(filename, src_waypoint, dst_waypoint)

    # Use this function to calculate the cost to all reachable cells from an origin point.
    cost_to_all_cells(filename, src_waypoint, 'my_maze_costs.csv')
