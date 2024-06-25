import tsp
import sys
from typing import List


def single_node_cycle(non_cluster: list, graph: List[List[int]]):
    """
    Computes the sum of single node cycle distance from each node in the non_cluster to the depot.

    Args:
    * non_cluster (list): The nodes outside the cluster
    * graph (list[list]): A graph of edges between tasks/nodes

    Returns:
    * single_node_cycle_distance: The total sum of single node cycle distances for non_cluster
    """

    single_node_cycle_distance = 0

    for node in non_cluster:
        single_node_cycle_distance += 2 * graph[0][node]

    return single_node_cycle_distance


def euler_cycle(cluster: list, graph: List[List[int]]):
    """
    Calculates the sum of the distance between each consecutive node in the cluster.

    Args:
    * cluster (list): The nodes in the cluster
    * graph (list[list]): A graph of edges between tasks/nodes

    Returns:
    * euler_cycle_distance: The total sum of distances for cluster
    """

    euler_cycle_distance = 0

    for i in range(len(cluster) - 1):
        euler_cycle_distance += graph[cluster[i]][cluster[i + 1]]

    euler_cycle_distance += graph[cluster[-1]][cluster[0]]

    return euler_cycle_distance


def restricted_nearest_neighbour(
    active_node: int,
    non_cluster: list,
    cluster_weight: int,
    capacity: int,
    graph: List[List[int]],
    weight: List[int],
):
    """
    Computes the nearest neighbour of the supplied node in terms of
    Euclidean distance outside the cluster that the node is in.

    Args:
    * active_node (int): The index of the node whose neighbour we want to find
    * non_cluster (list): The nodes outside the cluster
    * cluster_weight (int): The current_weight of the cluster
    * capacity (int): The maximum weight a robot can carry
    * graph (list[list]): A graph of edges between tasks/nodes
    * weight (list): The weights of each task/node

    Returns:
    * closest_node (int): The nearest neighbour that can fit in the cluster
    """

    closest_distance = None
    closest_node = 0

    for node in non_cluster:
        # If the addition of the node exceeds capacity, we do not consider it
        if cluster_weight + weight[node] <= capacity:
            distance = graph[active_node][node]
            if closest_distance is None or distance < closest_distance:
                closest_distance = distance
                closest_node = node

    return closest_node


def feasible_route(
    unassigned_nodes: List[int],
    capacity: int,
    graph: List[List[int]],
    weight: List[int],
):
    """
    Computes a cluster of nodes to be assigned to a robot from a
    set of unassigned nodes.

    Args:
    * unassigned_nodes (list): The set of unassigned nodes from which to choose
    * capacity (int): The maximum weight a robot can carry
    * graph (list[list]): A graph of edges between tasks/nodes
    * weight (list): The weights of each task/node

    Returns:
    * assigned (list): The cluster of nodes assigned to the robot
    * remaining (list): The nodes not included in the cluster
    """

    p_min = single_node_cycle(unassigned_nodes, graph) + 1
    assigned = []
    remaining = []

    for current_node in unassigned_nodes:
        cluster = [0, current_node]
        non_cluster = unassigned_nodes.copy()
        non_cluster.remove(current_node)
        cluster_weight = weight[current_node]
        active_node = current_node

        while cluster_weight < capacity:
            # Finding the nearest neighbour to current node (in Euclidean distance)
            k = restricted_nearest_neighbour(
                active_node, non_cluster, cluster_weight, capacity, graph, weight
            )

            if k == 0:
                break

            cluster_weight += weight[k]
            # Add node to potential cluster
            cluster.append(k)
            non_cluster.remove(k)
            active_node = k

        p = euler_cycle(cluster, graph) + single_node_cycle(non_cluster, graph)

        if p < p_min:
            p_min = p
            assigned = cluster
            remaining = non_cluster

    return assigned, remaining


def travelling_salesman(assigned_nodes: List[int], graph: List[List[int]]):
    """
    Given a set of nodes, compute the best path for the robot to take
    such that it only passes through each of them once.

    Uses the approximation algorithm for TSP by Christofides (an approximation
    ratio is 1.5), assuming starting node as 0.

    Args:
    * assigned_nodes (list): The nodes assigned to the robot
    * graph (list[list]): A graph of edges between tasks/nodes

    Returns:
    * cost (int): The cost of travelling along this route
    * route (list): The computed sequence of nodes
    """

    # Create a map of assigned nodes from 0 to len(assigned_nodes)
    print("Assigned nodes: ", assigned_nodes)
    assigned_nodes_map = {}

    for i in range(len(assigned_nodes)):
        assigned_nodes_map[assigned_nodes[i]] = i

    # Create a new graph with only the assigned nodes
    new_graph = {}

    for i in range(len(assigned_nodes)):
        for j in range(len(assigned_nodes)):
            if i != j:
                if i not in new_graph:
                    new_graph[i] = {}

                new_graph[i][j] = graph[assigned_nodes[i]][assigned_nodes[j]]

    route, cost = tsp.tsp(new_graph)

    for i in range(len(route)):
        route[i] = assigned_nodes[route[i]]

    return route, cost


def nCAR(capacity: int, graph: List[List[int]], weight: List[int]):
    """
    Allocates optimal paths to complete a set of tasks/nodes given
    a set weighted nodes for a group of robots.

    Args:
    * capacity (int): The maximum weight a robot can carry
    * graph (list[list]): A graph of edges between tasks/nodes
    * weight (list): The weights of each task/node

    Returns:
    * routes (list[list]): A sequence of nodes for each robot
    * robot_count (int): The minimum number of robots required
    * total_cost (int): The total cost to travel all routes
    """

    for node in range(len(graph)):
        if weight[node] > capacity:
            print("Error: Weight of node ", node, " exceeds capacity")
            sys.exit(0)

    routes = []
    robot_count = 0
    total_cost = 0

    # The set of nodes not assigned to any robot in each iteration
    unassigned_nodes = list(range(1, len(graph)))

    while len(unassigned_nodes) > 0:
        # Computing a cluster of nodes to be assigned to a robot
        assigned, remaining = feasible_route(unassigned_nodes, capacity, graph, weight)
        # Given the cluster, we compute the best path for the robot
        route, cost = travelling_salesman(assigned, graph)

        routes.append(route)
        total_cost += cost
        robot_count += 1

        unassigned_nodes = remaining
        print("Unassigned nodes: ", unassigned_nodes)

    return routes, robot_count, total_cost


if __name__ == "__main__":
    # The set of nodes/tasks. This would be a graph
    graph = []
    # The weights of each task/node
    weight = []
    # The capacity of each robot
    capacity = 0

    """
    Input format: File - input.txt
    First line: Number of objects 
    Second line: Capacity of each robot
    Third line: Weight of each object (n integers)
    Next n + 1 lines: Distance between each pair of objects (n+1 lines, each line with n + 1 integers)
    """
    try:
        with open("input.txt") as f:
            n = int(f.readline())
            capacity = int(f.readline())
            weight = [int(x) for x in f.readline().split()]
            for i in range(n + 1):
                graph.append([int(x) for x in f.readline().split()])
    except:
        print("Error reading file")
        sys.exit(0)

    # Print all the input
    print("Number of objects: ", n)
    print("Capacity of each robot: ", capacity)
    print("Weight of each object: ", weight)
    print("Distance between each pair of objects: ", graph)

    # Preappend 0 to weight
    weight.insert(0, 0)
    routes, robot_count, total_cost = nCAR(capacity, graph, weight)

    print("Total cost: ", total_cost)
    print("Number of robots: ", robot_count)
    print("Routes: ", routes)
