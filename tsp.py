"""
Contains the implementation of approximation algorithm to the
Travelling Salesman Problem by Christofides.
"""

import random


class UnionFind:
    """
    Class containing implementation of the union find algorithm used in Kruskal's algorithm.
    """

    def __init__(self):
        self.weights = {}
        self.parents = {}

    def __getitem__(self, object):
        if object not in self.parents:
            self.parents[object] = object
            self.weights[object] = 1
            return object

        # Find path of objects leading to the root
        path = [object]
        root = self.parents[object]

        while root != path[-1]:
            path.append(root)
            root = self.parents[root]

        # Compress the path and return
        for ancestor in path:
            self.parents[ancestor] = root

        return root

    def __iter__(self):
        return iter(self.parents)

    def union(self, *objects):
        roots = [self[x] for x in objects]
        heaviest = max([(self.weights[r], r) for r in roots])[1]

        for r in roots:
            if r != heaviest:
                self.weights[heaviest] += self.weights[r]
                self.parents[r] = heaviest


def minimum_spanning_tree(G):
    """
    Finds the minimum spanning tree of a given graph using Kruskal's algorithm.

    Args:
    * G: The graph of the cluster nodes

    Returns:
    * tree: The MST of the graph
    """

    tree = []
    subtrees = UnionFind()

    for W, u, v in sorted((G[u][v], u, v) for u in G for v in G[u]):
        if subtrees[u] != subtrees[v]:
            tree.append((u, v, W))
            subtrees.union(u, v)

    return tree


def find_odd_vertexes(MST):
    """
    Finds the vertexes with odd degree in a given MST.

    Args:
    * MST: The minimum spanning tree of the cluster graph

    Returns:
    * vertexes: The list of vertexes with odd degree in the MST
    """

    tmp_g = {}
    vertexes = []

    for edge in MST:
        if edge[0] not in tmp_g:
            tmp_g[edge[0]] = 0

        if edge[1] not in tmp_g:
            tmp_g[edge[1]] = 0

        tmp_g[edge[0]] += 1
        tmp_g[edge[1]] += 1

    for vertex in tmp_g:
        if tmp_g[vertex] % 2 == 1:
            vertexes.append(vertex)

    return vertexes


def minimum_weight_matching(MST, G, odd_vert):
    """
    Finds the minimum-cost perfect matching in a list of vertexes.

    Args:
    * MST: The minimum spanning tree of the cluster graph
    * G: The map representing the subgraph of the cluster whose route needs to be found
    * odd_vert: The list of odd-degree vertexes found in the MST

    Returns:
    null (modifies odd_vert to produce the matching)
    """

    random.shuffle(odd_vert)

    while odd_vert:
        v = odd_vert.pop()
        length = float("inf")
        u = 1
        closest = 0

        for u in odd_vert:
            if v != u and G[v][u] < length:
                length = G[v][u]
                closest = u

        MST.append((v, closest, length))
        odd_vert.remove(closest)


def remove_edge_from_matchedMST(MatchedMST, v1, v2):
    """
    Removes certain edge between 2 nodes in the matched MST.

    Args:
    * MatchedMST: The minimum-cost perfectly matched MST obtained
    * v1: The first node
    * v2: The second node

    Returns:
    * MatchedMST: The supplied MST with the edge removed
    """

    for i, item in enumerate(MatchedMST):
        if (item[0] == v2 and item[1] == v1) or (item[0] == v1 and item[1] == v2):
            del MatchedMST[i]

    return MatchedMST


def find_eulerian_tour(MatchedMSTree, G):
    """
    Finds an Eulerian tour of the matched MST obtained.

    Args:
    * MatchedMSTree: The minimum-cost perfectly matched MST obtained
    * G: The map representing the subgraph of the cluster whose route needs to be found

    Returns:
    * EP: The Eulerian tour of the matched MST
    """

    # Find neigbours
    neighbours = {}

    for edge in MatchedMSTree:
        if edge[0] not in neighbours:
            neighbours[edge[0]] = []

        if edge[1] not in neighbours:
            neighbours[edge[1]] = []

        neighbours[edge[0]].append(edge[1])
        neighbours[edge[1]].append(edge[0])

    # Finds the Hamiltonian circuit
    start_vertex = MatchedMSTree[0][0]
    EP = [neighbours[start_vertex][0]]

    while len(MatchedMSTree) > 0:
        for i, v in enumerate(EP):
            if len(neighbours[v]) > 0:
                break

        while len(neighbours[v]) > 0:
            w = neighbours[v][0]
            remove_edge_from_matchedMST(MatchedMSTree, v, w)
            del neighbours[v][(neighbours[v].index(w))]
            del neighbours[w][(neighbours[w].index(v))]

            i += 1
            EP.insert(i, w)
            v = w

    return EP


def tsp(G):
    """
    Serves as a caller function of the methods implementing the steps of the Christofides algorithm.

    Args:
    * G: The map representing the subgraph of the cluster whose route needs to be found

    Returns:
    * shifted_path: The minimum-cost route for the robot to touch each node once and return to node 0
    * length: The cost of the route found
    """

    # Build a minimum spanning tree
    MSTree = minimum_spanning_tree(G)

    # Find odd vertexes
    odd_vertexes = find_odd_vertexes(MSTree)

    # Add minimum weight matching edges to MST
    minimum_weight_matching(MSTree, G, odd_vertexes)

    # Find an Eulerian tour
    eulerian_tour = find_eulerian_tour(MSTree, G)

    current = eulerian_tour[0]
    path = [current]
    visited = [False] * len(eulerian_tour)
    visited[eulerian_tour[0]] = True
    length = 0

    # Remove repeated nodes
    for v in eulerian_tour:
        if not visited[v]:
            path.append(v)
            visited[v] = True
            length += G[current][v]
            current = v

    length += G[current][eulerian_tour[0]]
    path.append(eulerian_tour[0])

    # Cycle shift the path to start from 0
    index = path.index(0)
    shifted_path = []

    if index != 0:
        path.pop()
        for i in range(0, len(path)):
            shifted_path.append(path[(i + index) % len(path)])
        shifted_path.append(0)
    else:
        shifted_path = path

    return shifted_path, length
