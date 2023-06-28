#!/usr/bin/env pypy3

from queue import PriorityQueue
from fractions import Fraction

class Node:
    """
    Class for creating each Node in a graph

    Attributes:
        dist(int): the distance the battery life lasts for
        speed(int): the speed that the battery can move a car at
        neighbors(dict): neighboring Nodes as keys, distance between them as
            values
        name(int): the station number assigned to a Node object
        time(float): fastest time it takes to get to this Node from the start
        predecessor(Node or None): Node preceding it on the fastest route

        reach_dist(float): used to calculate the Nodes that it can reach
        neighbor_times(dict): neighboring nodes as keys, fastest time to reach
            neighbor as values
        reachable(set): all the Nodes that it can reach
    """

    def __init__(self, dist, speed, name):
        """
        Initializing the Node object
        """
        self.dist = dist
        self.speed = speed
        self.neighbors = {}
        self.name = name
        self.time = float('inf')
        self.predecessor = None

        #Use this to find which nodes are reachable or not
        self.reach_dist = float('inf')
        self.neighbor_times = {}
        self.reachable = set()


nodes = {}

def make_graph(batteries, edges):
    """
    Makes the graph with edge weights equal to time

    Inputs:
        batteries(lst of lst of ints): the battery that can move a car up to
            distance c_i with speed s_i
        edges(lst of lst of ints): two nodes and the time it takes to get
            between them

    Returns(dict): keys are the station numbers and values are Node objects
    """
    for i, battery in enumerate(batteries, 1):
        dist, speed = battery
        nodes[i] = Node(dist, speed, i)
    for edge in edges:
        u, v, d = edge
        dist = nodes[u].dist
        nodes[u].neighbors[nodes[v]] = d
    return nodes


def reachable():
    """
    Setting node.reachable for every node in the graph
    """
    for node in nodes.values():
        explore(node)


def explore(node):
    """
    Checks which nodes a single node can reach and adds it to node.reachable

    Inputs:
        node(Node): the single Node to check for
    """
    queue = PriorityQueue()
    node.reach_dist = 0
    queue.put((0, node.name))
    while not queue.empty():
        curr = queue.get()[1]
        curr = nodes[curr]
        for n, d in curr.neighbors.items():
            if n.reach_dist >= d + curr.reach_dist:
                n.reach_dist = d + curr.reach_dist
                if node.dist >= n.reach_dist:
                    node.reachable.add((n, n.reach_dist))
                    queue.put((n.reach_dist, n.name))
    for node in nodes.values():
        node.reach_dist = float('inf')


def set_time():
    """
    Setting edge weights in the graph equal to time
    """

    #Make a new graph using the reachable list
    for node in nodes.values():
        for neighbor in node.reachable:
            n, dist = neighbor
            node.neighbor_times[n] = Fraction(dist, node.speed)


def dijkstra(start):
    """
    Running Dijkstra's to find the shortest path from the start Node

    start(Node): the Node to start at
    """
    start.time = 0
    queue = PriorityQueue()
    queue.put((0, start.name))
    while not queue.empty():
        curr = queue.get()[1]
        curr = nodes[curr]
        for n, time in curr.neighbor_times.items():
            if n.time > curr.time + time:
                n.time = curr.time + time
                n.predecessor = curr
                queue.put((n.time, n.name))


def solve(batteries, dists):
    """
    Solves the problem

    Inputs:
        batteries(lst of lst of ints): the battery that can move a car up to
            distance c_i with speed s_i
        dists(lst of lst of ints): list of distances between each station

    Returns (float): the shortest time it takes to get to the end Node
    """
    nodes_dict = make_graph(batteries, dists)
    nodes = list(nodes_dict.values())

    #Adds all the reachable nodes to each node
    reachable()

    start, end = nodes[0], nodes[-1]

    #Setting edges as time
    set_time()

    #Running Dijkstra's on this time graph
    dijkstra(start)

    return float(end.time)


def read_input():
    """
    Reads the input from a .txt file
    """
    N, M = [int(i) for i in input().split()]
    batteries = [[int(i) for i in input().split()] for _ in range(N)]
    dists = [[int(i) for i in input().split()] for _ in range(M)]
    return batteries, dists

def main():
    batteries, dists = read_input()
    t = solve(batteries, dists)
    print(f'{t:.6f}')

if __name__ == '__main__':
    main()
