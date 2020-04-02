import osmnx as ox
import networkx as nx
import folium
import pandas as pd
import geopandas as gdf
import heapq
import time
import json
import re
from collections import OrderedDict
from osmnx.downloader import get_from_cache, get_http_headers, get_pause_duration, save_to_cache
import logging as lg
import networkx as nx
import time
from itertools import groupby
from osmnx import settings
from osmnx.utils import make_str, log
from osmnx.geo_utils import get_largest_component
from osmnx.downloader import overpass_request
from osmnx.errors import *
import pprint
import requests
import heapq as hq
from math import sin, cos, sqrt, atan2, radians, acos

from collections import defaultdict


# documentationfor networkx: https://networkx.github.io/documentation/stable/reference/algorithms/shortest_paths.html


class Node:
    # node class to simplify pathfinding
    def __init__(self, number, parent):
        self.number = number
        self.parent = parent
        self.currentCost = 0  # also known as G cost
        self.heuCost = 0  # also known as H (heuristic)  - cost of travel from current node to end node
        self.fullCost = 0  # combination of G and H
        self.adjacency = []

    def add_neighbour(self, neighbour):
        # check if this node is equal to another node
        self.adjacency.append(neighbour)


class Graph():
    def __init__(self):
        """
        self.edges is a dict of all possible next nodes
        e.g. {'X': ['A', 'B', 'C', 'E'], ...}
        self.weights has all the weights between two nodes,
        with the two nodes as a tuple as the key
        e.g. {('X', 'A'): 7, ('X', 'B'): 2, ...}
        """
        self.edges = defaultdict(list)
        self.weights = {}

    def add_edge(self, from_node, to_node, weight):
        # Note: assumes edges are bi-directional
        self.edges[from_node].append(to_node)
        self.edges[to_node].append(from_node)
        self.weights[(from_node, to_node)] = weight
        self.weights[(to_node, from_node)] = weight


def dijsktra(graph, initial, end):
    result = []

    # shortest paths is a dict
    # each value is a tuple of (previous node, weight)
    shortest_paths = {initial: (None, 0)}  # dictionary of shortest path nodes
    current_node = initial
    visited = set()  # set is unordered. if a node is inside visited it won't be dealt with. yay

    while current_node != end:  # while destination has not been reached yet
        visited.add(current_node)  # prevents current node from being visited again
        destinations = graph.edges[current_node]  # this adds all neighbouring nodes into potential destinations
        weight_to_current_node = shortest_paths[current_node][1]  # 1 refers to node's weight

        # filter through potential destinations, pick the best shortest path (greedily)
        for next_node in destinations:  # for every possible next node
            next_weight = graph.weights[(current_node, next_node)] + weight_to_current_node
            # acquires total weight (in the event that we take this route)
            if next_node not in shortest_paths:
                #
                shortest_paths[next_node] = (current_node, next_weight)
            else:
                cost_so_far = shortest_paths[next_node][
                    1]  # set current shortest weight to be the current node-to-node path
                if next_weight < cost_so_far:
                    # next proposed route is faster than the current route
                    # update with proposed route weight to have the true shortest path
                    shortest_paths[next_node] = (current_node, next_weight)

            # at this point, shortest_paths[next_node] will now contain the best edge
            # for current_node to travel to

        next_destinations = {node: shortest_paths[node] for node in shortest_paths if node not in visited}
        # preps next wave of nodes based on the unvisited node in shortest_paths

        if not next_destinations:
            # no more nodes exist, i.e route is impossible
            # will likely not happen since we're in a small map
            return "Route Not Possible"
        # next node is the destination with the lowest weight
        current_node = min(next_destinations, key=lambda k: next_destinations[k][1])
        # iterates into the next closest node (min checks for closest, based on weight)

    # shortest path now acquired
    end_cost = 0
    path = []
    while current_node is not None:  # interate backwards nodes until end of road is reached
        path.append(current_node)
        next_node = shortest_paths[current_node][0]
        if (next_node is not None):
            end_cost += graph.weights[(current_node, next_node)]
        current_node = next_node
    # Reverse path
    path = path[::-1]
    result.append(path)
    result.append(end_cost)
    return result


def get_lrt_route(graph, initial, dest, lrt_graph, lrt_exits):
    """
    :param graph: NetworkX multigraph for walking.
    :param initial: Starting node in X Y tuple. Pretty much is always punggol mrt.
    :param dest: Destination node in X Y tuple.
    :param lrt_graph: NetworkX multigraph consisting of only LRT nodes and edges.
    :return: returns finalRoues[] which consists of multiple (just 2) routes in an array
    """

    """
    essentially a redo of lrt movement, but with bus
    by right supposed to check for bus if node is far enough, but most nodes aren't far enough lol
    """

    # CURRENT LAYER: foot
    # now, we are at Punggol MRT.
    # We want to look for the closest LRT to the destination.

    endLRT = 0
    endLRTXY = 0
    lowest_dist = 5000  # dummy value.

    # loop through the LRT map and find the node that is closest to the destination with distance()
    for stn, d in lrt_graph.nodes(data=True):
        if 'ref' in d:
            # look at only valid stops - i.e nodes with a 'ref'
            stnXY = (lrt_graph.nodes[stn]['x'], lrt_graph.nodes[stn]['y'])
            print("|TEST| - Right now at Station ", stn, " coordinates: ", stnXY)
            curr_dist = distance(stnXY, dest)

            if curr_dist < lowest_dist:
                endLRT = stn
                endLRTXY = stnXY
                lowest_dist = curr_dist

    # CURRENT LAYER: lrt stations (real)
    # we know what LRT we want to go to
    # but real lrt stations are not actually connected together (in the graph)
    # so instead, we go to the closest connected node to it

    print("|TEST| - Closest real station to destination: ", endLRT, "at ", endLRTXY)
    closestEndLrt = ox.get_nearest_edge(lrt_graph, (endLRTXY[1], endLRTXY[0]))
    print("|TEST| - Connected ending node is : ", closestEndLrt[1])  # this is the node that's actually in the network
    # if using 103.9064504, 1.3988214 as end node, this should be Cove LRT
    # if using 103.9164448, 1.399601, this should be Kadaloor LRT
    wayID = 0

    # get id of the LRT route we are in
    for (u, v, d) in lrt_graph.edges(data=True):
        if u == closestEndLrt[1]:
            wayID = d['osmid']
            break

    print("|TEST| - OSM-ID of the proper LRT route is : ", wayID)

    # way ID of the LRT route we want to take has been acquired.

    # CURRENT LAYER: lrt stations
    # we now have a good connected dest. we now want to travel to that node.
    # but we need to figure out which connected node to start from (in Punggol)
    # that node must be in the same graph as the dest node

    closestStartLrt = 0
    lowest_dist = 5000

    for (u, v, d) in lrt_graph.edges(data=True):
        if d['osmid'] == wayID:
            # get XY of the node in the lrt route
            stnXY = (lrt_graph.nodes[u]['x'], lrt_graph.nodes[u]['y'])
            curr_dist = distance(stnXY, initial)

            if curr_dist < lowest_dist:
                closestStartLrt = u
                # closestStartLrtXY = stnXY
                lowest_dist = curr_dist

    print("|TEST| - Connected starting node is: ", closestStartLrt)

    # traverse to that node from startLRT to endLRT
    lrt_g = Graph()

    for i in lrt_graph.edges.data():
        length = i[2].get('length')
        lrt_g.add_edge(*(i[0], i[1], length))  # node1, node2, length

    # use dijkstra to acquire the best
    # graph is simple so dijkstra is not immediately the most useful
    lrtRoute = dijsktra(lrt_g, closestStartLrt, closestEndLrt[1])[0]

    # time complexity of dijkstra is O(n^2), where n is the amount of nodes.
    # the amount of LRT stations in punggol is negligible, so performance isn't a big issue.

    # to do - figure out how to make sure correct LRT route is taken for dijkstra
    print("|TEST| - Now taking LRT from ", closestStartLrt, " to ", closestEndLrt[1])

    print("|TEST| - LRT route: ", lrtRoute)
    lrtRouteYX = getRouteInYX(lrt_graph, lrtRoute)
    print("|TEST| - LRT route in XY (returns this): ", lrtRouteYX)

    # CURRENT LAYER: lrt stations
    # user has now ARRIVED at the lrt node.
    # we want to now bring the user to the closest foot path

    # most LRTs have a 'bridge' that connects to ground level via stairs/bridges
    # greedily find the closest bridge-end/stairs to the LRT
    # if distance() shows that the bridge is too far, (e.g Sam Kee station has no bridges)
    #   it's not considered to be an immediately linked bridge.
    # but if it is,
    #   then add XY to the paths array.

    exit = ox.get_nearest_node(lrt_exits, (endLRTXY[1], endLRTXY[0]))
    exitXY = (lrt_exits.nodes[exit]['x'], lrt_exits.nodes[exit]['y'])

    # after this, the user will be ready to start walking towards their destination

    if distance(exitXY, (endLRTXY)) > 100:
        # too far. just use current XY as last stop.
        print("|TEST| - Exit of ", exitXY, " is too far ,", distance(exitXY, (endLRTXY)), " disregarding exit.")
        return lrtRouteYX
    else:
        # use the exit as the last XY for routing purposes. flips exitXY to match the program.
        lrtRouteYX.append((exitXY[1], exitXY[0]))
        print("|TEST| - Adding X,Y of exit ", (exitXY[1], exitXY[0]), " to LRT route.")

    return lrtRouteYX

def lrt_bus_walk(graph, initial, dest, lrt_graph):

    # DISCLAIMER: function deprecated, leaving here for history's sake

    """
    :param graph: NetworkX multigraph for walking.
    :param initial: Starting node in X Y tuple. Pretty much is always punggol mrt.
    :param dest: Destination node in X Y tuple.
    :param lrt_graph: NetworkX multigraph consisting of only LRT nodes and edges.
    :return: returns finalRoues[] which consists of multiple (just 2) routes in an array
    """

    """
    essentially a redo of lrt movement, but with bus
    adds a bus route should the an additional taking of bus from the LRT is required
    movement pseudo:
    Start > LRT_start > LRT_end
    if distance to destination > some_distance
        LRT_end > bus_start > bus_end > walk to destination
    else
        LRT_end > walk to destination
    """

    finalRoutes = []  # will return this.

    # CURRENT LAYER: foot
    # now, we are at Punggol MRT.
    # We want to look for the closest LRT to the destination.

    endLRT = 0
    endLRTXY = 0
    lowest_dist = 5000  # dummy value.

    # loop through the LRT map and find the node that is closest to the destination with distance()
    for stn, d in lrt_graph.nodes(data=True):
        if 'ref' in d:
            # look at only valid stops - i.e nodes with a 'ref'
            stnXY = (lrt_graph.nodes[stn]['x'], lrt_graph.nodes[stn]['y'])
            print("|TEST| - Right now at Station ", stn, " coordinates: ", stnXY)
            curr_dist = distance(stnXY, dest)

            if curr_dist < lowest_dist:
                endLRT = stn
                endLRTXY = stnXY
                lowest_dist = curr_dist

    # CURRENT LAYER: lrt stations (real)
    # we know what LRT we want to go to
    # but real lrt stations are not actually connected together (in the graph)
    # so instead, we go to the closest connected node to it

    print("|TEST| - Closest real station to destination: ", endLRT, "at ", endLRTXY)
    closestEndLrt = ox.get_nearest_edge(lrt_graph, (endLRTXY[1], endLRTXY[0]))
    print("|TEST| - Connected ending node is : ", closestEndLrt[1])  # this is the node that's actually in the network
    # if using 103.9064504, 1.3988214 as end node, this should be Cove LRT
    # if using 103.9164448, 1.399601, this should be Kadaloor LRT
    wayID = 0

    # get id of the LRT route we are in
    for (u, v, d) in lrt_graph.edges(data=True):
        if u == closestEndLrt[1]:
            wayID = d['osmid']
            break

    print("|TEST| - OSM-ID of the proper LRT route is : ", wayID)

    # way ID of the LRT route we want to take has been acquired.

    # CURRENT LAYER: lrt stations (fake)
    # we now have a good connected dest. we now want to travel to that node.
    # but we need to figure out which connected node to start from (in Punggol)
    # that node must be in the same graph as the dest node
    # Punggol station is NOT a connected node. there are multiple nodes in Punggol station itself

    closestStartLrt = 0
    lowest_dist = 5000

    for (u, v, d) in lrt_graph.edges(data=True):
        if d['osmid'] == wayID:
            # get XY of the node in the lrt route
            stnXY = (lrt_graph.nodes[u]['x'], lrt_graph.nodes[u]['y'])
            curr_dist = distance(stnXY, initial)

            if curr_dist < lowest_dist:
                closestStartLrt = u
                # closestStartLrtXY = stnXY
                lowest_dist = curr_dist

    print("|TEST| - Connected starting node is: ", closestStartLrt)

    # traverse to that node from startLRT to endLRT
    lrt_g = Graph()

    for i in lrt_graph.edges.data():
        length = i[2].get('length')
        lrt_g.add_edge(*(i[0], i[1], length))  # node1, node2, length

    lrtRoute = dijsktra(lrt_g, closestStartLrt, closestEndLrt[1])[0]

    # to do - figure out how to make sure correct LRT route is taken for dijkstra
    print("|TEST| - Now taking LRT from ", closestStartLrt, " to ", closestEndLrt[1])

    print("|TEST| - LRT route: ", lrtRoute)
    lrtRouteYX = getRouteInYX(lrt_graph, lrtRoute)
    print("|TEST| - LRT route in YX (returns this): ", lrtRouteYX)

    finalRoutes.append(lrtRouteYX)

    # CURRENT LAYER: lrt stations (fake)
    # user has now ARRIVED at the connected lrt node.
    # we still have the real LRT destination node in endLRT (and endLRTXY for coord)
    # pop the user to the closest foot node.
    # start dijkstra from that foot node.

    # but if destination is still a good distance away (3200m, 400m, etc)
    # have the user take a bus instead
    if (distance(endLRTXY, dest) > 100):  # if dest is far enough
        # endLRTXY = (endLRTXY[1], endLRTXY[0])
        busRoute = []
        #busRoute = walk_bus_algor(endLRTXY, dest)  # DEPRECATED, no longer used, this code exists in walk_bus.py
        print("Bus route: ", busRoute)
        for x in busRoute:
            finalRoutes.append(x)
        return finalRoutes
    else:
        walkingStart = ox.get_nearest_node(graph, (endLRTXY[1], endLRTXY[0]))
        print("|TEST| - Now walking from: ", walkingStart, "(LRT) to destination ", dest)

        cGraph = Graph()

        for i in graph.edges.data():
            length = i[2].get('length')
            cGraph.add_edge(*(i[0], i[1], length))

        destID = ox.get_nearest_node(graph, (dest[1], dest[0]))

        walkingRoute = dijsktra(cGraph, walkingStart, destID)[0]
        print("|TEST| - Walking route: ", walkingRoute)
        print("|TEST| - End: ", destID, dest)

        finalRoutes.append(walkingRoute)
        return finalRoutes


def distance(source, target):
    """
    estimates the distance between two nodes
    nodes must be in the x,y format
    returns distance in meters(?)
    """
    R = 6371.0
    snlat = radians(source[1])
    snlon = radians(source[0])
    elat = radians(target[1])
    elon = radians(target[0])
    actual_dist = 6371.01 * acos(sin(snlat) * sin(elat) + cos(snlat) * cos(elat) * cos(snlon - elon))
    actual_dist = actual_dist * 1000

    return actual_dist


def getRouteInXY(G, route):
    # G = multigraph
    # route = node IDs in an array
    # returns an array of tuples, indicating X-Y coordinates of each node in the given array
    routeXY = []
    for id in route:
        routeXY.append((G.nodes[id]['x'], G.nodes[id]['y']))

    return routeXY

def getRouteInYX(G, route):
    # G = multigraph
    # route = node IDs in an array
    # returns an array of tuples, indicating Y-X coordinates of each node in the given array
    routeYX = []
    for id in route:
        routeYX.append((G.nodes[id]['y'], G.nodes[id]['x']))

    return routeYX

"""
dist = 1500
punggol = (1.4053828, 103.9022239)  # punggol MRT station
pgmap = folium.Map(height='100%', width='100%', location=punggol, tiles='OpenStreetMap', zoom_start=16)

# =========================================================
# PLACE IN INIT
# =========================================================

punggol = (1.4053828, 103.9022239)  # punggol MRT station, change according to whatever coordinates you are using
G = ox.graph_from_point(punggol, distance=dist, truncate_by_edge=True, network_type="walk",
                        infrastructure='way["highway"]')
G = ox.remove_isolated_nodes(G)

lrt_east = ox.graph_from_file(filename="data\lrt_pg_east.osm",
                              retain_all="true")  # retain all is impt. I don't know why exactly.
lrt_west = ox.graph_from_file(filename="data\lrt_pg_west.osm",
                              retain_all="true")  # retain all is impt. I don't know why exactly.
lrt_stations = nx.compose(lrt_east, lrt_west)

lrt_exits = ox.graph_from_file(filename="data\lrt_bridges.osm",
                              retain_all="true") 

"""



# =========================================================
# END OF "PLACE IN INIT" requirements
# =========================================================

# now, time to make sure primary graph has these nodes as coordinates.

# prints all node IDs that exists in main node graph

#n, e = ox.graph_to_gdfs(G)
#lrt_n, lrt_e = ox.graph_to_gdfs(lrt_stations)

# =========================================================
# TESTING NODES, REPLACE WITH REAL USER INPUT
# =========================================================

#node1 = (103.9022239, 1.4053828)
#node2 = (103.91602993011, 1.4030734871395)

# =========================================================
# END OF "TESTING NODES"
# =========================================================

# =========================================================
# ROUTE FUNCTION CALLING, this returns arrays of XY coordinates
# =========================================================

#routes = get_lrt_route(G, node1, node2, lrt_stations)
# routes = lrt_bus_walk(G, node1, node2, lrt_stations)

#lrtLast = routes[0][-1]
#routes[1].insert(0, lrtLast)

# print("PRINTING X,Y OF ROUTE", routes)
# print("lrt combined routes: ", routes)