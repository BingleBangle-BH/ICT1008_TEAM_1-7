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


def lrt_walk(graph, initial, dest, lrt_graph):
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

    walkingStart = ox.get_nearest_node(graph, (endLRTXY[1], endLRTXY[0]))
    print("|TEST| - Now walking from: ", walkingStart, "(LRT) to destination ", dest)

    lrtLastNode = lrtRoute[-1]
    cGraph = Graph()

    for i in graph.edges.data():
        length = i[2].get('length')
        cGraph.add_edge(*(i[0], i[1], length))

    destID = ox.get_nearest_node(graph, (dest[1], dest[0]))

    walkingRoute = dijsktra(cGraph, walkingStart, destID)[0]
    print("|TEST| - Walking route: ", walkingRoute)
    print("|TEST| - End: ", destID, dest)

    walkingRoute = getRouteInYX(G, walkingRoute)
    finalRoutes.append(walkingRoute)
    return finalRoutes


def lrt_bus_walk(graph, initial, dest, lrt_graph):
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
        busRoute = walk_bus_algor(endLRTXY, dest)  # replace coordinates with endXY
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


def walk_bus_algor(start, end):
    """
        It generates a three different routes
        1) Walking from start point to bus stop (A* algorithm)
        2) Bus service from start bus stop to end bus stop (Greedy algorithm)
        3) Walking from bus stop to end point (A* algorithm)
        Parameters
        ----------
        start : node id
        end : node id
        Returns
        -------
        list[start route,bus route,end route]
    """

    # ---CLASSES---#
    class my_dictionary(dict):
        """
        Creates a dictionary
        """

        def __init__(self):
            self = dict()

        def add(self, key, value):
            self[key] = value

    # ---FUNCTIONS---#
    def bus_layer(start, end, results, case):
        """
        It generates a bus route with the bus numbers via greedy algorithm
        Parameters
        ----------
        start : node id
        end : node id
        results : dict (From lta datamall)
        case : int
        Returns
        -------
        final_route_list : list
        """

        def overpass_request(data, pause_duration=None, timeout=180, error_pause_duration=None):
            """
            Send a request to the Overpass API via HTTP POST and return the JSON
            response.
            Parameters
            ----------
            data : dict or OrderedDict
                key-value pairs of parameters to post to the API
            pause_duration : int
                how long to pause in seconds before requests, if None, will query API
                status endpoint to find when next slot is available
            timeout : int
                the timeout interval for the requests library
            error_pause_duration : int
                how long to pause in seconds before re-trying requests if error
            Returns
            -------
            dict
            """

            # define the Overpass API URL, then construct a GET-style URL as a string to
            # hash to look up/save to cache
            url = settings.overpass_endpoint.rstrip('/') + '/interpreter'
            prepared_url = requests.Request('GET', url, params=data).prepare().url
            cached_response_json = get_from_cache(prepared_url)

            if cached_response_json is not None:
                # found this request in the cache, just return it instead of making a
                # new HTTP call
                return cached_response_json

            else:
                # if this URL is not already in the cache, pause, then request it
                if pause_duration is None:
                    this_pause_duration = get_pause_duration()
                log('Pausing {:,.2f} seconds before making API POST request'.format(this_pause_duration))
                time.sleep(this_pause_duration)
                start_time = time.time()
                log('Posting to {} with timeout={}, "{}"'.format(url, timeout, data))
                response = requests.post(url, data=data, timeout=timeout, headers=get_http_headers())

                # get the response size and the domain, log result
                size_kb = len(response.content) / 1000.
                domain = re.findall(r'(?s)//(.*?)/', url)[0]
                log('Downloaded {:,.1f}KB from {} in {:,.2f} seconds'.format(size_kb, domain, time.time() - start_time))

                try:
                    response_json = response.json()
                    if 'remark' in response_json:
                        log('Server remark: "{}"'.format(response_json['remark'], level=lg.WARNING))
                    save_to_cache(prepared_url, response_json)
                except Exception:
                    # 429 is 'too many requests' and 504 is 'gateway timeout' from server
                    # overload - handle these errors by recursively calling
                    # overpass_request until we get a valid response
                    if response.status_code in [429, 504]:
                        # pause for error_pause_duration seconds before re-trying request
                        if error_pause_duration is None:
                            error_pause_duration = get_pause_duration()
                        log(
                            'Server at {} returned status code {} and no JSON data. Re-trying request in {:.2f} seconds.'.format(
                                domain,
                                response.status_code,
                                error_pause_duration),
                            level=lg.WARNING)
                        time.sleep(error_pause_duration)
                        response_json = overpass_request(data=data, pause_duration=pause_duration, timeout=timeout)

                    # else, this was an unhandled status_code, throw an exception
                    else:
                        log('Server at {} returned status code {} and no JSON data'.format(domain,
                                                                                           response.status_code),
                            level=lg.ERROR)
                        raise Exception(
                            'Server returned no JSON data.\n{} {}\n{}'.format(response, response.reason, response.text))

                return response_json

        def get_node(element):
            """
            Convert an OSM node element into the format for a networkx node.
            Parameters
            ----------
            element : dict
                an OSM node element
            Returns
            -------
            dict
            """
            useful_tags_node = ['ref', 'highway', 'route_ref', 'asset_ref']

            node = {}
            node['y'] = element['lat']
            node['x'] = element['lon']
            node['osmid'] = element['id']

            if 'tags' in element:
                for useful_tag in useful_tags_node:
                    if useful_tag in element['tags']:
                        node[useful_tag] = element['tags'][useful_tag]
            return node

        def get_path(element, element_r):
            """
            Convert an OSM way element into the format for a networkx graph path.
            Parameters
            ----------
            element : dict
                an OSM way element
            element_r : dict
                an OSM way element
            Returns
            -------
            dict
            """
            useful_tags_path_e = ['bridge', 'tunnel', 'oneway', 'lanes', 'name',
                                  'highway', 'maxspeed', 'service', 'access', 'area',
                                  'landuse', 'width', 'est_width', 'junction']

            useful_tags_path_r = ['bridge', 'tunnel', 'oneway', 'lanes', 'ref', 'direction', 'from', 'to', 'name',
                                  'highway', 'maxspeed', 'service', 'access', 'area',
                                  'landuse', 'width', 'est_width', 'junction']

            path = {}
            path['osmid'] = element['id']

            # remove any consecutive duplicate elements in the list of nodes
            grouped_list = groupby(element['nodes'])
            path['nodes'] = [group[0] for group in grouped_list]

            if 'tags' in element:
                # for relation in element_r['elements']:
                #     if relation['type'] == 'relation':
                #         for members in relation['members']:
                #             if members['ref'] == element['id']:
                for useful_tag in useful_tags_path_e:
                    if useful_tag in element['tags']:
                        path[useful_tag] = element['tags'][useful_tag]
                # for useful_tag in useful_tags_path_r:
                #     if useful_tag in relation['tags']:
                #         try:
                #             path[useful_tag] = path[useful_tag] + ";" + relation['tags'][useful_tag]
                #         except KeyError:
                #             path[useful_tag] = relation['tags'][useful_tag]
                #             pass

            return path

        def parse_osm_nodes_paths(osm_data):
            """
            Construct dicts of nodes and paths with key=osmid and value=dict of
            attributes.
            Parameters
            ----------
            osm_data : dict
                JSON response from from the Overpass API
            Returns
            -------
            nodes, paths : tuple
            """

            nodes = {}
            paths = {}
            relation = {}

            # for element in osm_data['elements']:
            #     if element['type'] == 'relation':

            for element in osm_data['elements']:
                if element['type'] == 'node':
                    key = element['id']
                    nodes[key] = get_node(element)

                elif element['type'] == 'way':  # osm calls network paths 'ways'
                    key = element['id']
                    # pp.pprint(element)
                    paths[key] = get_path(element, osm_data)

            return nodes, paths

        def create_graph(response_jsons, name='unnamed', retain_all=True, bidirectional=False):
            """
            Create a networkx graph from Overpass API HTTP response objects.
            Parameters
            ----------
            response_jsons : list
                list of dicts of JSON responses from from the Overpass API
            name : string
                the name of the graph
            retain_all : bool
                if True, return the entire graph even if it is not connected
            bidirectional : bool
                if True, create bidirectional edges for one-way streets
            Returns
            -------
            networkx multidigraph
            """

            log('Creating networkx graph from downloaded OSM data...')
            start_time = time.time()

            # make sure we got data back from the server requests
            elements = []
            # for response_json in response_jsons:
            elements.extend(response_json['elements'])
            if len(elements) < 1:
                raise EmptyOverpassResponse('There are no data elements in the response JSON objects')

            # create the graph as a MultiDiGraph and set the original CRS to default_crs
            G = nx.MultiDiGraph(name=name, crs=settings.default_crs)

            # extract nodes and paths from the downloaded osm data
            nodes = {}
            paths = {}
            # for osm_data in response_jsons:
            nodes_temp, paths_temp = parse_osm_nodes_paths(response_jsons)
            for key, value in nodes_temp.items():
                nodes[key] = value
            for key, value in paths_temp.items():
                paths[key] = value

            # add each osm node to the graph
            for node, data in nodes.items():
                G.add_node(node, **data)

            # add each osm way (aka, path) to the graph
            G = ox.add_paths(G, paths, bidirectional=bidirectional)

            # retain only the largest connected component, if caller did not
            # set retain_all=True
            if not retain_all:
                G = get_largest_component(G)

            log('Created graph with {:,} nodes and {:,} edges in {:,.2f} seconds'.format(len(list(G.nodes())),
                                                                                         len(list(G.edges())),
                                                                                         time.time() - start_time))

            # add length (great circle distance between nodes) attribute to each edge to
            # use as weight
            if len(G.edges) > 0:
                G = ox.add_edge_lengths(G)

            return G

        def calculate_H(s_lat, s_lon, e_lat, e_lon):
            """
            Calculate a distance with x,y coordinates with
            Parameters
            ----------
            s_lat : float (starting lat)
            s_lon : float (starting lon)
            e_lat : float (ending lat)
            e_lon : float (ending lon)
            Returns
            -------
            distance
            """
            R = 6371.0
            snlat = radians(s_lat)
            snlon = radians(s_lon)
            elat = radians(e_lat)
            elon = radians(e_lon)
            actual_dist = 6371.01 * acos(sin(snlat) * sin(elat) + cos(snlat) * cos(elat) * cos(snlon - elon))
            actual_dist = actual_dist * 1000
            return actual_dist

        def bus_details_SD(adjacent_list):
            """
            store all details from LTA data mall into dictionary
            Parameters
            ----------
            adjacent_list : dict
            Returns
            -------
            adjacent_list : dict
            """

            temp = 0
            for x in results:
                if temp != x.get('ServiceNo'):
                    temp = x.get('ServiceNo')
                    count = 0
                    adja_bus_stop = my_dictionary()
                    adjacent_list.add(temp, adja_bus_stop)
                    adja_bus_stop.add(count, [x.get('BusStopCode'), x.get('Distance')])
                    count += 1
                else:
                    adja_bus_stop.add(count, [x.get('BusStopCode'), x.get('Distance')])
                    count += 1
            return adjacent_list

        def get_nearestedge_node(osm_id, a, G):
            """
            Find the nearest node available in Open street map
            Parameters
            ----------
            osm_id : node ID
            a : plotting graph
            g : bus graph
            Returns
            -------
            temp_nearest_edge[1]/temp_nearest_edge[2] : nearest node to a way ID
            """
            temp_y = G.nodes.get(osm_id).get('y')
            temp_x = G.nodes.get(osm_id).get('x')
            temp_nearest_edge = ox.get_nearest_edge(a, (temp_y, temp_x))
            temp_1 = temp_nearest_edge[0].coords[0]
            temp_2 = temp_nearest_edge[0].coords[1]
            temp1_x = temp_1[0]
            temp1_y = temp_1[1]
            temp_1_distance = calculate_H(temp1_y, temp1_x, temp_y, temp_x)

            temp2_x = temp_2[0]
            temp2_y = temp_2[1]
            temp_2_distance = calculate_H(temp2_y, temp2_x, temp_y, temp_x)
            if temp_1_distance < temp_2_distance:
                return temp_nearest_edge[1]
            else:
                return temp_nearest_edge[2]

        def delete_duplicate(x):
            """
            Delete duplicate within a list
            Parameters
            ----------
            x : list
            Returns
            -------
            list
            """
            return list(dict.fromkeys(x))

        def request_busG():
            """
            Find all nodes that is a bus stop
            Returns
            -------
            busG : dict
            """
            busG = {}
            for x in G.nodes.items():
                if x[1].get('highway') == 'bus_stop':
                    xy = []
                    xy.append(x[1].get('osmid'))
                    xy.append(x[1].get('x'))
                    xy.append(x[1].get('y'))
                    busG[x[1].get('osmid')] = xy

            return busG

        # ---MAIN---#

        query_str = '[out:json][timeout:180];node["type"="route"](1.385700,103.887300,1.422000,103.925900);way["type"="route"](1.385700,103.887300,1.422000,103.925900);(relation["type"="route"](1.385700,103.887300,1.422000,103.925900);>;);out;'
        response_json = overpass_request(data={'data': query_str}, timeout=180)
        pp = pprint.PrettyPrinter(indent=4)
        # start = 1847853709
        # end = 410472575
        # end = 3737148763
        # bus transit
        # start = 2110621974
        # end = 2085845884

        adjacent_list = my_dictionary()

        G = ox.load_graphml('Bus_Overpass.graphml')

        if case == 1:
            return request_busG()
        n, e = ox.graph_to_gdfs(G)
        # e.to_csv("Edge_test_busstop.csv")

        if len(results) == 0:
            results = bus_details_all(
                results)  # Details from LTA Datamall, extracting all details such as service no, bus stop number

        adjacent_list = bus_details_SD(adjacent_list)  # From results, it extracts bus stop number and distance
        start_busstop = (G.nodes.get(start)).get('asset_ref')
        end_busstop = (G.nodes.get(end)).get('asset_ref')

        # Start finding common bus service within the start bus stop and end bus stop
        if ";" in (G.nodes.get(start).get('route_ref')):
            start_rr = (G.nodes.get(start).get('route_ref')).split(";")
        else:
            start_rr = []
            start_rr.append((G.nodes.get(start).get('route_ref')))
        if ";" in (G.nodes.get(end).get('route_ref')):
            end_rr = (G.nodes.get(end).get('route_ref')).split(";")
        else:
            end_rr = []
            end_rr.append((G.nodes.get(end).get('route_ref')))
        common = list(set(start_rr) & set(end_rr))

        """
        This method strictly emphasis on greedy algorithm. Thus it will prioritze the numbers of transit rather than distance
        Check if any common bus service within start and end bus stop.
        If found, route_list will capture the entire route of the common bus service 
        No transit will occuer as it is a straight path, start busstop -> end busstop
        If not found, the program will proceed to find a common bus stop within the start and end bus services. 
        Thus a transit will occur, start busstop -> mid busstop -> end busstop
        """
        route_list = {}
        mid_route_list = {}
        print("TEST - Start: ", start_busstop)
        print("TEST - End: ", end_busstop)
        print("TEST - start_rr: ", start_rr)
        print("TEST - end_rr: ", end_rr)
        print("TEST - Common: ", common)
        common_mid = []
        if len(common) == 0:  # No common bus service found
            while (len(common_mid) == 0):  # Start finding a common mid busstop
                rona_one = []
                rona_two = []
                for start_to_mid in start_rr:  # Capture all common mid busstop
                    print("TEST - start_to_mid: ", start_to_mid)
                    for bus_sequence in adjacent_list.get(start_to_mid):
                        rona_one.append(str(adjacent_list.get(start_to_mid).get(bus_sequence)[0]))
                    for mid_to_end in end_rr:
                        print("TEST - mid_to_end: ", mid_to_end)
                        for bus_sequence in adjacent_list.get(mid_to_end):
                            rona_two.append(str(adjacent_list.get(mid_to_end).get(bus_sequence)[0]))
                        found_br = []
                        print("TEST rona 1:", rona_one)
                        print("TEST rona 2:", rona_two)
                        found_br.append(start_to_mid + ";" + mid_to_end)
                        found_br.extend(list(set(rona_one) & set(rona_two)))
                        common_mid.append(found_br)

                    print("TEST - common_mid: ", common_mid)

                    bus_service = start_to_mid
                    temp_bus = []
                    mid_busstop = 0
                    approved = 0
                    for bus_sequence in adjacent_list.get(
                            bus_service):  # Finding bus service for start busstop -> mid busstop
                        for x in range(0, len(common_mid)):
                            for i in common_mid[x]:
                                if str(adjacent_list.get(bus_service).get(bus_sequence)[0]) == str(start_busstop):
                                    temp_bus.append(adjacent_list.get(bus_service).get(bus_sequence)[0])
                                    approved = 1
                                if str(adjacent_list.get(bus_service).get(bus_sequence)[0]) == str(i) and approved == 1:
                                    mid_busstop = str(i)
                                    temp_bus.append(adjacent_list.get(bus_service).get(bus_sequence)[0])
                                    approved = 0
                                    break
                                if approved == 1:
                                    temp_bus.append(adjacent_list.get(bus_service).get(bus_sequence)[0])
                            if mid_busstop != 0:
                                break
                    if str(start_busstop) not in temp_bus or str(
                            mid_busstop) not in temp_bus:  # If not found, continue to next loop
                        continue
                    temp_bus = delete_duplicate(temp_bus)
                    mid_route_list[bus_service] = temp_bus

            for x in G.nodes:  # After finding bus service to mid busstop, start finding path mid busstop to end busstop
                if G.nodes.get(x).get('asset_ref') == mid_busstop:
                    if ";" in (G.nodes.get(x).get('route_ref')):
                        start_rr = (G.nodes.get(x).get('route_ref')).split(";")
                    else:
                        start_rr = []
                        start_rr.append((G.nodes.get(start).get('route_ref')))

            common = list(set(start_rr) & set(end_rr))
            start_busstop = mid_busstop
        if start == 1847853709:  # If bus service started from punggol interchange
            for bus_service in common:
                temp_bus = []
                approved = 0
                for bus_sequence in adjacent_list.get(bus_service):  # Capture bus route
                    if str(adjacent_list.get(bus_service).get(bus_sequence)[0]) == str(start_busstop) and \
                            adjacent_list.get(bus_service).get(bus_sequence)[1] == 0:
                        temp_bus.append(adjacent_list.get(bus_service).get(bus_sequence)[0])
                        approved = 1
                    if str(adjacent_list.get(bus_service).get(bus_sequence)[0]) == str(end_busstop) and approved == 1:
                        temp_bus.append(adjacent_list.get(bus_service).get(bus_sequence)[0])
                        approved = 0
                        break
                    if approved == 1:
                        temp_bus.append(adjacent_list.get(bus_service).get(bus_sequence)[0])
                if str(start_busstop) not in temp_bus or str(end_busstop) not in temp_bus:
                    continue
                route_list[bus_service] = temp_bus
        else:
            for bus_service in common:  # If bus service does not start from punggol interchange
                temp_bus = []
                approved = 0
                for bus_sequence in adjacent_list.get(bus_service):  # Capture bus route
                    if str(adjacent_list.get(bus_service).get(bus_sequence)[0]) == str(start_busstop):
                        temp_bus.append(adjacent_list.get(bus_service).get(bus_sequence)[0])
                        approved = 1
                    if str(adjacent_list.get(bus_service).get(bus_sequence)[0]) == str(end_busstop) and approved == 1:
                        temp_bus.append(adjacent_list.get(bus_service).get(bus_sequence)[0])
                        approved = 0
                        break
                    if approved == 1:
                        temp_bus.append(adjacent_list.get(bus_service).get(bus_sequence)[0])
                if str(start_busstop) not in temp_bus or str(end_busstop) not in temp_bus:
                    continue
                route_list[bus_service] = temp_bus

        """
        After capturing all the bus serivce. A comparison is made in favor for the number of bus stops
        It will choose the least amount of bus stops and store in post_compare
        """
        compare = [0, 100]
        if len(route_list.keys()) > 1:
            for i in route_list:
                if len(route_list.get(i)) < compare[1]:
                    compare[0] = i
                    compare[1] = len(route_list.get(i))
        else:
            for i in route_list:
                compare[0] = i
                compare[1] = len(route_list.get(i))
        post_compare = []
        print("TEST - Mid route list: ", mid_route_list)
        if len(mid_route_list) != 0:
            for i in mid_route_list:
                post_compare.append(i)
                route_list[i] = mid_route_list.get(i)
            post_compare.append(compare[0])
        else:
            post_compare.append(compare[0])

        """
        Upon comparison, it will start capturing the nodes within the bus path and store in plot_list
        """
        plot_list = []
        try:
            print("TEST - post_Compare: ", post_compare)
            print("TEST - Route list: ", route_list)
            for count in range(0, len(post_compare)):
                for x in route_list.get(str(post_compare[count])):
                    for i in G.nodes:
                        if str(G.nodes.get(i).get('asset_ref')) == str(x):
                            plot_list.append(G.nodes.get(i).get('osmid'))
                            break
        except:
            return -1
        edge_list = []
        punggol = (1.403948, 103.909048)
        """
        It will generate out the list of node ID for the UI to plot
        """
        a = ox.load_graphml('Bus_graph.graphml')
        for x in plot_list:
            edge_list.append(get_nearestedge_node(x, a, G))

        print("TEST - Plot list: ", plot_list)
        print("TEST - Edge list: ", edge_list)
        final_route_list = []
        count_stops = len(plot_list)
        for x in range(0, len(edge_list) - 1):
            final_route_list.append(nx.shortest_path(a, edge_list[x], edge_list[x + 1]))

        return final_route_list

    def bus_details_all():
        headers = {
            'AccountKey': '84lbH3B/QeOkRK/CHm3c2w==',
            'UniqueUserID': '8ecabd56-08a2-e843-0a7a-9944dccf124a',
            'accept': 'application/json'
        }
        global new_results
        if __name__ == "__main__":
            results = []
            bus_stop_url = "http://datamall2.mytransport.sg/ltaodataservice/BusRoutes"

            while True:
                new_results = requests.get(bus_stop_url, headers=headers, params={'$skip': len(results)}).json()[
                    'value']
                if new_results == []:
                    return results
                else:
                    results += new_results

    def calculate_H(s_lat, s_lon, e_lat, e_lon):  # y,x y,x
        R = 6371.0
        snlat = radians(s_lat)
        snlon = radians(s_lon)
        elat = radians(e_lat)
        elon = radians(e_lon)
        actual_dist = 6371.01 * acos(sin(snlat) * sin(elat) + cos(snlat) * cos(elat) * cos(snlon - elon))
        actual_dist = actual_dist * 1000
        return actual_dist

    def walk_pathfinder(start_osmid, end_osmid):
        priority_Q = []
        heap_Q = []
        closed_routes = {}
        start_node = (0, None, start_osmid, 0)
        heapq.heappush(heap_Q, (start_node))
        closed_routes[start_osmid] = None
        while (True):
            temp = heapq.heappop(heap_Q)
            if temp[2] == end_osmid:
                temp_end = end_osmid
                path = []
                path.append(end_osmid)
                while (temp_end is not None):
                    temp_list = closed_routes.get(temp_end)
                    if temp_list is not None:
                        temp_end = temp_list[0]
                        path.append(temp_end)
                    else:
                        final_path = path[::-1]
                        return final_path

            for counter, x in enumerate(list(G.edges())[0:]):
                if x[0] == temp[2]:
                    if x[1] in closed_routes:
                        continue
                    else:
                        length = list(G.edges.values())[counter].get("length", None)
                        current_length = length + temp[3]
                        slat = radians(G.nodes.get(x[1]).get('y'))
                        slon = radians(G.nodes.get(x[1]).get('x'))
                        dist = 6371.01 * acos(sin(slat) * sin(elat) + cos(slat) * cos(elat) * cos(slon - elon))
                        H = dist * 1000
                        if H < actual_dist + 100:
                            F = current_length + H
                            heapq.heappush(heap_Q, (F, x[0], x[1], current_length))
                            closed_routes[x[1]] = [x[0], length]

    def delete_duplicate(x):
        return list(dict.fromkeys(x))

    def get_nearestedge_node(osm_id, y, x):
        temp_nearest_edge = ox.get_nearest_edge(G, (y, x))
        temp_1 = temp_nearest_edge[0].coords[0]
        temp_2 = temp_nearest_edge[0].coords[1]
        temp1_x = temp_1[0]
        temp1_y = temp_1[1]
        temp_1_distance = calculate_H(temp1_y, temp1_x, y, x)
        temp2_x = temp_2[0]
        temp2_y = temp_2[1]
        temp_2_distance = calculate_H(temp2_y, temp2_x, y, x)
        if temp_1_distance < temp_2_distance:
            return [temp_nearest_edge[1], temp_1_distance, temp1_x, temp1_y]
        else:
            return [temp_nearest_edge[2], temp_2_distance, temp2_x, temp2_y]

    def find_XY(node, graph):
        for x in graph.nodes.items():
            if x[1].get('osmid') == node:
                node_x = x[1].get('x')
                node_y = x[1].get('y')
                node_list = (node_y, node_x)
                return node_list

    start_time = time.time()

    # start = (103.9028788, 1.4044948)
    # end = (103.8999124, 1.4035004)
    # start = (103.9073345, 1.4060506)
    # end = (103.9172982, 1.3956014)
    #
    # start = (103.9073345, 1.4060506)
    # end = (103.9172982, 1.3956014)

    # start = (103.910650, 1.400818)
    # end = (103.910296, 1.399252)

    # start =(103.9024 , 1.4052)
    # end = (103.897332 , 1.402272)

    # start = (103.91256451606752, 1.402580108598971)
    # end = (103.91270935535432, 1.401523634635178)

    start_osmid = 0
    end_osmid = 0
    punggol = (1.403948, 103.909048)
    G = ox.load_graphml('AStar_walk.graphml')
    nodelist_G = list(G.nodes.values())

    """
    Start finding start and end Node ID.
    If not found, find nearest node from the given coordinates by the user
    """
    for i in range(0, len(nodelist_G)):
        if nodelist_G[i].get('y') == start[1] and nodelist_G[i].get('x') == start[0]:
            start_osmid = nodelist_G[i].get('osmid')
        if nodelist_G[i].get('y') == end[1] and nodelist_G[i].get('x') == end[0]:
            end_osmid = nodelist_G[i].get('osmid')

    if start_osmid == 0 or end_osmid == 0:
        start_osmid = ox.get_nearest_node(G, (start[1], start[0]))
        end_osmid = ox.get_nearest_node(G, (end[1], end[0]))

    """
    To calculate distance from 2 x,y axis
    """
    R = 6371.0
    snlat = radians(start[1])
    snlon = radians(start[0])
    elat = radians(end[1])
    elon = radians(end[0])
    actual_dist = 6371.01 * acos(sin(snlat) * sin(elat) + cos(snlat) * cos(elat) * cos(snlon - elon))
    actual_dist = actual_dist * 1000
    edgelist_G = list(G.edges.values())

    """
    After having start and end nodes.
    The program will set a radius of 200 meters from start and end nodes
    Every nodes within 200 meters and is a bus stop node will be captured and stored in end1 and end2
    If within 200meters no bus stop is found, it will have a constant increment of 200meters until bus stop if found on both sides
    """
    bus_G = bus_layer(start_osmid, end_osmid, None, 1)
    start1 = start
    start2 = end

    for i in bus_G:
        temp_dis = calculate_H(bus_G.get(i)[2], bus_G.get(i)[1], start1[1], start1[0])
        bus_G.get(i).append(temp_dis)
        temp_dis = calculate_H(bus_G.get(i)[2], bus_G.get(i)[1], start2[1], start2[0])
        bus_G.get(i).append(temp_dis)
    end1 = []
    end2 = []
    limit = 0
    while (len(end1) == 0):
        limit += 200
        for i in bus_G:
            if bus_G.get(i)[3] < limit:
                temp = []
                temp.append(bus_G.get(i)[3])
                temp.append(bus_G.get(i)[0])
                temp.append(bus_G.get(i)[1])
                temp.append(bus_G.get(i)[2])
                hq.heappush(end1, temp)
    limit = 0
    while (len(end2) == 0):
        limit += 200
        for i in bus_G:
            if bus_G.get(i)[4] < limit:
                temp = []
                temp.append(bus_G.get(i)[4])
                temp.append(bus_G.get(i)[0])
                temp.append(bus_G.get(i)[1])
                temp.append(bus_G.get(i)[2])
                hq.heappush(end2, temp)

    """
    The following codes will capture all nodes on the road that is closest to the bus stop
    It will be stored in path1 and path2.
    """
    path1 = []
    for i in range(0, len(end1)):
        if 1847853709 == end1[i][1]:
            path1 = []
            path1.append([2019165453, 0, 0, 0, 0])
            break
        else:
            path1.append(get_nearestedge_node(end1[i][1], end1[i][3], end1[i][2]))

    for x in range(0, len(path1)):
        path1[x].append(calculate_H(path1[x][3], path1[x][2], start1[1], start1[0]))

    path2 = []
    for i in range(0, len(end2)):
        path2.append(get_nearestedge_node(end2[i][1], end2[i][3], end2[i][2]))
    for x in range(0, len(path2)):
        path2[x].append(calculate_H(path2[x][3], path2[x][2], start2[1], start2[0]))

    """
    Bus results will store all data obtained from lta datamall
    It will start calculating all possibilities from all bus stop captured in end1 and end2
    Example, end1 contains [1,2,3], end2 contains [4,5,6]
    The following code will start to find a route from [1,4] , [1,5] , [1,6] then [2,4] , [2,5] , [2,6] then [3,4] , [3,5] , [3,6]
    Once all these route is found, it will proceed to compare the derived routes and capture the least amount of bus stop
    Example, [1,4] is the shortest route found
    Upon capturing the route with the least amount of bus stop, it will start to plot the walking A* algorithm from start point to bus stop
    Example, [Start point, 1] then [End point, 4]
    In this case, it will return [[Start point,1] , [1,4] , [End point,4]]
    """
    with open("data\ltadatamall.txt", "r") as filehandler:
        bus_results=json.load(filehandler)
    approved = 0
    path1_end_count = 0
    path2_end_count = 0
    for i in range(0, len(end1)):
        if 1847853709 == end1[i][1]:
            approved = 1
    final_route_list = []
    if approved == 1:
        count = 99
        for x in range(0, len(end2)):
            final_route_list = bus_layer(1847853709, end2[x][1], bus_results, None)
            if len(final_route_list) < count:
                path1[path1_end_count][0] = 4598672210
                path2_end_count = x
                temp_route_list = final_route_list.copy()
                count = len(temp_route_list)
    else:
        count = 99
        if len(final_route_list) == 0:
            for i in range(0, len(end1)):
                for x in range(0, len(end2)):
                    final_route_list = bus_layer(end1[i][1], end2[x][1], bus_results, None)
                    if final_route_list == -1:
                        continue
                    if len(final_route_list) < count:
                        path1_end_count = i
                        path2_end_count = x
                        temp_route_list = final_route_list
                        count = len(temp_route_list)

    path1 = walk_pathfinder(start_osmid, path1[path1_end_count][0])
    path2 = walk_pathfinder(end_osmid, path2[path2_end_count][0])
    walking_Path1 = []
    walking_Path2 = []
    bus_path = []
    for x in path1:
        walking_Path1.append(find_XY(x, G))
    for x in path2:
        walking_Path2.append(find_XY(x, G))

    ox.plot_graph_routes(G, [path1, path2])
    plotting_route = []

    """
    Upon capturing all the bus routes and walking routes, it will proceed to return the route for further processing
    """

    a = ox.load_graphml('WalkBus_end_graph.graphml')
    for x in temp_route_list:
        plotting_route.extend(x)
    plotting_route = delete_duplicate(plotting_route)
    Tried = True
    try:
        ox.plot_graph_route(a, plotting_route)
        for x in plotting_route:
            bus_path.append(find_XY(x, a))
    except:
        ox.plot_graph_routes(a, temp_route_list)
        Tried = False
        for x in temp_route_list:
            for i in x:
                bus_path.append(find_XY(i, a))

    print("TEST - Start OSMID: ", start_osmid)
    print("TEST - End OSMID: ", end_osmid)
    print("TEST - Path 1: ", path1)
    print("TEST - Path 1 (X,Y): ", walking_Path1)
    print("TEST - Path 2: ", path2)
    print("TEST - Path 2 (X,Y): ", walking_Path2)
    print("TEST - BusRoute: ", plotting_route)
    print("TEST - Bus Path (X,Y): ", bus_path)

    walking_Path1.insert(0, (start[1], start[0]))
    walking_Path2.append((end[1], end[0]))

    # ox.plot_graph_route(G, final_path, fig_height=10, fig_width=10)
    if Tried == True:
        return [walking_Path1, bus_path, walking_Path2]
    else:
        return [walking_Path1, bus_path, walking_Path2]

    print("--- %s seconds ---" % (time.time() - start_time))


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

# =========================================================
# END OF "PLACE IN INIT" requirements
# =========================================================

# now, time to make sure primary graph has these nodes as coordinates.

# prints all node IDs that exists in main node graph

n, e = ox.graph_to_gdfs(G)
lrt_n, lrt_e = ox.graph_to_gdfs(lrt_stations)

# =========================================================
# TESTING NODES, REPLACE WITH REAL USER INPUT
# =========================================================

node1 = (103.9022239, 1.4053828)
node2 = (103.91602993011, 1.4030734871395)

# =========================================================
# END OF "TESTING NODES"
# =========================================================

# =========================================================
# ROUTE FUNCTION CALLING, this returns arrays of XY coordinates
# =========================================================

routes = lrt_walk(G, node1, node2, lrt_stations)
# routes = lrt_bus_walk(G, node1, node2, lrt_stations)

lrtLast = routes[0][-1]
routes[1].insert(0, lrtLast)

# print("PRINTING X,Y OF ROUTE", routes)
# print("lrt combined routes: ", routes)