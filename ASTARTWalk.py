def A_Star_Walk(start, end):
    import osmnx as ox
    from math import sin, cos, sqrt, atan2, radians, acos
    import heapq
    import time
    start_time = time.time()

    def find_XY(node, graph):
        for x in graph.nodes.items():
            if x[1].get('osmid') == node:
                node_x = x[1].get('x')
                node_y = x[1].get('y')
                node_list = (node_y, node_x)
                return node_list

    priority_Q = []
    heap_Q = []
    closed_routes = {}

    start_osmid = 0
    end_osmid = 0
    punggol = (1.403948, 103.909048)

    G = ox.load_graphml('AStar_walk.graphml')

    """
    Start finding start and end Node ID.
    If not found, find nearest node from the given coordinates by the user
    """
    nodelist_G = list(G.nodes.values())
    for i in range (0, len(nodelist_G)):
        if nodelist_G[i].get('y') == start[1] and nodelist_G[i].get('x') == start[0]:
            start_osmid = nodelist_G[i].get('osmid')
        if nodelist_G[i].get('y') == end[1] and nodelist_G[i].get('x') == end[0]:
            end_osmid = nodelist_G[i].get('osmid')

    if start_osmid == 0 or end_osmid == 0:
        start_osmid = ox.get_nearest_node(G, (start[1], start[0]))
        end_osmid = ox.get_nearest_node(G, (end[1], end[0]))

                 #f, parent, current, length
    start_node = (0, None, start_osmid, 0)

    heapq.heappush(heap_Q, (start_node))
    closed_routes[start_osmid] = None

    R = 6371.0

    snlat = radians(start[1])
    snlon = radians(start[0])
    elat = radians(end[1])
    elon = radians(end[0])
    actual_dist = 6371.01 * acos(sin(snlat) * sin(elat) + cos(snlat) * cos(elat) * cos(snlon - elon))
    actual_dist = actual_dist*1000

    edgelist_G = list(G.edges.values())

    """
    An A* algorithm is used to plot the shortest path
    Formula used F = G + H
    G = distance from start node to current node
    H = estimated distance from end node to current node
    A priority queue is used here as well by measuring F
    With this formula applied, it will continue to work towards the end goal
    """
    def walk_pathfinder(start_osmid, end_osmid):
        """
            It generates a walking path from start to end

            Parameters
            ----------
            start_osmid : node id
            end_osmid : node id

            Returns
            -------
            list[walking_path] : list of node ID
        """
        while(True):
            temp = heapq.heappop(heap_Q)
            if temp[2] == end_osmid: #If it reach the end, return path filled with node ID
                    temp_end = end_osmid
                    path = []
                    path.append(end_osmid)
                    while (temp_end is not None):
                        temp_list = closed_routes.get(temp_end)
                        if temp_list is not None:
                            temp_end =  temp_list[0]
                            path.append(temp_end)
                        else:
                            final_path = path[::-1]
                            return final_path
                            break
                    break

            for counter, x in enumerate(list(G.edges())[0:]): #While finding the next node, H must not exceed the initial distance from start node to end node.
                    if x[0] == temp[2]:
                        if x[1] in closed_routes:
                            continue
                        else:
                            length = list(G.edges.values())[counter].get("length", None)
                            current_length = length + temp[3]
                            slat = radians(G.nodes.get(x[1]).get('y'))
                            slon = radians(G.nodes.get(x[1]).get('x'))
                            dist = 6371.01 * acos(sin(slat) * sin(elat) + cos(slat) * cos(elat) * cos(slon - elon))
                            H = dist*1000
                            if H < actual_dist+100:
                                F = current_length + H
                                heapq.heappush(heap_Q, (F, x[0], x[1], current_length))
                                closed_routes[x[1]] = [x[0], length]

    final_path = walk_pathfinder(start_osmid, end_osmid)
    walk_XY = []
    walk_XY.append((start[1],start[0]))
    for i in final_path:
        walk_XY.append(find_XY(i,G))
    walk_XY.append((end[1],end[0]))
    return walk_XY

start = (103.9023302, 1.4052585)
end = (103.90864669999999, 1.4046221)

walkxy = A_Star_Walk(start,end)
print(walkxy)