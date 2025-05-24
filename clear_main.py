import pickle
import osmnx as ox
import networkx as nx
import heapq
from datetime import datetime, time, timezone

NUM_SAMPLES = 5 # numbers of optimal routes to be returned
USER_WALK_SPEED = 1.4  # meters/seconds
CHECK_RADIUS = 2000 # meters
DEFAULT_SPEED_LIMIT = 50  # km/h
KMH_TO_MS = 3.6  # conversion factor
routes = []

with open("drive_G.pkl", "rb") as f:    drive_G = pickle.load(f)
with open("walk_G.pkl", "rb") as f:     walk_G = pickle.load(f)

def add_route(start, end, start_time=None):
    path = get_drive_path(start, end)

    if start_time is None:
        today = datetime.now(timezone.utc).date()
        start_datetime = datetime.combine(today, time(hour=10, tzinfo=timezone.utc))
        start_time = start_datetime.isoformat()

    routes.append((path, start_time))

def get_drive_path(start, end):
    start_lat, start_lon = start
    end_lat, end_lon = end
    start_node = ox.distance.nearest_nodes(drive_G, X=start_lon, Y=start_lat)
    end_node = ox.distance.nearest_nodes(drive_G, X=end_lon, Y=end_lat)
    return nx.shortest_path(drive_G, source=start_node, target=end_node, weight="length")

def get_path_coords(node_ids,graph):
    coords = [[graph.nodes[node]['y'], graph.nodes[node]['x']] for node in node_ids]
    return coords

def get_user_route(start, end):
    start_lat, start_lon = start
    end_lat, end_lon = end

    start_walk = ox.distance.nearest_nodes(walk_G, X=start_lon, Y=start_lat)
    end_walk = ox.distance.nearest_nodes(walk_G, X=end_lon, Y=end_lat)

    pickup_lengths, pickup_paths = nx.single_source_dijkstra(walk_G, source=start_walk, cutoff=CHECK_RADIUS, weight='length')
    dropoff_lengths, dropoff_paths = nx.single_source_dijkstra(walk_G, source=end_walk, cutoff=CHECK_RADIUS, weight='length')

    route_objects = []

    for i, (route, start_time) in enumerate(routes):
        driver_nodes_set = set(route)

        pickup_candidates = [(node, dist) for node, dist in pickup_lengths.items() if node in driver_nodes_set]
        if not pickup_candidates:
            continue
        pickup_node, len_pick_up_path = min(pickup_candidates, key=lambda x: x[1])

        dropoff_candidates = [(node, dist) for node, dist in dropoff_lengths.items() if node in driver_nodes_set]
        if not dropoff_candidates:
            continue
        dropoff_node, len_dropoff_to_end_path = min(dropoff_candidates, key=lambda x: x[1])

        pickup_time = len_pick_up_path / USER_WALK_SPEED
        dropoff_to_end_time = len_dropoff_to_end_path / USER_WALK_SPEED

        pickup_idx = route.index(pickup_node)
        dropoff_idx = route.index(dropoff_node)

        wait_time = 0.0
        for j in range(pickup_idx):
            u, v = route[j], route[j + 1]
            edge_data = drive_G.get_edge_data(u, v, 0)
            wait_time += edge_data['travel_time']

        driving_time = 0.0
        for j in range(pickup_idx, dropoff_idx):
            u, v = route[j], route[j + 1]
            edge_data = drive_G.get_edge_data(u, v, 0)
            driving_time += edge_data['travel_time']

        total_user_time = wait_time + pickup_time + driving_time + dropoff_to_end_time
        walk_time = pickup_time + dropoff_to_end_time

        route_objects.append((i, pickup_node, dropoff_node, total_user_time, wait_time, driving_time, walk_time,  pickup_time, dropoff_to_end_time, start_time))

    route_objects = heapq.nsmallest(NUM_SAMPLES, route_objects, key=lambda x: x[3])
    results = []
    for obj in route_objects:
        driver_path = get_path_coords(routes[obj[0]][0], drive_G)
        pickup_path = get_path_coords(pickup_paths[obj[1]], walk_G)
        dropoff_path = get_path_coords(dropoff_paths[obj[2]], walk_G)

        result = {
            "driver_path": driver_path,
            "pickup_path": pickup_path,
            "dropoff_path": dropoff_path,
            "total_user_time": obj[3],
            "wait_time": obj[4],
            "drive_time": obj[5],
            "walk_time": obj[6],
            "pickup_time": obj[7],
            "dropoff_time": obj[8],
            "start_time": obj[9]
        }

        results.append(result)

    return results

#   Initially, run add_route(start, end) to add a driver route to the list
#   start,end have type list: [lat,lon], they denote origin and destination of DRIVER
#
#   To get optimal driver routes for user, run get_user_route(start, end)
#   start,end have type list: [lat,lon], they denote origin and destination of USER
#   NUM_SAMPLES - constant for adjusting how many optimal routes to return (Initially is set to 5 optimal routes)
#   get_user_route() returns a list of dictionaries, they have the following keys:
#
#   driver_path         - list of lists of coordinates of driver's path
#   pickup_path         - list of lists of coordinates of path from start to pickup
#   dropoff_path        - list of lists of coordinates of path from dropoff to users destination
#
#   total_user_time     - the sum of all times, basically total_user_time = drive_time + walk_time + wait_time
#
#   wait_time           - time from drivers start to pickup point
#   drive_time          - time from pickup to dropoff
#   walk_time           - time of both pickup and dropoff
#
#   pickup_time         - time of user reaching the pickup point
#   dropoff_time        - time of user reaching the end of his destination from dropoff
#
#   start_time          - when the driver will start driving, it's a string in ISO 8601