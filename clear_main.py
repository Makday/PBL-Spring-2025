import pickle
import osmnx as ox
import networkx as nx
import time

USER_WALK_SPEED = 1.4  # meters/seconds
CHECK_RADIUS = 2000 # meters
DEFAULT_SPEED_LIMIT = 50  # km/h
KMH_TO_MS = 3.6  # conversion factor

with open("drive_G.pkl", "rb") as f:    drive_G = pickle.load(f)
with open("walk_G.pkl", "rb") as f:     walk_G = pickle.load(f)

def get_drive_path(start_coords, end_coords, drive_G):
    start_lat, start_lon = start_coords
    end_lat, end_lon = end_coords
    start_node = ox.distance.nearest_nodes(drive_G, X=start_lon, Y=start_lat)
    end_node = ox.distance.nearest_nodes(drive_G, X=end_lon, Y=end_lat)
    return nx.shortest_path(drive_G, source=start_node, target=end_node, weight="length")

def get_user_route(start_coords, end_coords, routes, walk_G, drive_G, USER_WALK_SPEED, CHECK_RADIUS):
    start_lat, start_lon = start_coords
    end_lat, end_lon = end_coords

    start_walk = ox.distance.nearest_nodes(walk_G, X=start_lon, Y=start_lat)
    end_walk = ox.distance.nearest_nodes(walk_G, X=end_lon, Y=end_lat)

    distance_dict_pickup = nx.single_source_dijkstra_path_length(walk_G, source=start_walk, cutoff=CHECK_RADIUS, weight='length')
    distance_dict_dropoff = nx.single_source_dijkstra_path_length(walk_G, source=end_walk, cutoff=CHECK_RADIUS, weight='length')

    min_total_time = float('inf')
    best_route_index = -1
    for i, route in enumerate(routes):
        driver_nodes_set = set(route)

        pickup_candidates = [(node, dist) for node, dist in distance_dict_pickup.items() if node in driver_nodes_set]
        if not pickup_candidates:
            continue
        pickup_node, len_pick_up_path = min(pickup_candidates, key=lambda x: x[1])

        dropoff_candidates = [(node, dist) for node, dist in distance_dict_dropoff.items() if node in driver_nodes_set]
        if not dropoff_candidates:
            continue
        dropoff_node, len_dropoff_to_end_path = min(dropoff_candidates, key=lambda x: x[1])

        pick_up_time = len_pick_up_path / USER_WALK_SPEED
        dropoff_to_end_time = len_dropoff_to_end_path / USER_WALK_SPEED

        wait_time = nx.shortest_path_length(drive_G, source=route[0], target=pickup_node, weight='travel_time')
        driving_time = nx.shortest_path_length(drive_G, source=pickup_node, target=dropoff_node, weight='travel_time')

        total_user_time = wait_time + pick_up_time + driving_time + dropoff_to_end_time

        print(f"Route {i}: wait time {wait_time:.1f}s, walk to pickup {pick_up_time:.1f}s, drive {driving_time:.1f}s, walk from dropoff {dropoff_to_end_time:.1f}s, total {total_user_time:.1f}s")

        if total_user_time < min_total_time:
            min_total_time = total_user_time
            best_route_index = i

    return (min_total_time, best_route_index)

# driver routes
start1 = (46.999334786789575, 28.872833728440337)
end1 = (47.012661460108596, 28.855908564441723)
start2 = (47.027120260951015, 28.847004323106944)
end2 = (46.99425959639958, 28.846074823265926)
start3 = (46.99873402121167, 28.89602068961361)
end3 = (46.98797917804514, 28.82953045266552)
start4 = (46.98836212816632, 28.78268010995544)
end4 = (47.00649689732322, 28.82524349144823)
start5 = (46.2118038363676, 28.635244068298608)
end5 = (47.81097380127941, 28.44847650153039)

routes = []

for (start, end) in [(start1, end1), (start2, end2), (start3, end3), (start4, end4), (start5, end5)]:
    route = get_drive_path(start, end, drive_G)
    routes.append(route)

user_start = (47.024043038307035, 28.866757067308278)
user_end = (47.01034993294552, 28.83770338102563)

start_time = time.time()
min_time, best_index = get_user_route(user_start, user_end, routes, walk_G, drive_G, USER_WALK_SPEED, CHECK_RADIUS)
print("Time to execute best route function:",time.time() - start_time)

print(f"Best driver route index: {best_index}, with user total time: {min_time:.1f}s")