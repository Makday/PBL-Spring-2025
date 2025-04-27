import numpy as np
import osmnx as ox
import networkx as nx

USER_WALK_SPEED = 1.4  # meters/seconds
CHECK_RADIUS = 2000 # meters

drive_G = ox.load_graphml("Moldova_graph_drive.graphml")
walk_G = ox.load_graphml("Moldova_graph_walk.graphml")

def get_drive_path(start_coords, end_coords, drive_G):
    start_lat, start_lon = start_coords
    end_lat, end_lon = end_coords
    start_node = ox.distance.nearest_nodes(drive_G, X=start_lon, Y=start_lat)
    end_node = ox.distance.nearest_nodes(drive_G, X=end_lon, Y=end_lat)
    shortest_path = nx.shortest_path(drive_G, source=start_node, target=end_node, weight="length")
    return shortest_path

def get_user_route(start_coords, end_coords, routes, walk_G, drive_G, USER_WALK_SPEED, CHECK_RADIUS):
    start_lat, start_lon = start_coords
    end_lat, end_lon = end_coords

    user_start_walk = ox.distance.nearest_nodes(walk_G, X=start_lon, Y=start_lat)
    user_end_walk = ox.distance.nearest_nodes(walk_G, X=end_lon, Y=end_lat)

    min_total_time = float('inf')
    best_route_index = -1
    for i, route in enumerate(routes):
        x_array = np.array([drive_G.nodes[i]['x'] for i in route], dtype=np.float64)
        y_array = np.array([drive_G.nodes[i]['y'] for i in route], dtype=np.float64)
        potential_nodes = ox.distance.nearest_nodes(walk_G,X = x_array,Y = y_array)
        distance_dict_pickup = nx.single_source_dijkstra_path_length(walk_G, source = user_start_walk, cutoff=CHECK_RADIUS, weight='length')

        min_pickup_dist = float('inf')
        best_pickup_node = None
        for node_id in potential_nodes:
            if node_id in distance_dict_pickup:
                walk_distance = distance_dict_pickup[node_id]
                if walk_distance < min_pickup_dist:
                    min_pickup_dist = walk_distance
                    best_pickup_node = node_id
        if best_pickup_node is None: continue  # account for no available pickup nodes

        distance_dict_dropoff = nx.single_source_dijkstra_path_length(walk_G, source=user_end_walk, cutoff=CHECK_RADIUS, weight='length')
        min_dropoff_dist = float('inf')
        best_dropoff_node = None
        for node_id in potential_nodes:
            if node_id in distance_dict_dropoff:
                walk_distance = distance_dict_dropoff[node_id]
                if walk_distance < min_dropoff_dist:
                    min_dropoff_dist = walk_distance
                    best_dropoff_node = node_id
        if best_dropoff_node is None: continue # account for no available drop-off nodes

        path_pickup_to_dropoff = nx.shortest_path(drive_G, source=ox.distance.nearest_nodes(drive_G, X=walk_G.nodes[best_pickup_node]['x'], Y=walk_G.nodes[best_pickup_node]['y']), target=ox.distance.nearest_nodes(drive_G, X=walk_G.nodes[best_dropoff_node]['x'], Y=walk_G.nodes[best_dropoff_node]['y']), weight='length')

        len_pick_up_path = nx.shortest_path_length(walk_G, source=user_start_walk, target=best_pickup_node, weight='length')
        len_dropoff_to_end_path = nx.shortest_path_length(walk_G, source=best_dropoff_node, target=user_end_walk, weight='length')

        pick_up_time = len_pick_up_path / USER_WALK_SPEED
        dropoff_to_end_time = len_dropoff_to_end_path / USER_WALK_SPEED

        driving_time = 0
        for u, v in zip(path_pickup_to_dropoff[:-1], path_pickup_to_dropoff[1:]):
            edge_data = drive_G[u][v][0]
            edge_length = edge_data['length']
            speed_limit = edge_data.get('maxspeed', 50)
            if isinstance(speed_limit, list):
                speed_limit = speed_limit[0]
            if isinstance(speed_limit, str):
                speed_limit = float(speed_limit.split()[0])
            speed_limit = speed_limit / 3.6
            driving_time += edge_length / speed_limit

        total_user_time = pick_up_time + driving_time + dropoff_to_end_time

        print(f"Route {i}: walk to pickup {pick_up_time:.1f}s, drive {driving_time:.1f}s, walk from dropoff {dropoff_to_end_time:.1f}s, total {total_user_time:.1f}s")

        if total_user_time < min_total_time:
            min_total_time = total_user_time
            best_route_index = i

    return (min_total_time, best_route_index)

# Driver routes
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

place_latitudes = [user_start[0], user_end[0], start1[0], end1[0], start2[0], end2[0], start3[0], end3[0], start4[0], end4[0]]
place_longitudes = [user_start[1], user_end[1], start1[1], end1[1], start2[1], end2[1], start3[1], end3[1], start4[1], end4[1]]

north, south = max(place_latitudes), min(place_latitudes)
east, west = max(place_longitudes), min(place_longitudes)

buffer = 0.03
north += buffer
south -= buffer
east += buffer
west -= buffer

min_time, best_index = get_user_route(user_start, user_end, routes, walk_G, drive_G, USER_WALK_SPEED, CHECK_RADIUS)
print(f"Best driver route index: {best_index}, with user total time: {min_time:.1f}s")
