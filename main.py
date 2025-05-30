import time
import pickle
import osmnx as ox
import networkx as nx
from math import cos, radians, sqrt
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

USER_WALK_SPEED = 1.4  # meters/seconds
USER_MAX_DISTANCE = 1680 # meters
CHECK_RADIUS = 2000 # meters
DEFAULT_SPEED_LIMIT = 50  # km/h
KMH_TO_MS = 3.6  # Conversion factor

start = time.time()
with open("drive_G.pkl", "rb") as f:    drive_G = pickle.load(f)
with open("walk_G.pkl", "rb") as f:     walk_G = pickle.load(f)
print("Graphs load time:", time.time() - start)

def euclidean_distance(lat1, lon1, lat2, lon2):
    mean_lat = radians((lat1 + lat2) / 2)
    km_per_deg_lat = 111.32
    km_per_deg_lon = 111.32 * cos(mean_lat)
    dlat_km = (lat2 - lat1) * km_per_deg_lat
    dlon_km = (lon2 - lon1) * km_per_deg_lon
    return sqrt(dlat_km * dlat_km  + dlon_km * dlon_km) * 1000 # meters

def calculate_path_length(path, G):
    return sum(G[u][v][0]['length'] for u, v in zip(path[:-1], path[1:]))

def get_drive_path(start_coords, end_coords, drive_G):
    start_lat, start_lon = start_coords
    end_lat, end_lon = end_coords
    start_node = ox.distance.nearest_nodes(drive_G, X=start_lon, Y=start_lat)
    end_node = ox.distance.nearest_nodes(drive_G, X=end_lon, Y=end_lat)
    shortest_path = nx.shortest_path(drive_G, source=start_node, target=end_node, weight="length")
    return shortest_path

def plot_route(graph, route, color, ax, linestyle='solid', linewidth=2, zorder=1, user = 0):
    edges = [(route[i], route[i+1]) for i in range(len(route)-1)]
    for u, v in edges:
        u_lat, u_lon = graph.nodes[u]['y'], graph.nodes[u]['x']
        v_lat, v_lon = graph.nodes[v]['y'], graph.nodes[v]['x']
        ax.plot([u_lon, v_lon], [u_lat, v_lat], color=color, linestyle=linestyle, linewidth=linewidth, zorder=zorder)
    if user == 0:
        start_node = route[0]
        start_lat, start_lon = graph.nodes[start_node]['y'], graph.nodes[start_node]['x']
        ax.scatter(start_lon, start_lat, color='green', s=30, zorder=10)

        end_node = route[-1]
        end_lat, end_lon = graph.nodes[end_node]['y'], graph.nodes[end_node]['x']
        ax.scatter(end_lon, end_lat, color='orange', s=30, zorder=10)

def get_user_route(start_coords, end_coords, routes, walk_G, drive_G, USER_WALK_SPEED, USER_MAX_DISTANCE, CHECK_RADIUS, ax):
    start_lat, start_lon = start_coords
    end_lat, end_lon = end_coords

    start_walk = ox.distance.nearest_nodes(walk_G, X=start_lon, Y=start_lat)
    end_walk = ox.distance.nearest_nodes(walk_G, X=end_lon, Y=end_lat)

    distance_dict_pickup = nx.single_source_dijkstra_path_length(walk_G, source=start_walk, cutoff=CHECK_RADIUS, weight='length')
    distance_dict_dropoff = nx.single_source_dijkstra_path_length(walk_G, source=end_walk, cutoff=CHECK_RADIUS, weight='length')

    ax.scatter(start_lon, start_lat, color='blue', s=50, label='User Start', zorder=5) #showing
    ax.scatter(end_lon, end_lat, color='red', s=50, label='User End', zorder=5) #showing

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

        color_list = list(mcolors.TABLEAU_COLORS.values())
        color = color_list[i % len(color_list)]

        path_to_pickup = nx.shortest_path(walk_G, source=start_walk, target=pickup_node,weight='length')  #showing
        path_dropoff_to_end = nx.shortest_path(walk_G, source=dropoff_node, target=end_walk, weight='length') #showing

        plot_route(drive_G, route, color=color, ax=ax, linewidth=2, zorder=1)     #showing
        plot_route(walk_G, path_to_pickup, color=color, ax=ax, linestyle='dotted', linewidth=2, zorder=2, user=1)   #showing
        plot_route(walk_G, path_dropoff_to_end, color=color, ax=ax, linestyle='dotted', linewidth=2, zorder=2, user=1)  #showing

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

'''routes.append(get_drive_path(start2, end2, drive_G))'''

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

fig, ax = plt.subplots(figsize=(12, 12))
G_map = ox.graph_from_bbox((west,south,east,north), network_type='all')
ox.plot_graph(G_map, ax=ax, node_size=0, edge_linewidth=0.5, show=False, bgcolor='lightgray')

start_time = time.time()
min_time, best_index = get_user_route(user_start, user_end, routes, walk_G, drive_G, USER_WALK_SPEED, USER_MAX_DISTANCE, CHECK_RADIUS, ax)
print("Time to execute best route function:",time.time() - start_time)

print(f"Best driver route index: {best_index}, with user total time: {min_time:.1f}s")

ax.set_title("Driver Routes and User Connections")
plt.legend()
plt.show()