import osmnx as ox
import pickle

place_name = "Moldova"

drive_G = ox.graph_from_place(place_name, network_type='drive')
walk_G = ox.graph_from_place(place_name, network_type='walk')

ox.routing.add_edge_speeds(drive_G, fallback=50)
ox.routing.add_edge_travel_times(drive_G)

with open("drive_G.pkl", "wb") as f:
    pickle.dump(drive_G, f)

with open("walk_G.pkl", "wb") as f:
    pickle.dump(walk_G, f)

print("Graphs successfully downloaded and saved as pickle files!")