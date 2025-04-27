import osmnx as ox

place_name = "Moldova"

drive_G = ox.graph_from_place(place_name, network_type='drive')
walk_G = ox.graph_from_place(place_name, network_type='walk')

ox.save_graphml(drive_G, filepath="Moldova_graph_drive.graphml")
ox.save_graphml(walk_G, filepath="Moldova_graph_walk.graphml")

print("Graphs successfully downloaded!")
