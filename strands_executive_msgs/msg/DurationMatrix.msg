# Represents the travel times between nodes

# a list of node names, e.g. ['p1', 'p2', 'p3']
string[] node_names

# a list of lists to encode durations between the above nodes. the ith element of the durations list is a list of durations between the ith element of node_names and all the elements in node_names (in the same order as node_names)
DurationList[] durations
