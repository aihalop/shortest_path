import networkx as nx
import math

def calc_shortest_path(G, s, t):
    print(G)
    print(s)
    print(t)
    return dijkstra(G, s, t)


def dijkstra(G, s, t):
    '''reference: https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
    '''
    def get_nodes(G):
        return G.node.keys()

    def distance(s, v, G=None):
        return G[s][v]['weight']

    
    Q = get_nodes(G)
    dist = {k: float("inf") for k in get_nodes(G)}
    prev = {k: None for k in get_nodes(G)}
    dist[s] = 0

    while len(Q) > 0:
        unvisited_dist = {k:v for k,v in dist.items() if k in Q}
        u = min(unvisited_dist, key=lambda k: unvisited_dist[k])

        Q.remove(u)

        for v in G.neighbors_iter(u):
            alt = dist[u] + distance(u, v, G)
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u

    # build spf
    S = []
    u = t
    while prev[u]:
        S.append(u)
        u = prev[u]
    else:
        S.append(u)

    return list(reversed(S))



def get_weight(A, B):

    def calc_distance(lng1, lat1, lng2, lat2):
        '''
        '''
        lat1 = 2*math.pi*(90-lat1)/360.0
        lng1 = 2*math.pi*lng1/360.0
        lat2 = 2*math.pi*(90-lat2)/360.0
        lng2 = 2*math.pi*lng2/360.0

        con = math.sin(lat1)*math.sin(lat2)*math.cos(lng1-lng2)+math.cos(lat1)*math.cos(lat2)
        alpha = math.acos(con)

        return alpha * 6371004.0

    lng1, lat1 = A
    lng2, lat2 = B

    return calc_distance(lng1, lat1, lng2, lat2)


def get_graph(place_id, level):
    '''return graph object according to place_id and build level
    '''
    # change nodes and edges later for real data
    nodes = [
        (121.4335521, 31.1732386),
        (121.4335955, 31.1732467),
        (121.4336110, 31.1731824),
        (121.4336790, 31.1731894),
        (121.4337298, 31.1731985),
        (121.4337368, 31.1731691),
        (121.4338856, 31.1731953),
        (121.4339000, 31.1731632),
        (121.4336993, 31.1731407),
        (121.4340863, 31.1732311),
        (121.4340817, 31.1731980),
        (121.4341623, 31.1732097),
    ]

    edges = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 4),
        (4, 5),
        (5, 6),
        (6, 7),
        (6, 9),
        (7, 10),
        (9, 10),
        (10, 11),
        (3, 8),
        (7, 8),
    ]

    G=nx.Graph()
    weighted_edges = [(a, b, {'weight': get_weight(nodes[a], nodes[b])})
                      for a, b in edges]

    G.add_edges_from(weighted_edges)
    
    return G
