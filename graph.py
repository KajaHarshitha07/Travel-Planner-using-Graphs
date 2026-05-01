class Edge:
    def __init__(self, to, distance, time, cost, mode="road"):
        self.to = to
        self.distance = distance
        self.time = time
        self.cost = cost
        self.mode = mode

    def weight(self, by="distance"):
        return getattr(self, by, self.distance)


class City:
    def __init__(self, name, lat=0.0, lon=0.0):
        self.name = name
        self.lat = lat
        self.lon = lon


class TravelGraph:
    def __init__(self):
        self.graph = {}

    def add_edge(self, u, v, distance, time, cost):
        self.graph.setdefault(u, []).append((v, distance, time, cost))
        self.graph.setdefault(v, []).append((u, distance, time, cost))
