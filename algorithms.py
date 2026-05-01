import heapq

def dijkstra(graph, start, end, weight_type="distance"):
    index = {"distance": 0, "time": 1, "cost": 2}
    if weight_type not in index:
        raise ValueError("Optimize by must be one of: distance, time, cost")

    if start not in graph.graph or end not in graph.graph:
        return None, []

    pq = [(0, start, [])]
    visited = set()

    while pq:
        cost, node, path = heapq.heappop(pq)
        if node in visited:
            continue

        path = path + [node]
        visited.add(node)

        if node == end:
            return cost, path

        for neighbor, d, t, c in graph.graph[node]:
            weights = [d, t, c]
            heapq.heappush(pq, (cost + weights[index[weight_type]], neighbor, path))

    return None, []
