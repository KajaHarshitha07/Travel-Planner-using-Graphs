"""
╔══════════════════════════════════════════════════════════════════════════╗
║              🌍  TRAVEL PLANNER USING GRAPHS  🌍                        ║
║                                                                          ║
║  Project Scope:                                                          ║
║    ✔ Locations as graph nodes, routes as weighted edges                  ║
║    ✔ Multi-weight edges: distance (km), time (hrs), cost (₹)            ║
║    ✔ Dijkstra's — optimal path by any weight                             ║
║    ✔ A* Search — heuristic-guided fastest path                           ║
║    ✔ BFS — fewest stops                                                  ║
║    ✔ DFS — explore reachable cities                                      ║
║    ✔ Prim's MST — minimum road network                                   ║
║    ✔ All Paths — backtracking                                            ║
║    ✔ Budget & travel mode constraints                                     ║
║    ✔ Priority Queue / Min-Heap for efficiency                            ║
║    ✔ Adjacency List representation                                       ║
║    ✔ Interactive CLI with full menu                                      ║
╚══════════════════════════════════════════════════════════════════════════╝

 Real-World Applications:
    - GPS Navigation Systems
    - Logistics & Delivery Route Optimization
    - Travel Agency Trip Planning
"""

import heapq
import math
from collections import defaultdict, deque


# ══════════════════════════════════════════════════════
#  SECTION 1: GRAPH DATA STRUCTURE
# ══════════════════════════════════════════════════════

class Edge:
    """
    Represents a directed/undirected weighted edge.

    Weights:
        distance  — kilometers
        time      — hours (float)
        cost      — Indian Rupees (₹)
        mode      — 'road', 'train', 'flight', 'bus'
    """
    def __init__(self, to, distance, time, cost, mode="road"):
        self.to       = to
        self.distance = distance   # km
        self.time     = time       # hours
        self.cost     = cost       # ₹
        self.mode     = mode

    def weight(self, by="distance"):
        if by == "distance": return self.distance
        if by == "time":     return self.time
        if by == "cost":     return self.cost
        return self.distance


class City:
    """Node in the graph with geographic coordinates."""
    def __init__(self, name, lat, lon, state=""):
        self.name  = name
        self.lat   = lat    # latitude  (for A* heuristic)
        self.lon   = lon    # longitude (for A* heuristic)
        self.state = state

    def haversine(self, other):
        """
        Haversine formula — straight-line distance between two cities.
        Used as heuristic in A* algorithm.
        """
        R = 6371  # Earth radius in km
        lat1, lon1 = math.radians(self.lat),  math.radians(self.lon)
        lat2, lon2 = math.radians(other.lat), math.radians(other.lon)
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
        return R * 2 * math.asin(math.sqrt(a))

    def __repr__(self):
        return f"City({self.name})"


class TravelGraph:
    """
    Weighted, directed graph using Adjacency List.

    Supports:
        - Multiple edge weights (distance, time, cost)
        - Directed and undirected edges
        - Mode-of-transport filtering
        - Budget constraints
    """

    def __init__(self, directed=False):
        self.cities   = {}              # name → City object
        self.adj      = defaultdict(list)  # name → [Edge, ...]
        self.directed = directed

    # ── Graph Construction ────────────────────────────────

    def add_city(self, name, lat=0.0, lon=0.0, state=""):
        self.cities[name] = City(name, lat, lon, state)

    def add_route(self, a, b, distance, time, cost, mode="road"):
        """Add a route (edge) between cities a and b."""
        self.adj[a].append(Edge(b, distance, time, cost, mode))
        if not self.directed:
            self.adj[b].append(Edge(a, distance, time, cost, mode))

    def get_edges(self, city, mode_filter=None, max_cost=None):
        """Get edges with optional mode and budget filtering."""
        edges = self.adj[city]
        if mode_filter:
            edges = [e for e in edges if e.mode == mode_filter]
        if max_cost is not None:
            edges = [e for e in edges if e.cost <= max_cost]
        return edges

    # ── Display ───────────────────────────────────────────

    def display_map(self, mode_filter=None):
        print("\n" + "═"*65)
        print("  📍  TRAVEL MAP — Available Routes")
        if mode_filter:
            print(f"  Mode filter: {mode_filter}")
        print("═"*65)
        ICONS = {"road": "🚗", "train": "🚂", "flight": "✈️", "bus": "🚌"}
        seen = set()
        for city in sorted(self.cities):
            for e in self.adj[city]:
                key = tuple(sorted([city, e.to])) + (e.mode,)
                if key not in seen:
                    if mode_filter and e.mode != mode_filter:
                        continue
                    icon = ICONS.get(e.mode, "🛣️")
                    print(f"  {icon} {city:16} → {e.to:16} "
                          f"| {e.distance:>5}km "
                          f"| {e.time:>4.1f}h "
                          f"| ₹{e.cost:>5}")
                    seen.add(key)
        print("═"*65)

    def list_cities(self):
        cities = sorted(self.cities.keys())
        print("\n  🏙️  Available Cities:")
        for i, c in enumerate(cities, 1):
            state = self.cities[c].state
            print(f"  {i:>2}. {c}{f'  ({state})' if state else ''}")
        return cities


# ══════════════════════════════════════════════════════
#  SECTION 2: DIJKSTRA'S ALGORITHM
# ══════════════════════════════════════════════════════

def dijkstra(graph, source, by="distance", mode_filter=None, budget=None):
    """
    Dijkstra's Shortest Path Algorithm.

    Finds the optimal (minimum weight) path from source to all cities.

    Parameters:
        by           — 'distance' | 'time' | 'cost'
        mode_filter  — restrict to specific transport mode
        budget       — max total cost allowed (₹)

    Returns:
        dist      — { city: min_weight }
        prev      — { city: previous_city } for path reconstruction
        cost_spent— { city: total_cost_spent } (for budget tracking)

    Time:  O((V + E) log V)  using min-heap
    Space: O(V + E)
    """
    INF = float('inf')
    dist       = {c: INF for c in graph.cities}
    cost_spent = {c: 0   for c in graph.cities}
    prev       = {c: None for c in graph.cities}
    dist[source] = 0

    # Min-heap: (weight, city)
    heap = [(0, source)]

    while heap:
        curr_w, curr = heapq.heappop(heap)

        if curr_w > dist[curr]:
            continue   # stale entry

        for edge in graph.get_edges(curr, mode_filter):
            new_cost = cost_spent[curr] + edge.cost
            if budget is not None and new_cost > budget:
                continue    # constraint: over budget

            new_w = curr_w + edge.weight(by)
            if new_w < dist[edge.to]:
                dist[edge.to]       = new_w
                cost_spent[edge.to] = new_cost
                prev[edge.to]       = curr
                heapq.heappush(heap, (new_w, edge.to))

    return dist, prev, cost_spent


def reconstruct_path(prev, source, dest):
    """Rebuild the path from source to dest using prev dict."""
    path, curr = [], dest
    while curr:
        path.append(curr)
        curr = prev[curr]
    path.reverse()
    return path if path and path[0] == source else []


def path_summary(graph, path, by="distance"):
    """Compute total distance, time, and cost for a path."""
    total = {"distance": 0, "time": 0, "cost": 0}
    modes = []
    for i in range(len(path) - 1):
        a, b = path[i], path[i+1]
        for edge in graph.adj[a]:
            if edge.to == b:
                total["distance"] += edge.distance
                total["time"]     += edge.time
                total["cost"]     += edge.cost
                modes.append(edge.mode)
                break
    return total, modes


def find_shortest_path(graph, src, dst, by="distance", mode_filter=None, budget=None):
    """User-facing function: find and display shortest path."""
    labels = {"distance": "Distance (km)", "time": "Time (hrs)", "cost": "Cost (₹)"}
    print(f"\n{'═'*60}")
    print(f"  🔍  SHORTEST PATH  ({labels[by]})")
    print(f"  From: {src}  →  To: {dst}")
    if mode_filter: print(f"  Mode: {mode_filter}")
    if budget:      print(f"  Budget: ₹{budget}")
    print("─"*60)

    if src not in graph.cities or dst not in graph.cities:
        print("  ❌  One or both cities not found."); return

    dist, prev, cost_spent = dijkstra(graph, src, by, mode_filter, budget)

    if dist[dst] == float('inf'):
        print(f"  ❌  No feasible route from {src} to {dst}.")
        if budget:
            print(f"     Tip: Try increasing your budget.")
        return

    path = reconstruct_path(prev, src, dst)
    totals, modes = path_summary(graph, path, by)

    print(f"  Route    : {' → '.join(path)}")
    print(f"  Stops    : {len(path)-1}")
    print(f"  Distance : {totals['distance']} km")
    print(f"  Time     : {totals['time']:.1f} hrs")
    print(f"  Cost     : ₹{totals['cost']}")
    print(f"  Modes    : {', '.join(set(modes))}")
    print("═"*60)


# ══════════════════════════════════════════════════════
#  SECTION 3: A* ALGORITHM
# ══════════════════════════════════════════════════════

def astar(graph, source, destination, by="distance"):
    """
    A* Search Algorithm — heuristic-guided shortest path.

    Heuristic: Haversine straight-line distance between cities.
    A* expands fewer nodes than Dijkstra's when a good heuristic exists.

    f(n) = g(n) + h(n)
        g(n) = actual cost from source to n
        h(n) = estimated cost from n to destination

    Time:  O(E log V) — often faster than Dijkstra in practice
    Space: O(V)
    """
    INF = float('inf')
    g  = {c: INF  for c in graph.cities}
    prev = {c: None for c in graph.cities}
    g[source] = 0

    dest_city = graph.cities.get(destination)

    def h(city):
        """Heuristic: straight-line km to destination."""
        if not dest_city or city not in graph.cities:
            return 0
        return graph.cities[city].haversine(dest_city)

    # heap: (f_score, city)
    open_heap = [(h(source), source)]
    open_set  = {source}

    while open_heap:
        _, curr = heapq.heappop(open_heap)
        open_set.discard(curr)

        if curr == destination:
            break

        for edge in graph.get_edges(curr):
            new_g = g[curr] + edge.weight(by)
            if new_g < g[edge.to]:
                g[edge.to]    = new_g
                prev[edge.to] = curr
                f = new_g + h(edge.to)
                if edge.to not in open_set:
                    heapq.heappush(open_heap, (f, edge.to))
                    open_set.add(edge.to)

    return g, prev


def find_astar_path(graph, src, dst, by="distance"):
    """User-facing A* path finder."""
    print(f"\n{'═'*60}")
    print(f"  ⭐  A* SHORTEST PATH")
    print(f"  From: {src}  →  To: {dst}")
    print("─"*60)

    if src not in graph.cities or dst not in graph.cities:
        print("  ❌  One or both cities not found."); return

    g, prev = astar(graph, src, dst, by)

    if g[dst] == float('inf'):
        print(f"  ❌  No route found."); return

    path = reconstruct_path(prev, src, dst)
    totals, modes = path_summary(graph, path, by)

    print(f"  Route    : {' → '.join(path)}")
    print(f"  Distance : {totals['distance']} km")
    print(f"  Time     : {totals['time']:.1f} hrs")
    print(f"  Cost     : ₹{totals['cost']}")
    print("═"*60)


# ══════════════════════════════════════════════════════
#  SECTION 4: BFS — FEWEST STOPS
# ══════════════════════════════════════════════════════

def bfs(graph, source, destination, mode_filter=None):
    """
    Breadth-First Search — finds path with fewest stops (hops).
    Does NOT consider weights; purely counts edges.

    Time:  O(V + E)
    Space: O(V)
    """
    print(f"\n{'═'*60}")
    print(f"  🚌  FEWEST STOPS PATH (BFS)")
    print(f"  From: {source}  →  To: {destination}")
    print("─"*60)

    if source not in graph.cities or destination not in graph.cities:
        print("  ❌  One or both cities not found."); return

    visited = {source}
    queue   = deque([[source]])   # queue of paths

    while queue:
        path = queue.popleft()
        node = path[-1]

        if node == destination:
            totals, modes = path_summary(graph, path)
            print(f"  Route    : {' → '.join(path)}")
            print(f"  Stops    : {len(path)-1}")
            print(f"  Distance : {totals['distance']} km")
            print(f"  Time     : {totals['time']:.1f} hrs")
            print(f"  Cost     : ₹{totals['cost']}")
            print("═"*60)
            return

        for edge in graph.get_edges(node, mode_filter):
            if edge.to not in visited:
                visited.add(edge.to)
                queue.append(path + [edge.to])

    print(f"  ❌  No route found from {source} to {destination}.")
    print("═"*60)


# ══════════════════════════════════════════════════════
#  SECTION 5: DFS — EXPLORE
# ══════════════════════════════════════════════════════

def dfs(graph, source, destination):
    """
    Depth-First Search — explores deep paths first.
    Finds *a* path (not necessarily shortest).

    Time:  O(V + E)
    Space: O(V)
    """
    print(f"\n{'═'*60}")
    print(f"  🧭  DFS PATH")
    print(f"  From: {source}  →  To: {destination}")
    print("─"*60)

    if source not in graph.cities or destination not in graph.cities:
        print("  ❌  One or both cities not found."); return

    visited = set()
    stack   = [[source]]

    while stack:
        path = stack.pop()
        node = path[-1]

        if node == destination:
            totals, _ = path_summary(graph, path)
            print(f"  Route    : {' → '.join(path)}")
            print(f"  Stops    : {len(path)-1}")
            print(f"  Distance : {totals['distance']} km")
            print(f"  Cost     : ₹{totals['cost']}")
            print("═"*60)
            return

        if node not in visited:
            visited.add(node)
            for edge in graph.get_edges(node):
                if edge.to not in visited:
                    stack.append(path + [edge.to])

    print(f"  ❌  No route found.")
    print("═"*60)


# ══════════════════════════════════════════════════════
#  SECTION 6: ALL PATHS — BACKTRACKING
# ══════════════════════════════════════════════════════

def find_all_paths(graph, source, destination, max_paths=8, max_cost=None):
    """
    Find ALL paths from source to destination using backtracking DFS.
    Results are sorted by total distance.

    Parameters:
        max_paths — limit results for readability
        max_cost  — optional budget constraint

    Time:  O(V!) worst case (all permutations)
    Space: O(V)
    """
    print(f"\n{'═'*60}")
    print(f"  🗺️   ALL POSSIBLE PATHS (Backtracking)")
    print(f"  From: {source}  →  To: {destination}")
    if max_cost: print(f"  Budget: ₹{max_cost}")
    print("─"*60)

    if source not in graph.cities or destination not in graph.cities:
        print("  ❌  One or both cities not found."); return

    results = []

    def backtrack(curr, path, visited, curr_cost):
        if curr == destination:
            totals, _ = path_summary(graph, path)
            if max_cost is None or totals["cost"] <= max_cost:
                results.append((totals["distance"], list(path), totals))
            return
        if len(results) >= max_paths:
            return
        for edge in graph.get_edges(curr):
            if edge.to not in visited:
                if max_cost and curr_cost + edge.cost > max_cost:
                    continue
                visited.add(edge.to)
                path.append(edge.to)
                backtrack(edge.to, path, visited, curr_cost + edge.cost)
                path.pop()
                visited.remove(edge.to)

    backtrack(source, [source], {source}, 0)

    if not results:
        print(f"  ❌  No paths found.")
        print("═"*60)
        return

    results.sort()   # sort by distance
    print(f"  Found {len(results)} path(s), sorted by distance:\n")
    for i, (_, path, t) in enumerate(results, 1):
        print(f"  [{i}] {' → '.join(path)}")
        print(f"       {t['distance']}km | {t['time']:.1f}h | ₹{t['cost']}\n")
    print("═"*60)


# ══════════════════════════════════════════════════════
#  SECTION 7: PRIM'S MST
# ══════════════════════════════════════════════════════

def prims_mst(graph, by="distance"):
    """
    Prim's Algorithm — Minimum Spanning Tree.

    Finds the minimum total network to connect ALL cities.
    Real-world use: planning road/rail networks with minimal cost.

    Time:  O(E log V)
    Space: O(V + E)
    """
    print(f"\n{'═'*60}")
    print(f"  🌐  MINIMUM SPANNING TREE (Prim's)")
    print(f"  Weight: {by}")
    print("─"*60)

    if not graph.cities:
        print("  ❌  No cities in graph."); return

    start   = next(iter(graph.cities))
    visited = {start}
    heap    = [(e.weight(by), start, e.to, e) for e in graph.adj[start]]
    heapq.heapify(heap)

    mst_edges = []
    total     = 0

    while heap and len(visited) < len(graph.cities):
        w, frm, to, edge = heapq.heappop(heap)
        if to in visited:
            continue
        visited.add(to)
        mst_edges.append((frm, to, edge))
        total += w
        for e in graph.adj[to]:
            if e.to not in visited:
                heapq.heappush(heap, (e.weight(by), to, e.to, e))

    print(f"  MST Edges ({len(mst_edges)} connections):\n")
    for frm, to, e in mst_edges:
        print(f"  {frm:16} → {to:16} | {e.distance}km | {e.time:.1f}h | ₹{e.cost}")

    print(f"\n  Total {by}: {total:.1f}")
    print("═"*60)


# ══════════════════════════════════════════════════════
#  SECTION 8: MULTI-CITY TRIP PLANNER
# ══════════════════════════════════════════════════════

def plan_multi_city_trip(graph, cities, by="distance"):
    """
    Plan a multi-city trip visiting all given cities in order.
    Finds optimal leg-by-leg routes using Dijkstra's.
    """
    print(f"\n{'═'*60}")
    print(f"  🧳  MULTI-CITY TRIP PLANNER")
    print(f"  Cities: {' → '.join(cities)}")
    print("─"*60)

    grand_total = {"distance": 0, "time": 0, "cost": 0}
    full_route  = [cities[0]]

    for i in range(len(cities) - 1):
        src, dst = cities[i], cities[i+1]
        dist, prev, _ = dijkstra(graph, src, by=by)

        if dist[dst] == float('inf'):
            print(f"  ❌  No route from {src} to {dst}!")
            return

        path = reconstruct_path(prev, src, dst)
        totals, modes = path_summary(graph, path, by)

        full_route.extend(path[1:])
        print(f"  Leg {i+1}: {src} → {dst}")
        print(f"    {' → '.join(path)}")
        print(f"    {totals['distance']}km | {totals['time']:.1f}h | ₹{totals['cost']}")
        print(f"    Mode: {', '.join(set(modes))}\n")

        for k in grand_total:
            grand_total[k] += totals[k]

    print(f"  ─── Grand Total ───────────────────────────────")
    print(f"  Distance: {grand_total['distance']} km")
    print(f"  Time    : {grand_total['time']:.1f} hrs")
    print(f"  Cost    : ₹{grand_total['cost']}")
    print("═"*60)


# ══════════════════════════════════════════════════════
#  SECTION 9: SAMPLE DATA — INDIA TRAVEL MAP
# ══════════════════════════════════════════════════════

def build_india_graph():
    """
    Build a realistic Indian travel graph with:
    - Major cities as nodes (with GPS coordinates)
    - Road, Train, Bus, and Flight connections
    - Multiple weights: distance, travel time, cost
    """
    g = TravelGraph(directed=False)

    # ── Add Cities (with lat/lon for A* heuristic) ──────
    cities = [
        ("Delhi",       28.61, 77.20, "Delhi"),
        ("Agra",        27.17, 78.01, "UP"),
        ("Jaipur",      26.91, 75.78, "Rajasthan"),
        ("Lucknow",     26.84, 80.94, "UP"),
        ("Varanasi",    25.31, 83.00, "UP"),
        ("Patna",       25.59, 85.13, "Bihar"),
        ("Kolkata",     22.56, 88.36, "WB"),
        ("Bhubaneswar", 20.29, 85.82, "Odisha"),
        ("Mumbai",      19.07, 72.87, "Maharashtra"),
        ("Pune",        18.52, 73.85, "Maharashtra"),
        ("Ahmedabad",   23.02, 72.57, "Gujarat"),
        ("Hyderabad",   17.38, 78.48, "Telangana"),
        ("Bangalore",   12.97, 77.59, "Karnataka"),
        ("Chennai",     13.08, 80.27, "Tamil Nadu"),
        ("Chandigarh",  30.73, 76.78, "Punjab"),
        ("Amritsar",    31.63, 74.87, "Punjab"),
    ]
    for name, lat, lon, state in cities:
        g.add_city(name, lat, lon, state)

    # ── Add Routes ──────────────────────────────────────
    # Format: (from, to, dist_km, time_hrs, cost_₹, mode)
    routes = [
        # Road routes
        ("Delhi",      "Agra",         200, 3.0,  500,  "road"),
        ("Delhi",      "Jaipur",       270, 4.5,  650,  "road"),
        ("Delhi",      "Chandigarh",   250, 4.0,  600,  "road"),
        ("Agra",       "Jaipur",       240, 4.0,  580,  "road"),
        ("Agra",       "Lucknow",      330, 5.5,  750,  "road"),
        ("Jaipur",     "Ahmedabad",    660, 11.0, 1400, "road"),
        ("Lucknow",    "Varanasi",     320, 6.0,  700,  "road"),
        ("Lucknow",    "Patna",        530, 9.0,  1100, "road"),
        ("Varanasi",   "Patna",        280, 5.0,  600,  "road"),
        ("Patna",      "Kolkata",      600, 10.0, 1200, "road"),
        ("Kolkata",    "Bhubaneswar",  440, 8.0,  900,  "road"),
        ("Mumbai",     "Pune",         150, 3.0,  350,  "road"),
        ("Mumbai",     "Ahmedabad",    525, 8.5,  1100, "road"),
        ("Mumbai",     "Hyderabad",    710, 12.0, 1500, "road"),
        ("Pune",       "Hyderabad",    560, 9.5,  1200, "road"),
        ("Hyderabad",  "Bangalore",    570, 10.0, 1300, "road"),
        ("Hyderabad",  "Chennai",      630, 10.5, 1350, "road"),
        ("Bangalore",  "Chennai",      340, 6.0,  800,  "road"),
        ("Chandigarh", "Amritsar",     230, 4.0,  500,  "road"),
        ("Delhi",      "Amritsar",     450, 7.5,  900,  "road"),

        # Train routes (faster, cheaper than road for long distances)
        ("Delhi",      "Mumbai",       1400, 16.0, 800,  "train"),
        ("Delhi",      "Kolkata",      1450, 17.0, 850,  "train"),
        ("Delhi",      "Chennai",      2200, 28.0, 1100, "train"),
        ("Delhi",      "Bangalore",    2150, 34.0, 1050, "train"),
        ("Mumbai",     "Chennai",      1330, 22.0, 700,  "train"),
        ("Mumbai",     "Bangalore",    980,  15.0, 600,  "train"),
        ("Kolkata",    "Chennai",      1660, 26.0, 900,  "train"),
        ("Hyderabad",  "Mumbai",       710,  12.0, 500,  "train"),
        ("Delhi",      "Lucknow",      520,  7.0,  350,  "train"),
        ("Delhi",      "Varanasi",     830,  12.0, 500,  "train"),
        ("Patna",      "Kolkata",      600,  8.0,  400,  "train"),

        # Flight routes (fastest, most expensive)
        ("Delhi",      "Mumbai",       1400, 2.0,  4500, "flight"),
        ("Delhi",      "Kolkata",      1450, 2.0,  4800, "flight"),
        ("Delhi",      "Bangalore",    2150, 2.5,  5500, "flight"),
        ("Delhi",      "Chennai",      2200, 2.5,  5200, "flight"),
        ("Delhi",      "Hyderabad",    1570, 2.0,  4600, "flight"),
        ("Mumbai",     "Kolkata",      2050, 2.5,  5000, "flight"),
        ("Mumbai",     "Bangalore",    980,  1.5,  3800, "flight"),
        ("Mumbai",     "Chennai",      1330, 2.0,  4200, "flight"),
        ("Bangalore",  "Kolkata",      1870, 2.5,  5200, "flight"),
        ("Chennai",    "Kolkata",      1660, 2.0,  4900, "flight"),
        ("Hyderabad",  "Kolkata",      1500, 2.0,  4700, "flight"),

        # Bus routes (cheapest)
        ("Delhi",      "Agra",         200,  4.0,  250,  "bus"),
        ("Delhi",      "Jaipur",       270,  5.0,  300,  "bus"),
        ("Delhi",      "Chandigarh",   250,  5.0,  280,  "bus"),
        ("Mumbai",     "Pune",         150,  4.0,  180,  "bus"),
        ("Bangalore",  "Hyderabad",    570,  11.0, 600,  "bus"),
        ("Chennai",    "Bangalore",    340,  7.0,  400,  "bus"),
    ]

    for r in routes:
        g.add_route(*r)

    return g


# ══════════════════════════════════════════════════════
#  SECTION 10: INTERACTIVE CLI MENU
# ══════════════════════════════════════════════════════

def get_valid_city(prompt, graph):
    while True:
        city = input(prompt).strip().title()
        if city in graph.cities:
            return city
        print(f"  ⚠️  Not found. Type one of: {', '.join(sorted(graph.cities.keys())[:8])}...")


def get_weight_choice():
    print("  Optimize by: (1) Distance  (2) Time  (3) Cost")
    choice = input("  Enter choice [1]: ").strip() or "1"
    return {"1": "distance", "2": "time", "3": "cost"}.get(choice, "distance")


def get_mode_choice():
    print("  Mode: (1) Any  (2) Road  (3) Train  (4) Flight  (5) Bus")
    choice = input("  Enter choice [1]: ").strip() or "1"
    return {"2": "road", "3": "train", "4": "flight", "5": "bus"}.get(choice, None)


def main():
    print("""
╔══════════════════════════════════════════════════════════════╗
║          🌍  TRAVEL PLANNER USING GRAPHS  🌍                 ║
║    Algorithms: Dijkstra · A* · BFS · DFS · MST · Backtrack   ║
╚══════════════════════════════════════════════════════════════╝
    """)

    graph = build_india_graph()

    menu = """
┌─────────────────────────────────────────────────────────────┐
│  MENU                                                       │
│  1.  📍  View Travel Map                                    │
│  2.  🏙️   List Cities                                       │
│  3.  🔍  Shortest Path            (Dijkstra's)              │
│  4.  ⭐  A* Shortest Path         (A* Search)               │
│  5.  🚌  Fewest Stops Path        (BFS)                     │
│  6.  🧭  DFS Path                 (DFS)                     │
│  7.  🛤️   All Possible Paths      (Backtracking)            │
│  8.  🌐  Minimum Spanning Tree    (Prim's)                  │
│  9.  📊  All Distances from City  (Dijkstra's)              │
│  10. 🧳  Multi-City Trip Planner                            │
│  11. ➕  Add Custom Route                                   │
│  0.  ❌  Exit                                               │
└─────────────────────────────────────────────────────────────┘"""

    while True:
        print(menu)
        choice = input("  Enter your choice: ").strip()

        if choice == "1":
            mode = get_mode_choice()
            graph.display_map(mode_filter=mode)

        elif choice == "2":
            graph.list_cities()

        elif choice == "3":
            src = get_valid_city("  Source city      : ", graph)
            dst = get_valid_city("  Destination city : ", graph)
            by  = get_weight_choice()
            mode = get_mode_choice()
            bgt  = input("  Budget in ₹ (or press Enter for no limit): ").strip()
            budget = int(bgt) if bgt.isdigit() else None
            find_shortest_path(graph, src, dst, by, mode, budget)

        elif choice == "4":
            src = get_valid_city("  Source city      : ", graph)
            dst = get_valid_city("  Destination city : ", graph)
            by  = get_weight_choice()
            find_astar_path(graph, src, dst, by)

        elif choice == "5":
            src = get_valid_city("  Source city      : ", graph)
            dst = get_valid_city("  Destination city : ", graph)
            mode = get_mode_choice()
            bfs(graph, src, dst, mode)

        elif choice == "6":
            src = get_valid_city("  Source city      : ", graph)
            dst = get_valid_city("  Destination city : ", graph)
            dfs(graph, src, dst)

        elif choice == "7":
            src = get_valid_city("  Source city      : ", graph)
            dst = get_valid_city("  Destination city : ", graph)
            bgt = input("  Budget in ₹ (or Enter for no limit): ").strip()
            budget = int(bgt) if bgt.isdigit() else None
            find_all_paths(graph, src, dst, max_cost=budget)

        elif choice == "8":
            by = get_weight_choice()
            prims_mst(graph, by)

        elif choice == "9":
            src = get_valid_city("  Source city : ", graph)
            by  = get_weight_choice()
            dist, prev, _ = dijkstra(graph, src, by)
            print(f"\n{'═'*60}")
            print(f"  📊  ALL SHORTEST {by.upper()} FROM: {src}")
            print("─"*60)
            for city in sorted(dist):
                if city == src: continue
                d = dist[city]
                if d == float('inf'):
                    print(f"  {city:20} → Unreachable")
                else:
                    path = reconstruct_path(prev, src, city)
                    unit = {"distance": "km", "time": "hrs", "cost": "₹"}[by]
                    val  = f"{d:.1f}" if by == "time" else f"{int(d)}"
                    print(f"  {city:20} → {val:>6} {unit}  ({' → '.join(path)})")
            print("═"*60)

        elif choice == "10":
            print("  Enter cities to visit in order (comma separated):")
            raw = input("  e.g. Delhi, Agra, Jaipur: ").strip()
            cities = [c.strip().title() for c in raw.split(",")]
            invalid = [c for c in cities if c not in graph.cities]
            if invalid:
                print(f"  ❌  Not found: {', '.join(invalid)}")
            elif len(cities) < 2:
                print("  ❌  Please enter at least 2 cities.")
            else:
                by = get_weight_choice()
                plan_multi_city_trip(graph, cities, by)

        elif choice == "11":
            print("\n  ➕  ADD CUSTOM ROUTE")
            a    = input("  City A       : ").strip().title()
            b    = input("  City B       : ").strip().title()
            try:
                dist = float(input("  Distance (km): "))
                time = float(input("  Time (hrs)   : "))
                cost = float(input("  Cost (₹)     : "))
                print("  Mode: (1) Road  (2) Train  (3) Flight  (4) Bus")
                m = {"1":"road","2":"train","3":"flight","4":"bus"}.get(input("  ").strip(), "road")
                # Add cities if they don't exist
                if a not in graph.cities: graph.add_city(a)
                if b not in graph.cities: graph.add_city(b)
                graph.add_route(a, b, dist, time, cost, m)
                print(f"  ✅  Route added: {a} ↔ {b} [{dist}km | {time}h | ₹{cost} | {m}]")
            except ValueError:
                print("  ❌  Invalid input.")

        elif choice == "0":
            print("\n  👋  Thank you for using Travel Planner. Safe travels!\n")
            break
        else:
            print("  ⚠️  Invalid choice. Please try again.")

        input("\n  Press Enter to continue...")


if __name__ == "__main__":
    main()