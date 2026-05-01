from src.data import load_data
from src.algorithms import dijkstra

def main():
    g = load_data()
    start = input("Enter source: ").strip()
    end = input("Enter destination: ").strip()
    pref = input("Optimize by (distance/time/cost): ").strip().lower()

    try:
        cost, path = dijkstra(g, start, end, pref)
    except ValueError as exc:
        print(exc)
        return

    if cost is None or not path:
        print(f"No path found from {start} to {end}.")
        return

    print("Path:", " -> ".join(path))
    print("Total:", cost)

if __name__ == "__main__":
    main()
