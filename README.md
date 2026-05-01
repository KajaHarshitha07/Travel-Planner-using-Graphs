# 🌍 Travel Planner using Graphs

A Python-based intelligent travel planning system that uses **graph data structures and algorithms** to compute optimal routes between cities based on distance, time, and cost.

---

## 🚀 Project Overview

This project models real-world locations as nodes and travel routes as weighted edges in a graph. It allows users to find the most efficient path between cities using advanced algorithms like **Dijkstra’s Algorithm** and **A* Search**.

It simulates real-world applications such as GPS navigation, logistics optimization, and travel itinerary planning.

---

## ✨ Features

* 📍 Graph-based representation of cities and routes
* ⚖️ Multi-weight edges (distance, time, cost)
* 🔍 Shortest path using Dijkstra’s Algorithm
* ⭐ Optimized path using A* Search (heuristic-based)
* 🚌 BFS for minimum stops
* 🧭 DFS for route exploration
* 🌐 Minimum Spanning Tree (Prim’s Algorithm)
* 🗺️ Find all possible paths (Backtracking)
* 💰 Budget-based route filtering
* 🚆 Multiple transport modes (road, train, flight, bus)
* 🧳 Multi-city trip planner
* 🖥️ Interactive Command Line Interface

---

## 🧠 Concepts Used

* Graphs (Adjacency List Representation)
* Priority Queue / Min Heap
* Greedy Algorithms
* Shortest Path Algorithms
* Backtracking
* Heuristic-based Search

---

## 🏗️ Project Structure

```
travel-planner-graphs/
│
├── src/
│   ├── graph.py          # Graph, City, Edge classes
│   ├── algorithms.py     # Dijkstra, A*, BFS, DFS, MST
│   ├── planner.py        # Path reconstruction & utilities
│   ├── data.py           # Sample dataset (India cities)
│
├── main.py               # CLI interface
├── README.md
├── requirements.txt
└── .gitignore
```

---

## ▶️ How to Run

1. Clone the repository:

```bash
git clone https://github.com/your-username/travel-planner-graphs.git
cd travel-planner-graphs
```

2. Run the project:

```bash
python main.py
```

---

## 📊 Example Use Case

* Find shortest route from **Delhi → Bangalore**
* Optimize based on:

  * Distance
  * Time
  * Cost

Output includes:

* Route path
* Total distance, time, and cost
* Travel modes used

---

## 🌍 Real-World Applications

* 🧭 GPS Navigation Systems
* 🚚 Logistics & Supply Chain Optimization
* ✈️ Travel & Tourism Planning
* 🏙️ Smart City Route Optimization

---

## 🔮 Future Enhancements

* Integration with APIs like Google Maps for real-time data
* Traffic and weather-based route optimization
* Web-based UI using Streamlit or Flask
* Map visualization using Folium


---

## ⭐ Conclusion

This project demonstrates the practical application of **Data Structures and Algorithms (DSA)** in solving real-world problems like route optimization, making it a strong addition to a software or data science portfolio.
