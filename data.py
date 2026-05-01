try:
    from .graph import TravelGraph
except ImportError:
    import os
    import sys

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from graph import TravelGraph


def load_data():
    g = TravelGraph()
    g.add_edge("Guntur", "Vijayawada", 35, 1, 100)
    g.add_edge("Vijayawada", "Hyderabad", 275, 5, 800)
    g.add_edge("Guntur", "Hyderabad", 300, 6, 900)
    return g
