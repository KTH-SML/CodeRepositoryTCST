#!/usr/bin/env python
import networkx as nx
import matplotlib.pyplot as plt
import pylab

def plot_automaton(automaton):
        options = {
            'node_color': 'blue',
            'node_size': 1500,
            'width': 3,
            'arrowstyle': '-|>',
            'arrowsize': 1,
        }
        pos = nx.circular_layout(automaton)
        nx.draw_networkx(automaton, pos, arrows=True, **options)
        pylab.show()
