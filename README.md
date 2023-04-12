# PRM-
무작위 경로와 장애물을 피해 목표를 찾는 코드
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from scipy.spatial.distance import euclidean

def sample_free_space(space_size, num_samples, obstacles):
    samples = []
    while len(samples) < num_samples:
        point = np.random.uniform(0, space_size, 2)
        if not is_colliding(point, obstacles):
            samples.append(tuple(point)) # Convert to tuple
    return samples

def is_colliding(point, obstacles):
    for obs in obstacles:
        if euclidean(point, obs[0]) < obs[1]:
            return True
    return False


def connect_nodes(nodes, max_distance, obstacles):
    edges = []
    for i, n1 in enumerate(nodes):
        for n2 in nodes[i+1:]:
            n1_array, n2_array = np.array(n1), np.array(n2)
            if euclidean(n1_array, n2_array) <= max_distance:   
                mid_point = (n1_array + n2_array) / 2
                if not is_colliding(mid_point, obstacles):
                    edges.append((n1, n2))
    return edges

####################

def prm(space_size, num_samples, max_distance, start, goal, obstacles):
    if is_colliding(start, obstacles) or is_colliding(goal, obstacles):
        print("Start or goal position is in collision")
        return None
    
    nodes = sample_free_space(space_size, num_samples, obstacles)
    nodes = [tuple(start)] + nodes + [tuple(goal)] # Convert to tuples
    edges = connect_nodes(nodes, max_distance, obstacles)

    graph = nx.Graph()
    graph.add_nodes_from(nodes)
    graph.add_edges_from(edges)

    if nx.has_path(graph, tuple(start), tuple(goal)):
        path = nx.shortest_path(graph, tuple(start), tuple(goal), weight="distance")
        return path
    else:
        print("No path found")
        return None
    
#########################

def plot_prm(start, goal, space_size, obstacles, nodes, edges, path):
    fig, ax = plt.subplots()
    
    for obs in obstacles:
        circle = plt.Circle(obs[0], obs[1], color="gray", alpha = 0.5)
        ax.add_artist (circle)
    for node in nodes:
        plt.scatter(*node, c='b', s=10)
    for edge in edges:
        plt.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]], 'k-', lw=0.5)

    ax.plot(start[0], start[1], 'bo', markersize=10, label="Start")
    ax.plot(goal[0], goal[1], 'mo', markersize=10, label="Target")

    if path:
        plt.plot(*zip(*path), 'go-', lw=2)

    plt.xlim()
    plt.ylim()
    plt.legend(loc='upper left')
    plt.show()

if __name__ == "__main__":

    space_size = 100
    num_samples = 100
    max_distance = 20
    start = np.array([5, 5])
    goal = np.array([95, 95])
    obstacles = [(np.array([50, 50]), 10), (np.array([30, 70]), 5), (np.array([70, 30]), 5)]


    path = prm(space_size, num_samples, max_distance, start, goal, obstacles)
    if path:
        print("Path found!: ", path)

    nodes = sample_free_space(space_size, num_samples, obstacles)
    nodes = [start] + nodes + [goal]
    edges = connect_nodes(nodes, max_distance, obstacles)
    plot_prm(start, goal, space_size, obstacles, nodes, edges, path)
