#import matplotlib.pyplot as plt
from collections import Counter
def euclidean_distance(point1, point2):
    return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5

def region_query(data, point, epsilon):
    neighbors = []
    for i, p in enumerate(data):
        if euclidean_distance(point, p) <= epsilon:
            neighbors.append(i)
    return neighbors

def expand_cluster(data, labels, point_index, cluster_id, epsilon, min_samples):
    seeds = [point_index]
    labels[point_index] = cluster_id

    while seeds:
        current_point_index = seeds.pop(0)
        current_point = data[current_point_index]
        neighbors = region_query(data, current_point, epsilon)

        if len(neighbors) >= min_samples:
            for neighbor_index in neighbors:
                if labels[neighbor_index] == -1 or labels[neighbor_index] == 0:
                    if labels[neighbor_index] == -1:
                        seeds.append(neighbor_index)
                    labels[neighbor_index] = cluster_id

def dbscan(data, epsilon, min_samples):
    n = len(data)
    labels = [0] * n  # 0 represents unclassified, -1 represents noise, and positive integers represent clusters
    cluster_id = 0

    for i in range(n):
        if labels[i] != 0:
            continue

        neighbors = region_query(data, data[i], epsilon)
        if len(neighbors) < min_samples:
            labels[i] = -1  # Mark as noise
        else:
            cluster_id += 1
            expand_cluster(data, labels, i, cluster_id, epsilon, min_samples)

    return labels

def priminent_cluster(epsilon,min_samples,data):

    #cluster data
    cluster_labels = dbscan(data, epsilon, min_samples)

    # Find the most prominent cluster
    cluster_counts = Counter(cluster_labels)
    #print(cluster_labels)
    most_prominent_cluster_label = max(cluster_counts, key=cluster_counts.get)

    # Filter data points belonging to the most prominent cluster
    most_prominent_cluster_data = [data[i] for i, label in enumerate(cluster_labels) if label == most_prominent_cluster_label]
    return most_prominent_cluster_data

