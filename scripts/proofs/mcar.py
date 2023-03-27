import open3d as o3d
import numpy as np
import random

from scipy.spatial.distance import euclidean
from scipy.sparse.linalg import eigsh
from scipy.sparse import csr_matrix

import matplotlib.pyplot as plt

cloud= o3d.geometry.PointCloud()
line_set = o3d.geometry.LineSet()
radius = 0.1
mult = 1

fig, axs = plt.subplots(1, 3, figsize=(15, 6))

def genereateRandomCloud():
    global cloud
    points = [[.0, .0, .0]]

    for i in range(1, 100):
        random_idx = random.randint(0, len(points) - 1)

        random_x = random.uniform(-0.55, 0.55)
        random_y = random.uniform(-0.55, 0.55)
        random_z = random.uniform(-0.55, 0.55)

        current_point = points[random_idx]
        new_x = current_point[0] + random_x
        new_y = current_point[1] + random_y
        new_z = current_point[2] + random_z

        points.append([new_x, new_y, new_z])


    # TODO
    # 1) 1st point at (0,0,0)
    # 2) generate random number and add it ot 

    np_points = np.asarray(points)
    cloud.points = o3d.utility.Vector3dVector(np_points)

    colors = np.zeros((len(cloud.points), 3))
    colors[:] = [0, 0, 0]

    color = o3d.utility.Vector3dVector(colors)

    cloud.colors = color
    return cloud

def display_mcar_hist():
    global cloud

    r = 0.0
    rows, cols = 100, 100
    laplacian = csr_matrix((rows, cols), dtype=np.float32)
    tree = o3d.geometry.KDTreeFlann(cloud)
    for q_idx, query_point in enumerate(cloud.points):
        [k, idx, _] = tree.search_knn_vector_3d(query_point, 2)
        point = cloud.points[idx[1]]
        dist = euclidean(point, query_point)
        if dist > r:
            r = dist

    for q_idx, query_point in enumerate(cloud.points):
        [k, idx, _] = tree.search_radius_vector_3d(query_point, r)
        for i in idx:
            if i == q_idx:
                laplacian[q_idx, q_idx] = len(idx) - 1
            else:
                laplacian[q_idx, i] = -1
                laplacian[i, q_idx] = -1



    eigenvalues, _ = eigsh(laplacian, k=laplacian.shape[0]-1, which='SM')
    plt.ion()
    axs[1].hist(eigenvalues.real, bins=20)
    axs[1].set_title('MCAR Spectral Features radius: ' + str(round(r, 4)))
    plt.draw()
    plt.show(block=False)
    plt.pause(0.001)

def display_fc_hist():
    global cloud

    r = 100
    rows, cols = 100, 100
    laplacian = csr_matrix((rows, cols), dtype=np.float32)
    tree = o3d.geometry.KDTreeFlann(cloud)
    for q_idx, query_point in enumerate(cloud.points):
        [k, idx, _] = tree.search_radius_vector_3d(query_point, r)
        if k != 100:
            print('ERROR radius too small')
            print(k)
            exit(1)
        for i in idx:
            if i == q_idx:
                laplacian[q_idx, q_idx] = len(idx) - 1
            else:
                laplacian[q_idx, i] = -1
                laplacian[i, q_idx] = -1



    eigenvalues, _ = eigsh(laplacian, k=laplacian.shape[0]-1, which='SM')
    plt.ion()
    axs[2].hist(eigenvalues.real, bins=20)
    axs[2].set_title('Fully Connected Graph')
    plt.draw()
    plt.show(block=False)
    plt.pause(0.001)


def increase_radius(vis):
    global cloud, line_set, radius, mult

    tree = o3d.geometry.KDTreeFlann(cloud)
    new_lines = list()
    for q_idx, query_point in enumerate(cloud.points):
        [k, idx, _] = tree.search_radius_vector_3d(query_point, radius)
        for i in idx:
            new_lines.append([q_idx, i])


    line_set.lines = o3d.utility.Vector2iVector(new_lines)

    # Get spectral features
    rows, cols = 100, 100
    laplacian = csr_matrix((rows, cols), dtype=np.float32)
    for q_idx, query_point in enumerate(cloud.points):
        [k, idx, _] = tree.search_radius_vector_3d(query_point, radius)
        for i in idx:
            if i == q_idx:
                laplacian[q_idx, q_idx] = len(idx) - 1
            else:
                laplacian[q_idx, i] = -1
                laplacian[i, q_idx] = -1



    eigenvalues, _ = eigsh(laplacian, k=laplacian.shape[0]-1, which='SM')
    plt.ion()
    axs[0].hist(eigenvalues.real, bins=20)
    axs[0].set_title('Graph With Radius: ' + str(round(radius, 4)))
    plt.draw()
    plt.show(block=False)
    plt.pause(0.001)
    #plt.ion()
    #plt.hist(eigenvalues.real)
    #
    ## Add labels and title
    #plt.xlabel('Value')
    #plt.ylabel('Frequency')
    #plt.title('Spectral Features With Radius: ' + str(radius))
    #plt.show(block=False)
    #plt.pause(0.001)

    vis.update_geometry(line_set)
    radius += 0.05 * mult
    mult += 1

def main():
    global cloud, line_set


    genereateRandomCloud()
    line_set.points = cloud.points

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(height=400, width=900)
    vis.get_render_option().point_size = 2.0

    # TODO function to get eigs
    # TODO function to get display hist 2 and 3
    display_mcar_hist()
    display_fc_hist()
    vis.add_geometry(cloud)
    vis.add_geometry(line_set)
    vis.register_key_callback(65, increase_radius)

    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()
