
import open3d as o3d
import numpy as np
import random

from scipy.spatial.distance import euclidean
from scipy.sparse.linalg import eigs
from scipy.sparse import csr_matrix

import matplotlib.pyplot as plt

cloud= o3d.geometry.PointCloud()
line_set = o3d.geometry.LineSet()
radius = 0

def calc_lineset():
    global cloud, line_set
    tree = o3d.geometry.KDTreeFlann(cloud)
    new_lines = list()
    for q_idx, query_point in enumerate(cloud.points):
        [k, idx, _] = tree.search_knn_vector_3d(query_point, 4)
        for i in idx:
            new_lines.append([q_idx, i])


    line_set.points = cloud.points
    line_set.lines = o3d.utility.Vector2iVector(new_lines)

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

def calc_mcar():
    global cloud, radius
    tree = o3d.geometry.KDTreeFlann(cloud)
    for q_idx, query_point in enumerate(cloud.points):
        [k, idx, _] = tree.search_knn_vector_3d(query_point, 2)
        point = cloud.points[idx[1]]
        dist = euclidean(point, query_point)
        if dist > radius:
            radius = dist

    radius = round(radius, 4)
    print(radius)

    #eigenvalues, _ = eigsh(laplacian, k=50, which='SM')
    #plt.ion()
    #plt.hist(eigenvalues.real)
    #
    ## Add labels and title
    #plt.xlabel('Value')
    #plt.ylabel('Frequency')
    #plt.title('Spectral Features')
    #plt.show(block=False)
    #plt.pause(0.001)

def plot_hist(eigs):
    plt.ion()
    plt.hist(eigs)
    
    plt.xlabel('Value')
    plt.ylabel('Frequency')
    plt.title('Spectral Features')
    plt.show(block=False)
    plt.pause(0.001)


def get_eigs():
    global cloud, line_set

    rows, cols = 100, 100
    laplacian = csr_matrix((rows, cols), dtype=np.float32)
    for line in line_set.lines:
        idx_1 = line[0]
        idx_2 = line[1]

        if idx_1 == idx_2:
            laplacian[idx_1, idx_1] = 3
        else:
            laplacian[idx_1, idx_2] = -1
            laplacian[idx_2, idx_1] = -1

    eigenvalues, _ = eigs(laplacian, k=50, which='LM')
    return eigenvalues.real


def rotate(vis):
    global cloud, radius, line_set
    # TODO need to rotate
    calc_mcar()
    R = np.asarray([[1, 0, 0],
                [0, np.cos(np.radians(30)), -np.sin(np.radians(30))],
                [0, np.sin(np.radians(30)), np.cos(np.radians(30))]])

    t = np.asarray([1, 0, 0])

    T = np.identity(4)
    T[:3, :3] = R
    T[:3, 3] = t
    cloud.transform(T)

    # Calculate lineset
    #tree = o3d.geometry.KDTreeFlann(cloud)
    #new_lines = list()
    #for q_idx, query_point in enumerate(cloud.points):
    #    [k, idx, _] = tree.search_radius_vector_3d(query_point, radius)
    #    for i in idx:
    #        new_lines.append([q_idx, i])


    line_set.points = cloud.points
    #line_set.lines = o3d.utility.Vector2iVector(new_lines)

    eigs = get_eigs()
    plot_hist(eigs)

    vis.update_geometry(cloud)
    vis.update_geometry(line_set)


def main():
    global cloud, line_set

    genereateRandomCloud()
    calc_lineset()

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(height=400, width=600)
    vis.get_render_option().point_size = 2.0

    eigs = get_eigs()
    plot_hist(eigs)

    vis.add_geometry(cloud)
    vis.add_geometry(line_set)
    vis.register_key_callback(65, rotate)

    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()
