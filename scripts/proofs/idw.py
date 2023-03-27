import open3d as o3d
import numpy as np
import random

from scipy.spatial.distance import euclidean
from scipy.sparse.linalg import eigsh
from scipy.sparse import csr_matrix

import matplotlib.pyplot as plt

cloud= o3d.geometry.PointCloud()
line_set = o3d.geometry.LineSet()

fig, axs = plt.subplots(1, 2, figsize=(12, 6))

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

def display_gereric_hist(eigs):
    plt.ion()
    axs[0].hist(eigs, bins=20)
    axs[0].set_title('Generic Laplacian')
    plt.draw()
    plt.show(block=False)
    plt.pause(0.001)

def display_idw_hist(eigs):
    plt.ion()
    axs[1].hist(eigs, bins=20)
    axs[1].set_title('IDW Laplacian')
    plt.draw()
    plt.show(block=False)
    plt.pause(0.001)

def calc_gen_eigs():
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

    eigenvalues, _ = eigsh(laplacian, k=70, which='LM')
    print(eigenvalues.real)
    return eigenvalues.real

def calc_idw_eigs():
    global cloud, line_set

    # find bias term
    d_s = 100
    for line in line_set.lines:
        p1 = cloud.points[line[0]]
        p2 = cloud.points[line[1]]
        dist = euclidean(p1, p2)
        if dist < d_s:
            d_s = dist

    rows, cols = 100, 100
    laplacian = csr_matrix((rows, cols), dtype=np.float32)
    bias = 1 - d_s
    for line in line_set.lines:
        p1 = cloud.points[line[0]]
        p2 = cloud.points[line[1]]
        idx_1 = line[0]
        idx_2 = line[1]
        dist = euclidean(p1, p2)

        if idx_1 == idx_2:
            laplacian[idx_1, idx_1] = 3
        else:
            laplacian[idx_1, idx_2] = -1 / (dist + bias) * 2
            laplacian[idx_2, idx_1] = -1 / (dist + bias) * 2


    eigenvalues, _ = eigsh(laplacian, k=laplacian.shape[0]-1, which='LM')
    return eigenvalues.real

def change_distances(vis):
    global cloud, line_set

    # TODO randomly change half of the points
    # When you change them make sure that the end distance is not greater than 1
    change_idx_list = [random.randint(0, 99) for _ in range(50)]
    for idx in change_idx_list:
        point = cloud.points[idx]
        random_x = random.uniform(-0.3, 0.3)
        random_y = random.uniform(-0.3, 0.3)
        random_z = random.uniform(-0.3, 0.3)

        new_x = point[0] + random_x
        new_y = point[1] + random_y
        new_z = point[2] + random_z

        cloud.points[idx] = [new_x, new_y, new_z]

    line_set.points = cloud.points
    gen_eigs = calc_gen_eigs()
    idw_eigs = calc_idw_eigs()
    display_gereric_hist(gen_eigs)
    display_idw_hist(idw_eigs)
    vis.update_geometry(cloud)
    vis.update_geometry(line_set)

def main():
    global cloud, line_set

    genereateRandomCloud()
    calc_lineset()

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(height=400, width=600)
    vis.get_render_option().point_size = 5.0

    vis.add_geometry(cloud)
    vis.add_geometry(line_set)

    gen_eigs = calc_gen_eigs()
    idw_eigs = calc_idw_eigs()
    display_gereric_hist(gen_eigs)
    display_idw_hist(idw_eigs)
    # press a key
    vis.register_key_callback(65, change_distances)
    #vis.register_key_action_callback(65, key_action_callback)

    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()
