import time
import math
import os

import astar
import utils

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

DIRECTORY_PATH = "../pcd"
FILE_NAME = "munes_ground.pcd"

VOXEL_SIZE = [2, 2, 0.5]

OCCUPIED_CRITERIA = 10

EXTRACTION_NUM = 5

GOAL_POINTS = [[0, 0, 0], [0, 0, 0]]

if __name__ == "__main__":
    pcd = o3d.io.read_point_cloud(os.path.join(DIRECTORY_PATH, FILE_NAME))
    points_array = np.asarray(pcd.points, dtype=np.int)
    points_array_raw = np.asarray(pcd.points)

    points_array_raw[:,0] -= points_array_raw[:,0].min()
    points_array_raw[:,1] -= points_array_raw[:,1].min()
    points_array_raw[:,2] -= points_array_raw[:,2].min()

    points_array[:,0] -= points_array[:,0].min()
    points_array[:,1] -= points_array[:,1].min()
    points_array[:,2] -= points_array[:,2].min()

    x_size = math.ceil((points_array_raw[:,0].max() - points_array_raw[:,0].min()) / VOXEL_SIZE[0]) + math.ceil(1 / VOXEL_SIZE[0])
    y_size = math.ceil((points_array_raw[:,1].max() - points_array_raw[:,1].min()) / VOXEL_SIZE[1]) + math.ceil(1 / VOXEL_SIZE[1])
    z_size = math.ceil((points_array_raw[:,2].max() - points_array_raw[:,2].min()) / VOXEL_SIZE[2]) + math.ceil(1 / VOXEL_SIZE[2])

    x_unique, x_counts = np.unique(np.floor(points_array_raw[:,0] / VOXEL_SIZE[0]).astype("int"), return_counts=True)
    y_unique, y_counts = np.unique(np.floor(points_array_raw[:,1] / VOXEL_SIZE[1]).astype("int"), return_counts=True)
    z_unique, z_counts = np.unique(np.floor(points_array_raw[:,2] / VOXEL_SIZE[2]).astype("int"), return_counts=True)

    voxels = np.zeros((x_size, y_size, z_size))
    voxels_count = np.zeros((x_size, y_size, z_size))

    road_list = []
    points_list = []

    for i,p in enumerate(points_array):
        x_idx = math.floor(p[0] / VOXEL_SIZE[0])
        y_idx = math.floor(p[1] / VOXEL_SIZE[1])
        z_idx = math.floor(points_array_raw[i][2] / VOXEL_SIZE[2])
        voxels_count[x_idx, y_idx, z_idx] += 1
        if voxels_count[x_idx, y_idx, z_idx] >= OCCUPIED_CRITERIA:
            points_list.append(np.array([x_idx, y_idx, z_idx]))
            if voxels[x_idx, y_idx, z_idx] < 1:
                voxels[x_idx, y_idx, z_idx] = 1
                road_list.append(np.array([x_idx, y_idx, z_idx]))

    roads = np.array(road_list, dtype=np.int)
    points_array = np.array(points_list, dtype=np.int)

    points_dict = {}
    for pa in points_array:
        if pa[2] in points_dict:
            points_dict[pa[2]].append(pa)
        else:
            points_dict[pa[2]] = [pa]

    most_common_x_values = x_unique[x_counts.argsort()[-10:]]
    most_common_y_values = y_unique[y_counts.argsort()[-10:]]
    most_common_z_values = z_unique[z_counts.argsort()[-2:]]
    secondary_common_z_values = z_unique[z_counts.argsort()[-4:-2]]

    points_array_raw_test = np.empty((0,2), dtype=np.int)
    for pa in points_array:
        if pa[2] not in most_common_z_values and pa[2] not in secondary_common_z_values:
            points_array_raw_test = np.vstack((points_array_raw_test, pa[:2]))

    frequency_count_dict = {}
    for r in points_array_raw_test:
        new_key = str(r[0]) + " " + str(r[1])
        if new_key in frequency_count_dict:
            frequency_count_dict[new_key] += 1
        else:
            frequency_count_dict[new_key] = 1

    max_val = sorted(frequency_count_dict.values(), reverse=True)[0]
    m_c_list = []
    for k,v in frequency_count_dict.items():
        if v == max_val:
            k_split = list(map(int, k.split(" ")))
            m_c_list.append(k_split)


    points_total_count_dict = {}
    for k,v in points_dict.items():
        for p in v:
            if [p[0], p[1]] in m_c_list or p[2] in most_common_z_values:
                continue
            new_key = str(p[0]) + " " + str(p[1])
            if new_key in points_total_count_dict:
                points_total_count_dict[new_key] += 1
            else:
                points_total_count_dict[new_key] = 1

    elv_xy = np.array(list(map(int, max(points_total_count_dict, key=points_total_count_dict.get).split(" "))))

    marked_points = []
    for k, v in points_dict.items():
        points_count_dict = {}
        for p in v:
            if [p[0], p[1]] in m_c_list or p[2] in most_common_z_values:
                continue
            new_key = str(p[0]) + " " + str(p[1])
            if new_key in points_count_dict:
                points_count_dict[new_key] += 1
            else:
                points_count_dict[new_key] = 1
        if len(points_count_dict.keys()) > 0:
            i = 0
            added_num = 0
            while True:
                if len(points_count_dict.keys()) > i + 1 and added_num < EXTRACTION_NUM:
                    x, y = list(map(int, list(points_count_dict.keys())[list(points_count_dict.values()).index(sorted(list(points_count_dict.values()), reverse=True)[i])].split(" ")))
                    if x != elv_xy[0] or y != elv_xy[1]:
                        marked_points.append([x, y, k])
                        added_num += 1
                    i += 1
                else:
                    break
    marked_points = np.array(marked_points)

    for z in range(most_common_z_values.min(), most_common_z_values.max() + 1):
        marked_points = np.vstack((marked_points, np.array([elv_xy[0], elv_xy[1], z])))

    for checked_point in roads:
        if most_common_z_values.min() <= checked_point[2] and checked_point[2] <= most_common_z_values.max():
            if [checked_point[0], checked_point[1]] in m_c_list or checked_point[2] in most_common_z_values:
                continue
            elif utils.check_in_2d_array(checked_point, np.array(marked_points)):
                continue
            else:
                voxels[checked_point[0], checked_point[1], checked_point[2]] = 0
        else:
            voxels[checked_point[0], checked_point[1], checked_point[2]] = 0

    mutf_map = np.transpose((voxels > 0).nonzero())

    # GOAL_POINTS[0][0] = np.median(roads[roads[:,2]==most_common_z_values[1]][:,0])
    GOAL_POINTS[0][0] = np.quantile(roads[roads[:,2]==most_common_z_values[1]][:,0], 0.6, interpolation="lower")
    GOAL_POINTS[0][1] = np.median(roads[roads[:,2]==most_common_z_values[1]][:,1])
    GOAL_POINTS[0][2] = most_common_z_values[1]
    # GOAL_POINTS[1][0] = np.median(roads[roads[:,2]==most_common_z_values[0]][:,0])
    GOAL_POINTS[1][0] = np.quantile(roads[roads[:,2]==most_common_z_values[0]][:,0], 0.6, interpolation="lower")
    GOAL_POINTS[1][1] = np.median(roads[roads[:,2]==most_common_z_values[0]][:,1])
    GOAL_POINTS[1][2] = most_common_z_values[0]

    start_point = np.array(GOAL_POINTS[0], dtype=int)
    end_point = np.array(GOAL_POINTS[-1], dtype=int)

    sta = time.time()
    total_paths = []
    total_vs = []
    for i in range(len(GOAL_POINTS) - 1):
        s_start = GOAL_POINTS[i]
        s_goal = GOAL_POINTS[i+1]
        astar_instance = astar.Weighted_A_star(s_start, s_goal, mutf_map, 1.0)
        astar_instance.run()
        total_paths += astar_instance.Path
        total_vs += astar_instance.V
    print(time.time() - sta)

    paths = np.array(total_paths[::-1])[:,1,:]
    paths = np.vstack((paths, np.array(total_paths[::-1])[-1,0]))
    normalized_path_xs = paths[:,0]
    normalized_path_ys = paths[:,1]
    normalized_path_zs = paths[:,2]
    normalized_path = np.empty((paths.shape[0], 3), dtype=np.int)
    normalized_path[:,0] = normalized_path_xs
    normalized_path[:,1] = normalized_path_ys
    normalized_path[:,2] = normalized_path_zs

    vertices = np.array(total_vs)
    normalized_vertex_xs = vertices[:,0]
    normalized_vertex_ys = vertices[:,1]
    normalized_vertex_zs = vertices[:,2]
    normalized_vertex = np.empty((vertices.shape[0], 3), dtype=np.int)
    normalized_vertex[:,0] = normalized_vertex_xs
    normalized_vertex[:,1] = normalized_vertex_ys
    normalized_vertex[:,2] = normalized_vertex_zs

    # Visualization
    colors = np.empty(voxels.shape, dtype=object)

    for checked_point in roads:
        if most_common_z_values.min() <= checked_point[2] and checked_point[2] <= most_common_z_values.max():
            if np.array_equal(checked_point, start_point):
                colors[checked_point[0], checked_point[1], checked_point[2]] = "#03C04A" # green
            elif np.array_equal(checked_point, end_point):
                colors[checked_point[0], checked_point[1], checked_point[2]] = "blue"
            elif utils.check_in_2d_array(checked_point, normalized_path):
                colors[checked_point[0], checked_point[1], checked_point[2]] = "#C5A3FF"
            elif [checked_point[0], checked_point[1]] in m_c_list or checked_point[2] in most_common_z_values:
                colors[checked_point[0], checked_point[1], checked_point[2]] = "#EBECF0" ## elevators and stairs
            elif utils.check_in_2d_array(checked_point, np.array(marked_points)):
                colors[checked_point[0], checked_point[1], checked_point[2]] = "#EBECF0" ## floors of corridors

    ax = plt.figure().add_subplot(projection='3d')
    ax.grid(False)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    plt.axis('off')
    plt.grid(b=None)
    ax.voxels(voxels, facecolors=colors, edgecolor='k')

    plt.show()