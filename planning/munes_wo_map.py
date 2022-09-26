import time

import astar
import utils

import numpy as np
import matplotlib.pyplot as plt

ROAD_WIDTH = 5
TRACK_SIZE = 20 # length of corridor
INTERSECTION_SIZE = 6 # size of intersection connecting a corridor and stiars
FLOOR_GAP = 10 # distance between floors
STAIR_SIZE = int(FLOOR_GAP/2)
NUM_FLOORS = 5
CURR_ELV_FLOOR_FROM_BASE = 0 # current location of elevator (related to elevator waiting time)
ELV_SPEED = 1
ROBOT_SPEED = 1
VOXEL_SIZE = [1, 1, 1]
GOAL_POINTS = [[ 8,  1, 40],
    [ 10,  3, 30],
    [ 18,  2, 20],
    [7,  3, 10],
    [ 21,  2,  0]]
# GOAL_POINTS = [
#             [ 8,  1, 10],
#     [ 21,  2,  0]
#     ]

if __name__ == "__main__":

    track_size = int(TRACK_SIZE / VOXEL_SIZE[0])
    road_width = int(ROAD_WIDTH / VOXEL_SIZE[1])
    intersection_size = int(INTERSECTION_SIZE / VOXEL_SIZE[0])
    floor_gap = int(FLOOR_GAP / VOXEL_SIZE[2])
    stair_size = int(STAIR_SIZE / VOXEL_SIZE[2])

    CURR_ELV_LOC = CURR_ELV_FLOOR_FROM_BASE * floor_gap

    floors_list = []
    for i in range(NUM_FLOORS):
        floors_list.append(i*floor_gap)

    # Map building
    road_list = []
    stair_list = []
    corridor_list = []
    elevator_list = []

    for i in range(track_size+1):
        for j in range(road_width):
            for z in floors_list:
                point = np.array([i,j,z])
                road_list.append(point)
                corridor_list.append(point)
    for i in range(intersection_size):
        for j in range(road_width):
            r_x = track_size + 1 + i
            for z in floors_list:
                point = np.array([r_x, j, z])
                road_list.append(point)
                corridor_list.append(point)
            if j <= int(road_width / 2):
                for n in range(NUM_FLOORS - 1):
                    point = np.array([r_x, j + road_width + stair_size - 1, int(floor_gap/2) + floor_gap * n])
                    road_list.append(point)
                    stair_list.append(point)
        for n in range(NUM_FLOORS - 1):
            for k in range(stair_size - 1):
                r_y = road_width + k
                if i >= intersection_size / 2:
                    r_z = int(floor_gap/2) * k / stair_size + 1 + n * floor_gap
                else:
                    r_z = (n+1) * floor_gap - int(floor_gap/2) * k / stair_size - 1
                point = np.array([r_x, r_y, r_z])
                road_list.append(point)
                stair_list.append(point)
    for i in range(floor_gap * (NUM_FLOORS - 1)):
        point = np.array([0, int(road_width/2), i])
        road_list.append(point)
        elevator_list.append(point)
    roads = np.array(road_list, dtype=np.int)
    stairs = np.array(stair_list, dtype=np.int)
    corridors = np.array(corridor_list, dtype=np.int)
    elevators = np.array(elevator_list, dtype=np.int)

    return_goal_points = GOAL_POINTS[::-1] ## Reverse the goal points
    return_goal_points = [GOAL_POINTS[-1], GOAL_POINTS[0]]

    sta = time.time()
    total_paths = []
    total_vs = []
    total_return_paths = []
    total_return_vs = []
    partial_paths = []
    curr_elv_loc = CURR_ELV_LOC
    
    # Execute the trajectory planning
    for i in range(len(GOAL_POINTS) - 1):
        print(i)
        s_start = GOAL_POINTS[i]
        s_goal = GOAL_POINTS[i+1]
        astar_instance = astar.Weighted_A_star(s_start, s_goal, roads, 1.0, NUM_FLOORS, curr_elv_loc, ELV_SPEED, ROBOT_SPEED, VOXEL_SIZE[2])
        astar_instance.run()
        total_paths += astar_instance.Path[::-1][1:]
        total_vs += astar_instance.V[::-1][1:]
        curr_elv_loc = s_goal[2]
        partial_paths.append(astar_instance.Path[::-1][1:])
    # Planning for the returning trajectory
    for i in range(len(return_goal_points) - 1):
        print(i + len(GOAL_POINTS) - 1)
        s_start = return_goal_points[i]
        s_goal = return_goal_points[i+1]
        astar_instance = astar.Weighted_A_star(s_start, s_goal, roads, 1.0, NUM_FLOORS, curr_elv_loc, ELV_SPEED, ROBOT_SPEED, VOXEL_SIZE[2])
        astar_instance.run()
        total_return_paths += astar_instance.Path[::-1][1:]
        total_return_vs += astar_instance.V[::-1][1:]
        curr_elv_loc = s_goal[2]
        partial_paths.append(astar_instance.Path[::-1][1:])
    print(time.time() - sta)

    # Visualize the trajectory with map
    x_size = int(roads[:,0].max() - roads[:,0].min()) + 1
    y_size = int(roads[:,1].max() - roads[:,1].min()) + 1
    z_size = int(roads[:,2].max() - roads[:,2].min()) + 1
    x, y, z = np.indices((x_size, y_size, z_size))

    normalized_road_xs = roads[:,0] - roads[:,0].min()
    normalized_road_ys = roads[:,1] - roads[:,1].min()
    normalized_road_zs = roads[:,2] - roads[:,2].min()
    normalized_road = np.empty((roads.shape[0], 3), dtype=np.int)
    normalized_road[:,0] = normalized_road_xs
    normalized_road[:,1] = normalized_road_ys
    normalized_road[:,2] = normalized_road_zs

    paths = np.array(total_paths)[:,1,:]
    normalized_path_xs = paths[:,0] - roads[:,0].min()
    normalized_path_ys = paths[:,1] - roads[:,1].min()
    normalized_path_zs = paths[:,2] - roads[:,2].min()
    normalized_path = np.empty((paths.shape[0], 3), dtype=np.int)
    normalized_path[:,0] = normalized_path_xs
    normalized_path[:,1] = normalized_path_ys
    normalized_path[:,2] = normalized_path_zs

    normalized_partial_paths = []
    for pp in partial_paths:
        pp_array = np.array(pp)[:,1,:]
        normalized_pp_xs = pp_array[:,0] - roads[:,0].min()
        normalized_pp_ys = pp_array[:,1] - roads[:,1].min()
        normalized_pp_zs = pp_array[:,2] - roads[:,2].min()
        normalized_pp = np.empty((pp_array.shape[0], 3), dtype=np.int)
        normalized_pp[:,0] = normalized_pp_xs
        normalized_pp[:,1] = normalized_pp_ys
        normalized_pp[:,2] = normalized_pp_zs
        normalized_partial_paths.append(normalized_pp)

    return_paths = np.array(total_return_paths)[:,1,:]
    normalized_return_path_xs = return_paths[:,0] - roads[:,0].min()
    normalized_return_path_ys = return_paths[:,1] - roads[:,1].min()
    normalized_return_path_zs = return_paths[:,2] - roads[:,2].min()
    normalized_return_path = np.empty((return_paths.shape[0], 3), dtype=np.int)
    normalized_return_path[:,0] = normalized_return_path_xs
    normalized_return_path[:,1] = normalized_return_path_ys
    normalized_return_path[:,2] = normalized_return_path_zs

    vertices = np.array(total_vs)
    normalized_vertex_xs = vertices[:,0] - roads[:,0].min()
    normalized_vertex_ys = vertices[:,1] - roads[:,1].min()
    normalized_vertex_zs = vertices[:,2] - roads[:,2].min()
    normalized_vertex = np.empty((vertices.shape[0], 3), dtype=np.int)
    normalized_vertex[:,0] = normalized_vertex_xs
    normalized_vertex[:,1] = normalized_vertex_ys
    normalized_vertex[:,2] = normalized_vertex_zs

    return_vertices = np.array(total_return_vs)
    normalized_return_vertex_xs = return_vertices[:,0] - roads[:,0].min()
    normalized_return_vertex_ys = return_vertices[:,1] - roads[:,1].min()
    normalized_return_vertex_zs = return_vertices[:,2] - roads[:,2].min()
    normalized_return_vertex = np.empty((return_vertices.shape[0], 3), dtype=np.int)
    normalized_return_vertex[:,0] = normalized_return_vertex_xs
    normalized_return_vertex[:,1] = normalized_return_vertex_ys
    normalized_return_vertex[:,2] = normalized_return_vertex_zs

    goal_points = np.array(GOAL_POINTS)
    normalized_gp_xs = goal_points[:,0] - roads[:,0].min()
    normalized_gp_ys = goal_points[:,1] - roads[:,1].min()
    normalized_gp_zs = goal_points[:,2] - roads[:,2].min()
    normalized_gp = np.empty((goal_points.shape[0], 3), dtype=np.int)
    normalized_gp[:,0] = normalized_gp_xs
    normalized_gp[:,1] = normalized_gp_ys
    normalized_gp[:,2] = normalized_gp_zs

    voxels = np.zeros((x_size, y_size, z_size))
    colors = np.empty(voxels.shape, dtype=object)

    for checked_point in roads:
        if utils.check_in_2d_array(checked_point, normalized_road):
            if utils.check_in_2d_array(checked_point, normalized_gp[:1]):
                colors[checked_point[0], checked_point[1], checked_point[2]] = "red"
            elif utils.check_in_2d_array(checked_point, normalized_gp[-1:]):
                colors[checked_point[0], checked_point[1], checked_point[2]] = "purple"
            elif utils.check_in_2d_array(checked_point, normalized_gp[1:2]):
                colors[checked_point[0], checked_point[1], checked_point[2]] = "yellow"
            elif utils.check_in_2d_array(checked_point, normalized_gp[2:3]):
                colors[checked_point[0], checked_point[1], checked_point[2]] = "green"
            elif utils.check_in_2d_array(checked_point, normalized_gp[3:4]):
                colors[checked_point[0], checked_point[1], checked_point[2]] = "blue"
            elif utils.check_in_2d_array(checked_point, normalized_return_path):
                colors[checked_point[0], checked_point[1], checked_point[2]] = "lightskyblue"# "#C5A3FF"
            elif utils.check_in_2d_array(checked_point, normalized_path):
                colors[checked_point[0], checked_point[1], checked_point[2]] = "#C5A3FF"# "#C5A3FF"
            else:
                colors[checked_point[0], checked_point[1], checked_point[2]] = (235 / 255, 236 / 255, 240 / 255) #"#EBECF0" # light gray
            voxels[checked_point[0], checked_point[1], checked_point[2]] = 1

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.view_init(elev=17, azim=-64)
    ax.grid(False)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    ax.set_xlim(normalized_road[:,0].min(), normalized_road[:,0].max())
    ax.set_ylim(normalized_road[:,1].min(), normalized_road[:,1].max())
    ax.set_zlim(normalized_road[:,2].min(), normalized_road[:,2].max())
    plt.axis('off')
    plt.grid(b=None)
    ax.voxels(voxels, facecolors=colors, edgecolor='k')
    fig.tight_layout()
    # fig.canvas.manager.full_screen_toggle()

    plt.show()