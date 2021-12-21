import os
import numpy as np
import matplotlib.pyplot as plt

# bound_l_ori = np.load('left_lane_bound.npy')
# bound_r_ori = np.load('right_lane_bound.npy')
# bound_c_ori = np.load('center_lane_bound_ori.npy')
bound_l = np.load('left_lane_bound_dense.npy')
bound_r = np.load('right_lane_bound_dense.npy')
lane_c = np.load('traj_race_cl_4_8_mincur_dense.npy')
bound_c = np.load('center_lane.npy')

# def adjust_l():
#     bound_l = bound_l_ori.copy()
#     bound_l[33:385-(48-33)] = bound_l_ori[48:385]
#     bound_l[385-(48-33):385] = bound_l_ori[33:48]
#     bound_l_ori = bound_l.copy()
#     bound_l[98:385-(122-98)] = bound_l_ori[122:385]
#     bound_l[385-(122-98):385] = bound_l_ori[98:122]
#     bound_l_ori = bound_l.copy()
#     bound_l[173:173+(385-361)] = bound_l_ori[361:385]
#     bound_l[173+(385-361):385] = bound_l_ori[173:361]

# def adjust_r():
#     bound_r = bound_r_ori.copy()
#     start_ind = 33
#     bound_r[start_ind:start_ind+(129-64)] = bound_r_ori[64:129]
#     start_ind += (129-64)
#     bound_r[start_ind:start_ind+(228-153)] = bound_r_ori[153:228]
#     start_ind += (228-153)
#     bound_r[start_ind:start_ind+(153-129)] = bound_r_ori[129:153]
#     start_ind += (153-129)
#     bound_r[start_ind:start_ind+(385-228)] = bound_r_ori[228:385]
#     start_ind += (385-228)
#     bound_r[start_ind:start_ind+(64-33)] = bound_r_ori[33:64]
#     start_ind += (64-33)

# def adjust_l():
#     bound_c = bound_c_ori.copy()
#     start_ind = 207
#     bound_c[start_ind:start_ind+(625-313)] = bound_c_ori[313:625]
#     start_ind += (625-313)
#     bound_c[start_ind:start_ind+(1157-777)] = bound_c_ori[777:1157]
#     start_ind += (1157-777)
#     bound_c[start_ind:start_ind+(777-625)] = bound_c_ori[625:777]
#     start_ind += (777-625)
#     bound_c[start_ind:start_ind+(2095-1157)] = bound_c_ori[1157:2095]
#     start_ind += (2095-1157)
#     bound_c[start_ind:start_ind+(313-207)] = bound_c_ori[207:313]

def print_out(line):
    for ind in range(line.shape[0]):
        print(ind, line[ind, 0], line[ind, 1])

def densify_line_simple():
    bound_c_dense = []
    for ind in range(bound_c.shape[0]-1):
        bound_c_dense.append([bound_c[ind, 0], bound_c[ind, 1]])
        bound_c_dense.append([(bound_c[ind, 0]+bound_c[ind+1, 0])/2, (bound_c[ind, 1] + bound_c[ind+1, 1])/2])
    bound_c_dense = np.array(bound_c_dense)

def densify_line(bound_r, minimum_distance = 0.7):
    duplicate_point = False
    line_dense = []
    # print(bound_r.shape)
    for ind in range(bound_r.shape[0]-1):
        if not duplicate_point:
            line_dense.append([bound_r[ind, 0], bound_r[ind, 1]])
            duplicate_point = False
        distance = np.sqrt((bound_r[ind+1, 0] - bound_r[ind, 0]) ** 2 + (bound_r[ind+1, 1] - bound_r[ind, 1]) ** 2)
        distance_fixed = distance
        if distance <= 0.1:
            duplicate_point = True
        new_point = [bound_r[ind, 0], bound_r[ind, 1]]
        while distance > minimum_distance:
            distance -= minimum_distance
            new_point = [(bound_r[ind+1, 0] - bound_r[ind, 0])/distance_fixed * minimum_distance + new_point[0], (bound_r[ind+1, 1] - bound_r[ind, 1])/distance_fixed * minimum_distance + new_point[1]]
            line_dense.append([new_point[0], new_point[1]])
        # print(distance)
    line_dense = np.array(line_dense)
    print(line_dense.shape)
    return line_dense

def sparsify_line(bound_r, minimum_distance = 2.0):
    line_sparse = []
    distance_accumulate = 0
    # print(bound_r.shape)
    for ind in range(bound_r.shape[0]-1):
        distance = np.sqrt((bound_r[ind+1, 0] - bound_r[ind, 0]) ** 2 + (bound_r[ind+1, 1] - bound_r[ind, 1]) ** 2)
        distance_accumulate += distance
        if distance_accumulate > minimum_distance:
            distance_accumulate = 0
            new_point = [bound_r[ind, 0], bound_r[ind, 1]]
            line_sparse.append([new_point[0], new_point[1]])
        # print(distance)
    line_sparse = np.array(line_sparse)
    print(line_sparse.shape)
    return line_sparse

def find_side_lane(bound_c, shift_distance, side):
    # shift_distance = 4.0
    new_lane = []
    new_point = bound_c[0]
    for ind in range(bound_c.shape[0]-1):
        distance = np.sqrt((bound_c[ind+1, 0] - bound_c[ind, 0]) ** 2 + (bound_c[ind+1, 1] - bound_c[ind, 1]) ** 2)
        diff = (bound_c[ind+1] - bound_c[ind]) / distance
        theta = np.radians(side)
        c, s = np.cos(theta), np.sin(theta)
        R = np.array(((c, -s), (s, c)))
        perpendicular_diff = diff.dot(R)
        
        new_point_diff = np.linalg.norm(new_point - (bound_c[ind] + shift_distance * perpendicular_diff))
        new_point = bound_c[ind] + shift_distance * perpendicular_diff
        if new_point_diff > 2:
            print(ind, new_point_diff, bound_c[ind], bound_c[ind+1])
        new_lane.append(new_point)
    new_lane = np.array(new_lane)
    return new_lane

def clean_center_lane(bound_c):
    new_lane = bound_c[0:1800, :]
    new_lane = np.append(new_lane, bound_c[2250:3870, :], axis=0)
    new_lane = np.append(new_lane, bound_c[4230:, :], axis=0)
    # new_lane = densify_line(new_lane, 0.7)
    # np.save('clean_center_lane.npy', new_lane)
    print(new_lane.shape)
    return new_lane


shift = 3.2
# new_lane = clean_center_lane(bound_c.copy())
# print(new_lane.shape)
# 
new_lane = np.load('clean_lane_0_sparse.npy')
new_lane = find_side_lane(new_lane, np.abs(shift), np.sign(shift) * 90)
new_lane = densify_line(new_lane, 0.5)
# new_lane = sparsify_line(new_lane, 4.0)
# np.save('clean_lane_' + str(shift) + '_dense.npy', new_lane)
# new_lane = np.append(new_lane, new_lane[0:])
np.save('clean_lane_3.2_dense.npy', new_lane)



plt.figure()
plt.plot(bound_r[:, 0], bound_r[:, 1], "k-", linewidth=0.7)
plt.plot(bound_l[:, 0], bound_l[:, 1], "k-", linewidth=0.7)
plt.plot(lane_c[:, 0], lane_c[:, 1], "b-", linewidth=0.7)
plt.plot(new_lane[:, 0], new_lane[:, 1], "r-", linewidth=0.7)

plt.grid()
ax = plt.gca()
# ax.arrow(point1_arrow[0], point1_arrow[1], vec_arrow[0], vec_arrow[1],
#             head_width=7.0, head_length=7.0, fc='g', ec='g')
ax.set_aspect("equal", "datalim")
plt.xlabel("east in m")
plt.ylabel("north in m")
plt.show()