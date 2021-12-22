import os
import numpy as np
import matplotlib.pyplot as plt
import csv


def generate_csv():
    bound_c = np.load('clean_lane_0_sparse.npy')
    print(bound_c.shape)

    with open('jase.csv', 'w', newline='\n') as csvfile:
        spamwriter = csv.writer(csvfile)
        for ind in range(bound_c.shape[0]):
            spamwriter.writerow([bound_c[ind, 0], bound_c[ind, 1], 6, 4])

    plt.figure()
    plt.plot(bound_c[:, 0], bound_c[:, 1], "b-", linewidth=0.7)
    plt.grid()
    ax = plt.gca()
    # ax.arrow(point1_arrow[0], point1_arrow[1], vec_arrow[0], vec_arrow[1],
    #             head_width=7.0, head_length=7.0, fc='g', ec='g')
    ax.set_aspect("equal", "datalim")
    plt.xlabel("east in m")
    plt.ylabel("north in m")
    plt.show()

def read_csv():
    lane = []
    filename = 'traj_race_cl_6_8'
    with open(filename + '.csv', 'r', newline='\n') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=';', quotechar='|')
        pass_line = 3
        for row in spamreader:
            if (pass_line == 0):
                lane.append([float(row[1]), float(row[2])])
                print([float(row[1]), float(row[2])])
            else:
                pass_line -= 1
    np.save(filename + '.npy', np.array(lane))

read_csv()