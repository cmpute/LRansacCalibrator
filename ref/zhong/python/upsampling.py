# upsample data from LiDAR

import os
import numpy as np
import scipy.interpolate as ip

root_dir = 'D:\\编程\\Lidar\\data\\'

resolution = 0.005 # phi resolution in radian
inter_num = 127 # number of interpolated points

for frame in range(5):
    data_path = os.path.join(root_dir, 'result', "%06d.pcd" % frame)
    points_list = list()
    points_fields = list()

    with open(data_path, 'r') as f:
        points_str = f.readlines()
    for point_str in points_str:
        if not point_str[0].isdigit():
            continue
        point_num = point_str.strip().split(' ')

        points_list.append((float(point_num[0]), float(point_num[1]), float(point_num[2])))
        points_fields.append((float(point_num[3]), float(point_num[4]), int(point_num[5])))
    
    points = np.array(points_list)
    points_polar = np.zeros(points.shape)
    points_polar[:, 0] = np.sqrt(points[:, 0]**2 + points[:, 1]**2, points[:, 2]**2) # rho in x-y
    points_polar[:, 1] = np.arctan2(points[:, 1], points[:, 0]) # phi in x-y
    points_polar[:, 2] = np.arcsin(points[:, 2] / points_polar[:, 0]) #theta in rho-z

    # Warning : [list()] * count will cause all of the list pointed to one
    slices = [list() for i in range(int(2 * np.pi / resolution))]
    for point in points_polar:
        slices[int((point[1] + np.pi) / resolution)].append(point)

    for i in range(int(2 * np.pi / resolution)):
        if len(slices[i]) <= 4: continue

        linemat = np.array(slices[i])
        theta = linemat[:, 2]
        rho = linemat[:, 0]
        
        inter = ip.interp1d(theta, rho, kind='slinear')
        theta_min, theta_max = min(theta), max(theta)
        theta_inter = (np.linspace(0, (theta_max-theta_min)**3, inter_num))**(1/3) * 0.999 + theta_min
        rho_inter = inter(theta_inter)
        phi_inter = i * resolution - np.pi

        points_inter = np.zeros((inter_num, 3))
        points_inter[:, 0] = (rho_inter * np.cos(theta_inter) * np.cos(phi_inter)).T
        points_inter[:, 1] = (rho_inter * np.cos(theta_inter) * np.sin(phi_inter)).T
        points_inter[:, 2] = (rho_inter * np.sin(theta_inter)).T

        points = np.vstack((points, points_inter))

    print("frame " + str(frame) + " upsampled with " + str(len(points)) + " points.")

    save_path = os.path.join(root_dir, 'result', "%06d_upsampled.pcd" % frame)
    with open(save_path, 'w') as f:
        headers = ["VERSION .7\n",
                    "FIELDS x y z\n",
                    "SIZE 4 4 4\n",
                    "TYPE F F F\n",
                    "COUNT 1 1 1\n",
                    "WIDTH " + str(len(points)) + '\n',
                    "HEIGHT 1\n",
                    "VIEWPOINT 0 0 0 1 0 0 0\n",
                    "POINTS " + str(len(points)) + '\n',
                    "DATA ascii\n"]
        f.writelines(headers)
        for value in points:
            for data in value:
                # if(data - int(data)==0):
                #     data = int(data)
                f.write(str(data) + ' ')
            f.write('\n')