import os
import numpy as np
from scipy.io import loadmat, savemat

relationship = [
    # road, sidewalk, building, wall, fence,
	# pole, traffic_light, traffic_sign, vegetation, terrain,
	# sky, person, rider, car, truck,
	# bus, train, motorcycle, bicycle, traffic_facility
    # traffic_pole, two_wheel_vehicle
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], # NOT_GROUND_TRUTHED
    [0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # building
    [0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0], # sky
    [1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0], # road
    [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0], # vegetation
    [0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # sidewalk
    [0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0], # car
    [0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0], # pedestrian
    [0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0], # cyclist
    [0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0], # signage
    [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # fence
]

def loadpcd(path):
    pc = list()
    with open(path, 'r') as f:
        points_str = f.readlines()
    for point_str in points_str:
        if not point_str[0].isdigit() and not point_str[0] is '-':
            continue
        point_num = point_str.strip().split(' ')
        pc.append([float(num) for num in point_num])

    return np.array(pc)

for frame in range(252):
    data = loadpcd("D:/编程/Lidar/data_annotated/evaluation/%06d.pcd" % frame)
    savemat("D:/编程/Lidar/data_annotated/evaluation/%06d.mat" % frame,
            {'detail': data[:, 3:].astype('int8'),
             'titles':['groundtruth','imagelabel','fusionlabel','imagecorrectness','fusioncorrectness']})
