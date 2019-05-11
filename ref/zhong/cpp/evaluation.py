import os
import numpy as np
from scipy.io import loadmat, savemat

dataroot = 'D:/编程/Lidar/data_annotated/'

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

def sortpoint(data):
    data = data[data[:,2].argsort()] # First sort doesn't need to be stable.
    data = data[data[:,1].argsort(kind='mergesort')]
    data = data[data[:,0].argsort(kind='mergesort')]
    return data

results = list()
result_cover = list()

for frame in range(252):
    ''' read groud truth '''
    gd = loadmat(dataroot + "groundtruth/lidar/%06d.mat" % frame)['truth']
    origin_pc = loadpcd(dataroot + "lidar/%06d.pcd" % frame)
    origin_pc = np.hstack((origin_pc, gd))

    ''' read result file '''
    result_pc = loadpcd(dataroot + "result/%06d_test_output.pcd" % frame)

    ''' sort '''
    # origin_pc.sort(axis=0)
    # result_pc.sort(axis=0)
    origin_pc = sortpoint(origin_pc)
    result_pc = sortpoint(result_pc)
    origin_ptr = result_ptr = 0
    compare_thres = 0.0009
    
    aligned = list()
    while origin_ptr < len(origin_pc) and result_ptr < len(result_pc):
        if abs(origin_pc[origin_ptr][0] - result_pc[result_ptr][0]) < compare_thres:
            if abs(origin_pc[origin_ptr][1] - result_pc[result_ptr][1]) < compare_thres:
                if abs(origin_pc[origin_ptr][2] - result_pc[result_ptr][2]) < compare_thres:
                    image_right = relationship[int(origin_pc[origin_ptr][-1])][int(result_pc[result_ptr][5])]
                    fusion_right = relationship[int(origin_pc[origin_ptr][-1])][int(result_pc[result_ptr][6])]
                    aligned.append(np.hstack((origin_pc[origin_ptr][:3], # xyz
                                              origin_pc[origin_ptr][-1], # groud truth
                                              result_pc[result_ptr][5:7], # image and fusion label
                                              image_right, # image correctness
                                              fusion_right))) #fusion correctness
                    origin_ptr += 1
                    result_ptr += 1
                elif origin_pc[origin_ptr][2] > result_pc[result_ptr][2]:
                    result_ptr += 1
                else: # origin_pc[origin_ptr][2] < result_pc[result_ptr][2]
                    origin_ptr += 1
            elif origin_pc[origin_ptr][1] > result_pc[result_ptr][1]:
                result_ptr += 1
            else: # origin_pc[origin_ptr][1] < result_pc[result_ptr][1]
                origin_ptr += 1
        elif origin_pc[origin_ptr][0] > result_pc[result_ptr][0]:
            result_ptr += 1
        else: # origin_pc[origin_ptr][0] < result_pc[result_ptr][0]
            origin_ptr += 1
    aligned = np.array(aligned)
    
    ''' save file '''
    save_path = dataroot + "evaluation/%06d.pcd" % frame
    with open(save_path, 'w') as f:
        headers = ["VERSION .7\n",
                    "FIELDS x y z label imagelabel fusionlabel imagecorrect fusioncorrect\n",
                    "SIZE 4 4 4 4 1 1 1 1\n",
                    "TYPE F F F U U U U U\n",
                    "COUNT 1 1 1 1 1 1 1 1\n",
                    "WIDTH " + str(len(aligned)) + '\n',
                    "HEIGHT 1\n",
                    "VIEWPOINT 0 0 0 1 0 0 0\n",
                    "POINTS " + str(len(aligned)) + '\n',
                    "DATA ascii\n"]
        f.writelines(headers)
        for value in aligned:
            for data in value:
                if(data - int(data)==0):
                    data = int(data)
                f.write(str(data) + ' ')
            f.write('\n')
    
    print('frame %d process finished!' % frame)
    print('recall: %d / %d' % (len(aligned), len(result_pc)))
    results.append((sum(aligned[:, 6]), sum(aligned[:, 7])))
    result_cover.append(len(aligned) == len(result_pc))
    print('image / fusion label correctness: %d / %d' % results[-1])
    print('-------------------------')
    
    save_path = dataroot + "evaluation/%06d.mat" % frame
    savemat(save_path, {'detail': aligned[:, 3:].astype('int8'),
                        'titles':['groundtruth','imagelabel','fusionlabel','imagecorrectness','fusioncorrectness']})

savemat(dataroot + "evaluation/summary.mat",
            {'detail': np.array(results).astype(int),
             'covered' : result_cover,
             'titles':['image_correct_num','fusion_correct_num']})
print("summary saved")