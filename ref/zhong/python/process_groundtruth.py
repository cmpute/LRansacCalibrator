import pcl
from pcl.io import loadpcd, savepcd
from scipy.io import loadmat
import numpy as np

pc = loadpcd('D:/编程/Lidar/data_annotated/lidar/000001.pcd')
gt = loadmat('D:/编程/Lidar/data_annotated/groundtruth/lidar/000001.mat')['truth']
pc.append_fields([('truth', 'u1')], {'truth':gt.flatten()})
savepcd('D:/编程/Lidar/data_annotated/lidar/000001.pcd', pc)