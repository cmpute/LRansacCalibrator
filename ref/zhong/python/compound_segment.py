# Fusing the segmented image with original one

import os
import numpy as np
from scipy.misc import imread, imsave

root_dir = 'D:\\编程\\Lidar\\data\\'
ratio = 0.5


for frame in range(100):
    img_path = os.path.join(root_dir, 'rgb', "%06d.png" % frame)
    seg_path = os.path.join(root_dir, 'segment', 'color', "%06d.png" % frame)

    img = imread(img_path)
    segment = imread(seg_path)
    save = img * ratio + segment * (1 - ratio)

    imsave(os.path.join(root_dir, 'segment', 'compound', "%06d.png" % frame), save)
    imsave(os.path.join(root_dir, 'segment', 'compound', "%06d_stack.png" % frame), 
           np.vstack((img, segment, save)))
    print("frame {0} process finished.".format(frame))