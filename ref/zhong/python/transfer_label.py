"""
Script to deal with object evaluation data from kitti dataset
"""
import os
import numpy as np

root_dir = 'D:\\编程\\Lidar\\data_annotated\\'
# image_shape = 375, 1242

############################################################
def load_velodyne_points(frame, luminance=False):
    points_path = os.path.join(root_dir, 'lidar', "%06d.bin" % frame)
    points = np.fromfile(points_path, dtype=np.float32).reshape(-1, 4)
    return (points[:, :3], points[:, 3]) if luminance else points[:, :3]

def load_calib_file(frame):
    calib_path = os.path.join(root_dir, 'calib', "%06d.txt" % frame)
    float_chars = set("0123456789.e+- ")

    data = {}
    with open(calib_path, 'r') as f:
        for line in [l for l in f.read().split('\n') if len(l) > 0]:
            key, value = line.split(' ', 1)
            if key.endswith(':'):
                key = key[:-1]
            value = value.strip()
            if float_chars.issuperset(value):
                data[key] = np.array([float(v) for v in value.split(' ')])
            else:
                print('warning: unknown value!')

    return data

def load_image_file(frame):
    from scipy.misc import imread

    img_path = os.path.join(root_dir, 'rgb', "%06d.png" % frame)
    return imread(img_path)

def load_segment_file(frame):
    from scipy.misc import imread

    img_path = os.path.join(root_dir, 'segment', 'gray', "%06d.png" % frame)
    return imread(img_path)

def get_rgb_from_image(image, points):
    values = list()

    # get rgb information
    for point in points:
        rgb = image[int(point[1]), int(point[0])]
        values.append(rgb)

    values = np.array(values, dtype='uint8')
    values = np.insert(values, 3, np.zeros(len(values)), 1) # hstack is alternative
    values.dtype = 'float32'

    return values

def get_label_from_segment(frame, points):
    values = list()
    seg = load_segment_file(frame)

    # get labeling result
    for point in points:
        values.append(seg[int(point[1]), int(point[0])])

    values = np.array(values).reshape((-1, 1))

    return values

def color_tuple(color):
    return (color[0], color[1], color[2])

def get_label_list():
    from scipy.misc import imread
    
    count = 0
    labels = ['road', 'sidewalk', 'building', 'wall', 'fence',
              'pole', 'traffic light', 'traffic sign', 'vegetation', 'terrain',
              'sky', 'person', 'rider', 'car', 'truck',
              'bus', 'train', 'motorcycle', 'bicycle']

    label_map = dict()
    img_path = os.path.join(root_dir, 'segment', "colormap.png")
    img_map = imread(img_path)

    w, h = img_map.shape
    for i in range(4):
        for j in range(5):
            color = color_tuple(img_map[i*w/5+1, j*h/4+1])
            label_map[color] = count
            count = count + 1

############################################################
def get_velo2rect(calib_data):
    RT_velo2cam = np.eye(4)
    RT_velo2cam[:3, :4] = calib_data['Tr_velo_cam'].reshape(3, 4) # Tr_velo_to_cam

    R_rect00 = np.eye(4)
    R_rect00[:3, :3] = calib_data['R_rect'].reshape(3, 3) # R0_rect

    RT_velo2rect = np.dot(R_rect00, RT_velo2cam)
    return RT_velo2rect.T

def get_rect2disp(calib_data, color=True):
    cam0, cam1 = (0, 1) if not color else (2, 3)
    P_rect0 = calib_data['P' + str(cam0)].reshape(3, 4)
    P_rect1 = calib_data['P' + str(cam1)].reshape(3, 4)

    P0, P1, P2 = P_rect0
    Q0, Q1, Q2 = P_rect1

    # create disp transform
    T = np.array([P0, P1, P0 - Q0, P2])

    return T.T

def homogeneous_transform(points, transform):
    """
    Parameters
    ----------
    points : (n_points, M) array-like
        The points to transform. If `points` is shape (n_points, M-1), a unit
        homogeneous coordinate will be added to make it (n_points, M).
    transform : (M, N) array-like
        The right-multiplying transformation to apply.
    """
    points = np.asarray(points)
    transform = np.asarray(transform)
    n_points, D = points.shape
    M, N = transform.shape

    # do transformation in homogeneous coordinates
    if D == M - 1:
        points = np.hstack([points, np.ones((n_points, 1), dtype=points.dtype)])
    elif D != M:
        raise ValueError("Number of dimensions of points (%d) does not match"
                         "input dimensions of transform (%d)." % (D, M))

    new_points = np.dot(points, transform)

    # normalize homogeneous coordinates
    new_points = new_points[:, :-1] / new_points[:, [-1]]

    # x,y is exchanged in original function
    # new_points[:,[0,1]] = new_points[:,[1,0]]

    return new_points

def filter_disps(xyd, image_shape, max_disp=255, return_mask=False):
    x, y, d = xyd.T
    mask = ((x >= 0) & (x <= image_shape[1] - 1) &
            (y >= 0) & (y <= image_shape[0] - 1) &
            (d >= 0) & (d <= max_disp))
    xyd = xyd[mask]
    return (xyd, mask) if return_mask else xyd

############################################################
# main function to load points to image
def load_disparity_points(frame: "the number of the frame from object dataset",
                          image_shape: "the size of image",
                          luminance: "whether return luminance data"=False,
                          color: "whether use color camera"=True):

    calib = load_calib_file(frame)

    # read velodyne points
    if luminance:
        points, lumin = load_velodyne_points(frame, luminance)
    else:
        points = load_velodyne_points(frame)

    mask = np.arange(len(points))

    # remove all points behind image plane (approximation)
    mask_behind = points[:, 0] >= 1
    points = points[mask_behind, :]
    mask = mask[mask_behind]

    # convert points to each camera
    velo2disp = np.dot(get_velo2rect(calib), get_rect2disp(calib, color))
    xyd = homogeneous_transform(points, velo2disp)

    # take only points that fall in the first image
    xyd, mask_disp = filter_disps(xyd, image_shape, return_mask=True)
    mask = mask[mask_disp]

    if luminance:
        lumin = lumin[mask]

    return (xyd, lumin, mask) if luminance else (xyd, mask)


def plot_to_camera(frame, plot: "whether show scatter plot"=True,
                   save: "whether save to pcd file"=False):
    import matplotlib.pyplot as plt

    img = load_image_file(frame)

    points, lumin, mask = load_disparity_points(frame, img.shape, True)
    # label = points[:,2]
    label = lumin

    if save:
        original = load_velodyne_points(frame)
        original = original[mask]
        all_values = np.hstack((original, lumin.reshape(len(lumin), 1),
                                get_rgb_from_image(img, points),
                                get_label_from_segment(frame, points)))

        save_path = os.path.join(root_dir, 'result', "%06d.pcd" % frame)
        with open(save_path, 'w') as f:
            headers = ["VERSION .7\n",
                       "FIELDS x y z intensity rgb label\n",
                       "SIZE 4 4 4 4 4 4\n",
                       "TYPE F F F F F U\n",
                       "COUNT 1 1 1 1 1 1\n",
                       "WIDTH " + str(len(all_values)) + '\n',
                       "HEIGHT 1\n",
                       "VIEWPOINT 0 0 0 1 0 0 0\n",
                       "POINTS " + str(len(all_values)) + '\n',
                       "DATA ascii\n"]
            f.writelines(headers)
            all_values[np.isnan(all_values)] = 0
            for value in all_values:
                for data in value:
                    if(data - int(data)==0):
                        data = int(data)
                    f.write(str(data) + ' ')
                f.write('\n')

    if plot:
        plt.imshow(img)
        plt.scatter(points[:, 0], points[:, 1],
                    # c=(label - min(label))/max(label),
                    c = (points[:, 2] - min(points[:, 2])/max(points[:, 2])),
                    s=10, # marker size
                    lw=0.0)
        plt.show()
    else:
        print("frame {0} process finished.".format(frame))

# for i in range(252):
#     plot_to_camera(i, False, True)
plot_to_camera(18, True, True)