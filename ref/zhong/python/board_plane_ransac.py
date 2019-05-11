import os, sys
import numpy as np
import numpy.random as npr
import numpy.linalg as npl
from shutil import copyfile
from scipy.io import loadmat, savemat
from scipy.optimize import minimize, basinhopping
from math import pi
# import pcl

# Wait for analyse: 30
# Error Candidate:5, 21-26, 31-32，【35人和板子站成一个平面了; 29板子总是会连上一个人】

# dataroot = 'E:/Dataset/cal_20170531/'
# dataprefix = '20170531_'
dataroot = '/media/jacobz/Library/MCity/extri_data/extri_bags_0505/all_3/'
# dataroot = 'E:/Dataset/20171125_crop/'
dataprefix = '0000'
frame_num = 1
printiter = False
direct_use = False

normal_knum = 20 # 20 default
grow_knum = 30 # 30 default, important, 当板子识别有问题时可以尝试调大
smooth_thres = 8 # degree, 6.0 default, important
curvature_thres = 2 # 1.5 default

plane_ransac_thres = 0.03 # m
board_ransac_thres = 0.005 # m
expected_count_thres = 2 # 1.5 default, important
# expected_count_thres_near = 1.5
# expected_count_thres_dist = 1.2
plane_inlier_thres = 0.85
board_inlier_thres = 0.75

res_vertical = 1.33 # degree
res_horizontal = 0.16 # degree approximate

cut_max = 2
cut_min = 1

dataidx = 11
run_all_frames = False
skip_step_one = False
skip_step_two = False
skip_step_three = False

################
#    PCL IO    #
################
pcd_header = '''# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {0}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {0}
DATA ascii
'''

def load_pcd(cloud_dir):
    return np.loadtxt(cloud_dir, skiprows=11)[:,:3]

def save_pcd(cloud_dir, data):
    with open(cloud_dir, 'w') as cloud_fout:
        cloudlist = list(data)
        cloud_fout.write(pcd_header.format(len(cloudlist)))
        for x,y,z in cloudlist:
            cloud_fout.write("%f %f %f\n" % (x,y,z))

def displayregions():
    data = load_pcd(datapath)

    indices = filter(lambda name: name.endswith('.idx'), os.listdir(dataroot + 'planes/'))
    for idx in indices:
        if not idx.startswith(str(dataidx) + '_'):
            continue
        idxarray = np.loadtxt(dataroot + 'planes/' + idx).astype(int)
        save_pcd(dataroot + 'planes/' + idx + '.pcd', data[idxarray])

################
#   Step Two   #
################
def randFloat():
    '''
    give a random float between -0.5 and 0.5
    '''
    return (npr.randint(1000) - 500) / 1000

def computeBounding(p1, p2, p3):
    '''
    Compute the minimal bounding square of given three points
    '''
    # p1 p2 is the base, p3 is the vortex
    base = p2 - p1
    leg1 = p3 - p1
    leg2 = p3 - p2
    proj1 = leg1.dot(base)
    proj2 = leg2.dot(base)
    lenbase = npl.norm(base)
    if proj1 > 0 and proj2 >= 0:
        width = lenbase + proj2
    elif proj1 <= 0 and proj2 < 0:
        width = lenbase - proj1
    else:
        width = lenbase
    height = npl.norm(leg2 - base*proj2/lenbase)

    return width, height

def ransac_plane(data, thresDist = 0.01, iters = 10000):
    num = len(data)
    bestInNum = 0
    bestInIdx = []
    bestParameter = [0, 0, 0]
    bestdist = 0
    realiter = 0
    while realiter < iters:

        # Get Samples
        idx = npr.permutation(num)
        p1, p2, p3 = data[idx[:3],:]
        p2p1 = p2 - p1
        p3p1 = p3 - p1
        
        # Check for collinearity
        dy1dy2 = p2p1 / p3p1
        if dy1dy2[0] == dy1dy2[1] and dy1dy2[2] == dy1dy2[1]:
            continue
        # Estimate tile direction
        # TODO: Compute smallest retangle here, but it may be not the smallest square.
        w1 = max(computeBounding(p1, p2, p3))
        w2 = max(computeBounding(p2, p3, p1))
        w3 = max(computeBounding(p3, p1, p2))
        widths = (w1,w2,w3)
        if(min(widths) > 1.1): #larger than the plain border
            continue

        # Compute plane parameters
        normvec = [0, 0, 0]
        normvec[0] = p2p1[1] * p3p1[2] - p2p1[2] * p3p1[1]
        normvec[1] = p2p1[2] * p3p1[0] - p2p1[0] * p3p1[2]
        normvec[2] = p2p1[0] * p3p1[1] - p2p1[1] * p3p1[0]
        normvec = np.array(normvec)
        normvec = normvec/npl.norm(normvec)
        const = -normvec.dot(p1)
        parameter = np.hstack((normvec, const))
        
        # Compute distance
        dist = np.abs(data.dot(normvec.reshape(-1, 1)) + const) # distance to the plane


        #Count inliers
        inlieridx = np.where(dist < thresDist)[0]
        inliernum = inlieridx.size
        inlierdist = sum(dist[inlieridx])
        if inliernum > bestInNum or (inliernum == bestInNum and inlierdist < bestdist):
            bestInNum = inliernum
            bestInIdx = inlieridx
            bestParameter = parameter
            bestdist = inlierdist

            if printiter:
                print('iteration ' + str(realiter) + '..\tbestnum = ' + str(bestInNum))

        realiter += 1
    
    return bestParameter, bestInNum, bestInIdx.astype('int')

def ransac_board(data, planeParam, thresDist = 0.01, iters = 10000):
    num = len(data)
    randrate = 1

    bestInNum = 0
    bestInIdx = np.array([])
    bestParameter = [0, 0, 0]
    bestdist = 0
    bestfitness = 0
    realiter = 0
    while realiter < iters:
        # Get Samples
        idx = npr.permutation(num)
        p1, p2 = data[idx[:2],:]
        if(npl.norm(p2 - p1) > 1.6): #larger than the plain border
            continue

        # Compute plane parameters
        origin = (p1 + p2) / 2
        origin += (p1-origin) * randFloat()
        origin += (p2-origin) * randFloat()
        diameter = p2 - p1
        normvec = planeParam[:3]
        adiameter = np.cross(diameter, normvec)
        # Randomize x-direction
        xdirection = diameter/npl.norm(diameter) * randFloat() * randrate + adiameter/npl.norm(diameter) * randFloat() * randrate 
        xdirection = xdirection/npl.norm(xdirection)
        parameter = np.hstack((origin, xdirection))
        ydirection = np.cross(normvec, xdirection)
        ydirection = ydirection/npl.norm(ydirection)
        
        # Compute distance
        # assert normvec.dot(xdirection) < 1e-7
        # assert normvec.dot(ydirection) < 1e-7
        # assert xdirection.dot(ydirection) < 1e-7
        normvec = normvec.reshape(-1, 1) # normal vector
        xdirection = xdirection.reshape(-1, 1) # x-axis vector
        ydirection = ydirection.reshape(-1, 1) # y-axis vector
        originrepeat = origin.reshape(1,-1).repeat(len(data), axis=0)
        zdist = (data-originrepeat).dot(normvec) # distance to the plane
        projection = (data-originrepeat) - zdist*normvec.T # projection to the plane
        projection = np.abs(np.hstack((projection.dot(xdirection), projection.dot(ydirection))))

        xydist = np.zeros(projection.shape)
        dist = np.zeros(len(projection))
        xydist[projection > 0.55] += projection[projection > 0.55] - 0.55
        dist += np.sqrt(np.sum(xydist*xydist, axis=1)) # out of board
        xydist = np.zeros(projection.shape)
        xydist[projection > 0.3] += 0.1 - np.abs(0.4 - projection[projection > 0.3])
        xydist[projection < 0.1] += 0.1 - projection[projection < 0.1]
        #dist = np.sqrt(np.sum(xydist*xydist, axis=1) + (zdist*zdist).reshape(-1))
        inboard = (projection[:,0] < 0.5) & (projection[:,1] < 0.5) & (xydist[:,0] > 0) & (xydist[:,1] > 0)
        dist[inboard] += np.min(xydist, axis=1)[inboard]

        #Count inliers
        inlieridx = np.where(dist < thresDist)[0]
        inliernum = inlieridx.size
        inlierdist = sum(dist[inlieridx])

        #Compute fitness
        projection = projection[inlieridx] # count only inliers
        countx = []
        countx.append(np.sum(projection[:, 0] < 0.1))
        countx.append(np.sum((projection[:, 0] >= 0.1) & (projection[:, 0] < 0.2)) * 2 / 5.5)
        countx.append(np.sum((projection[:, 0] >= 0.2) & (projection[:, 0] < 0.3)) * 2 / 5.5)
        countx.append(np.sum((projection[:, 0] >= 0.3) & (projection[:, 0] < 0.4)))
        countx.append(np.sum((projection[:, 0] >= 0.4) & (projection[:, 0] < 0.5)))
        countx.append(np.sum((projection[:, 0] >= 0.5) & (projection[:, 0] < 0.55)) * 2 / 5.5 * 2)
        county = []
        county.append(np.sum(projection[:, 1] < 0.1))
        county.append(np.sum((projection[:, 1] >= 0.1) & (projection[:, 1] < 0.2)) * 2 / 5.5)
        county.append(np.sum((projection[:, 1] >= 0.2) & (projection[:, 1] < 0.3)) * 2 / 5.5)
        county.append(np.sum((projection[:, 1] >= 0.3) & (projection[:, 1] < 0.4)))
        county.append(np.sum((projection[:, 1] >= 0.4) & (projection[:, 1] < 0.5)))
        county.append(np.sum((projection[:, 1] >= 0.5) & (projection[:, 1] < 0.55)) * 2 / 5.5 * 2)
        fitness = np.abs(np.var(countx) - np.var(county)) # less the better

        # compute bounding fitness
        if inliernum > 0:
            xmax, ymax = np.max(projection, axis=0)
            xmin, ymin = np.min(projection, axis=0)
            fitness /= np.exp(((xmax - xmin) * (ymax - ymin) / 1.21))

        if inliernum > bestInNum or (inliernum == bestInNum and inlierdist < bestdist):
            bestInNum = inliernum
            bestInIdx = inlieridx
            bestParameter = parameter
            bestdist = inlierdist
            bestfitness = fitness

            if printiter:
                print('iteration ' + str(realiter) + '..\tbestnum = ' + str(bestInNum))

        realiter += 1
    
    return bestParameter, bestInNum, bestInIdx.astype('int'), bestdist, fitness


def run_step_two(planeiters = 5000, boarditers = 5000, thresDist = 0.01):
    data = load_pcd(datapath)
    print(data)
    print('Points loaded.')

    maxr = -float('inf')
    bestplane = None
    bestparam = None
    bestnum = 0
    bestidx = None

    nums = []
    expectcoeff = 85 / 121 * 1.1**2 / (res_horizontal * res_vertical * pi**2 / 180**2)

    indices = filter(lambda name: name.endswith('.idx'), os.listdir(dataroot + 'planes/'))
    for idx in indices:  
        if not idx.startswith(str(dataidx) + '_'):
            continue
        idxarray = np.loadtxt(dataroot + 'planes/' + idx)
        idxlen = len(idxarray)
        print('Plane with ' + str(idxlen) + ' points.')
        points = data[idxarray.astype(int)]

        mdist = np.mean(np.linalg.norm(points, axis=1))
        # print(mdist)
        expectcount = expectcoeff / mdist**2
        if idxlen > (expectcount * expected_count_thres) and not direct_use: # much more than expect
            nums.append((idx,'too many points','nan',idxlen,expectcount))
            continue
        if idxlen < (expectcount / expected_count_thres) and not direct_use: # less more than expect
            nums.append((idx,'too few points','nan',idxlen,expectcount))
            continue

        plane, nump, planeidx = ransac_plane(points, iters = planeiters, thresDist = plane_ransac_thres)
        print('Found ' + str(nump) + ' / ' + str(idxlen) + ' plane inliers finally.')

        if nump / idxlen < plane_inlier_thres and not direct_use: # like a smooth surface rather than plane
            nums.append((idx,'plane fails ' + str(nump / idxlen),nump,idxlen,expectcount))
            continue

        param, num, subidx, dist, fitness = ransac_board(points[planeidx], plane, iters = boarditers, thresDist = thresDist)
        print('Found ' + str(num) + ' / ' + str(idxlen) + ' board inliers finally.')

        if np.isnan(param).any(): # invalid computation
            nums.append((idx,'nan params',num,idxlen,expectcount))
            continue

        if num / idxlen < board_inlier_thres and not direct_use: # Too many outliers that it seems like wall
            nums.append((idx,'inlier fails ' + str(num / idxlen),num,idxlen,expectcount))
            continue

        # if num > (expectcount * expected_count_thres): # much more than expect
        #     nums.append((idx,'too many inliers','nan',idxlen,expectcount))
        #     continue
        # if num < (expectcount / expected_count_thres): # less more than expect
        #     nums.append((idx,'too few inliers','nan',idxlen,expectcount))
        #     continue

        # if num/idxlen > maxr or (num/idxlen == maxr and num > bestnum):
        # r = num * np.sqrt(np.mean(npl.norm(points, axis=1))) # makes distance doesn't matter by means of density
        r = -fitness
        if r > maxr:
            maxr = r
            bestplane = plane
            bestparam = param
            bestnum = num
            bestidx = idxarray[planeidx[subidx]]
        
        nums.append((idx,r,num,idxlen,expectcount))
    
    print('-----------result summary-------------')
    for value in nums:
        print(value[0] + ': ' + str(value[1]) + ' , ' + str(value[2]) + ' / ' + str(value[3]) + ' %.1f expected' % value[4])
    
    if maxr == -float('inf'):
        print ('FAILED to find a best model')
    else:
        print('The best model has ' + str(bestnum) + ' points (' + str(maxr) + ')')
        np.savetxt('bestParam.txt', np.hstack((bestplane[:3], bestparam)))
        np.savetxt('inliersIndex.txt', bestidx, '%d')
        np.savetxt(dataroot + 'params/' + str(dataidx) + '_bestParam.txt', np.hstack((bestplane[:3], bestparam)))
        np.savetxt(dataroot + 'params/' + str(dataidx) + '_inliersIndex.txt', bestidx, '%d')

################
#  Step Three  #
################
def to_nearest_line(partial):
    '''
    minus distance indicate the point is outside the board range (>0.5)
    '''
    print(partial)
    if partial <= -.55:
        assert -(-.55 - partial) < 0
        return -.55, -(-.55 - partial)
    elif -.55 < partial and partial <= -.4:
        assert partial + .5 > 0
        return -.5, partial + .5
    elif -.4 < partial and partial <= -.2:
        assert -.3 - partial > 0
        return -.3, -.3 - partial
    elif -.2 < partial and partial <= 0:
        assert partial +.1 > 0
        return -.1, partial +.1
    elif 0 < partial and partial <= .2:
        assert .1 - partial
        return .1, .1 - partial
    elif .2 < partial and partial <= .4:
        assert partial -.3
        return .3, partial -.3
    elif .4 < partial and partial <= .55:
        assert .5 - partial
        return .5, .5 - partial
    elif .55 < partial:
        assert -(partial - .55) < 0
        return .55, -(partial - .55)

def nearest_point(point):
    '''
    get the position of the nearest point in the board
    '''
    x, y = point
    nx, xdist = to_nearest_line(x)
    ny, ydist = to_nearest_line(y)
    if xdist >= 0 and ydist >= 0:
        if xdist < ydist:
            return nx, y
        else:
            return x, ny
    elif xdist < 0 and ydist < 0:
        return nx, ny
    elif xdist < 0: # ydist >= 0
        return nx, y
    else: # xdist >= 0 and ydist < 0
        return x, ny

def similarity_transform(from_points, to_points):
    
    # assert len(from_points.shape) == 2, \
    #     "from_points must be a m x n array"
    # assert from_points.shape == to_points.shape, \
    #     "from_points and to_points must have the same shape"

    N, m = from_points.shape
    
    mean_from = from_points.mean(axis = 0)
    mean_to = to_points.mean(axis = 0)
    
    delta_from = from_points - mean_from # N x m
    delta_to = to_points - mean_to       # N x m
    
    sigma_from = (delta_from * delta_from).sum(axis = 1).mean()
    sigma_to = (delta_to * delta_to).sum(axis = 1).mean()
    
    cov_matrix = delta_to.T.dot(delta_from) / N
    
    U, d, V_t = np.linalg.svd(cov_matrix, full_matrices = True)
    cov_rank = np.linalg.matrix_rank(cov_matrix)
    S = np.eye(m)
    
    if cov_rank >= m - 1 and np.linalg.det(cov_matrix) < 0:
        S[m-1, m-1] = -1
    elif cov_rank < m-1:
        raise ValueError("colinearility detected in covariance matrix:\n{}".format(cov_matrix))
    
    R = U.dot(S).dot(V_t)
    # c = (d * S.diagonal()).sum() / sigma_from
    c = 1 # no scaling
    t = mean_to - c*R.dot(mean_from)
    
    return c*R, t

dist_method = 'sim' # 'sim' / 'brute'

def board_dist_3d(params, data, output=False):

    origin = params[:3]
    normvec = np.append(params[3:5], [1])
    normvec = normvec / npl.norm(normvec)
    globalxdir = np.cross(normvec, [1, 0, 0])
    globalydir = np.cross(normvec, globalxdir)
    globalxdir = globalxdir / npl.norm(globalxdir)
    globalydir = globalydir / npl.norm(globalydir)
    theta = params[5]
    xdirection = globalxdir * np.cos(theta) + globalydir * np.sin(theta)
    ydirection = np.cross(normvec, xdirection)
    xdirection = xdirection / npl.norm(xdirection)
    ydirection = ydirection / npl.norm(ydirection)
    
    normvec = normvec.reshape(-1, 1) # normal vector
    xdirection = xdirection.reshape(-1, 1) # x-axis vector
    ydirection = ydirection.reshape(-1, 1) # y-axis vector
    originrepeat = origin.reshape(1,-1).repeat(len(data), axis=0)
    zdist = (data-originrepeat).dot(normvec) # distance to the plane
    projection = (data-originrepeat) - zdist*normvec.T # projection to the plane
    projection = np.hstack((projection.dot(xdirection), projection.dot(ydirection)))
    projection_abs = np.abs(projection)
    inhole = (projection_abs[:,0] > 0.55) | (projection_abs[:,1] > 0.55)
    inhole |= ((projection_abs[:,0] < 0.1) | ((0.3 < projection_abs[:,0]) & (projection_abs[:,0] < 0.5))) &\
                ((projection_abs[:,1] < 0.1) | ((0.3 < projection_abs[:,1]) & (projection_abs[:,1] < 0.5)))
    
    if dist_method is 'sim':
        projection = projection_abs
        # out of board
        xydist = np.zeros(projection.shape)
        dist = np.zeros(len(projection))
        xydist[projection > 0.55] += projection[projection > 0.55] - 0.55
        dist += np.sqrt(np.sum(xydist*xydist, axis=1))
        # in board
        xydist = np.zeros(projection.shape)
        xydist[projection > 0.3] += 0.1 - np.abs(0.4 - projection[projection > 0.3])
        xydist[projection < 0.1] += 0.1 - projection[projection < 0.1]
        inboard = (projection[:,0] < 0.5) & (projection[:,1] < 0.5) & (xydist[:,0] > 0) & (xydist[:,1] > 0)
        dist[inboard] += np.min(xydist, axis=1)[inboard]

    elif dist_method is 'brute':
        # nearest_point is invalid if one direction is beyond 0.55 and another is in non-hole area (e.g. =0.2)
        odist = np.array([nearest_point(point) for point in projection[inhole]])
        odist -= projection[inhole]
        dist = np.zeros(len(projection))
        dist[inhole] = np.sum(odist*odist, axis=1)
        if output:
            np.savetxt('outer.txt', inhole)
    
    dist = dist[:, np.newaxis]
    # print(origin - np.mean(data, axis=0), np.sum(np.sqrt(dist*dist + zdist*zdist)))
    return np.sum(np.sqrt(dist + zdist*zdist)) * np.sqrt(np.sum(inhole))

def run_step_three(blur_thres_plane=0.05, blur_thres_thick=.05, max_iter=5000, method='icp'):
    '''
    blur_thres: blur distance threshold in meters
    '''
    bestparam = np.loadtxt(dataroot + 'params/' + str(dataidx) + '_bestParam.txt')
    normvec = bestparam[:3]
    origin = bestparam[3:6]
    xdirection = bestparam[6:]
    ydirection = np.cross(normvec, xdirection)
    xdirection = xdirection / np.linalg.norm(xdirection)
    ydirection = ydirection / np.linalg.norm(ydirection)

    # cut plane points from original point cloud instead of using inliers of last step
    data = load_pcd(datapath)
    originrepeat = origin.reshape(1,-1).repeat(len(data), axis=0)
    zdist = (data-originrepeat).dot(normvec)
    zmask = np.abs(zdist) < blur_thres_thick
    
    zdist = zdist[zmask]
    projection = (data[zmask]-originrepeat[zmask]) - zdist[:, np.newaxis]*normvec[np.newaxis, :]
    projection = np.vstack((projection.dot(xdirection), projection.dot(ydirection))).T
    xymask = np.all(np.abs(projection) <= (0.55 + blur_thres_plane), axis=1)
    
    projection = projection[xymask]
    cloud = data[zmask][xymask]
    save_pcd('boxed.pcd', cloud)

    if method is 'icp':
        iter = 0
        R = np.eye(2)
        T = np.zeros(2)
        while(iter < max_iter):
            projection_transferred = projection.dot(R) + T
            projection_abs = np.abs(projection_transferred)

            inhole = (projection_abs[:,0] > 0.55) | (projection_abs[:,1] > 0.55)
            inhole |= ((projection_abs[:,0] < 0.1) | ((0.3 < projection_abs[:,0]) & (projection_abs[:,0] < 0.5))) &\
                    ((projection_abs[:,1] < 0.1) | ((0.3 < projection_abs[:,1]) & (projection_abs[:,1] < 0.5)))
            if np.sum(inhole) < 3: break

            print('iter %d / %d' % (np.sum(inhole), len(inhole)))
            # cloud = pcl.PointCloud(data[zmask][xymask][inhole], fields=['x', 'y', 'z'], copy=False)
            # save_pcd('box_hole.pcd', cloud)
            # break

            source_cloud = projection[inhole]
            target_cloud = np.array([nearest_point(point) for point in source_cloud])
            nR, nT = similarity_transform(source_cloud, target_cloud)
            R = nR.dot(R)
            T = nR.dot(T) + nT

            iter += 1
        
        origin -= xdirection * T[0] + ydirection * T[1]
        # http://www.cnblogs.com/graphics/archive/2012/08/10/2627458.html
        costheta, sintheta = R[0]
        xdirection = xdirection * costheta + ydirection * sintheta

    elif method is 'optimize':
        nx, ny, _ = normvec / normvec[-1]
        globalxdir = np.cross(normvec, [1, 0, 0])
        globalydir = np.cross(normvec, globalxdir)
        theta = np.arctan2(np.dot(globalydir, xdirection) / np.linalg.norm(globalydir), np.dot(globalxdir, xdirection) / np.linalg.norm(globalxdir))

        init_params = np.append(origin, [nx, ny, theta])
        origin_limit = [origin - .05, origin + .05]
        normal_limit = np.sort([[nx * .95, ny * .95], [nx * 1.05, ny * 1.05]], axis=0)
        theta_limit = [[theta - .05], [theta + .05]]
        params_limit = np.hstack((origin_limit, normal_limit, theta_limit)).T
        print('Initial Distance:', board_dist_3d(init_params, data[zmask][xymask]))
        minimizer = dict(method='L-BFGS-B', args=data[zmask][xymask], options={'maxiter': max_iter})
        # minimizer = dict(method='L-BFGS-B', args=data[zmask][xymask], bounds=params_limit, options={'maxiter': max_iter})
        opti_result = basinhopping(board_dist_3d, init_params, minimizer_kwargs=minimizer, niter=5, disp=True)
        opti_params = opti_result.x
        # print(opti_result)
        print('Successful:', opti_result.lowest_optimization_result.success)

        origin = opti_params[:3]
        normvec = np.append(opti_params[3:5], [1])
        normvec = normvec / npl.norm(normvec)
        globalxdir = np.cross(normvec, [1, 0, 0])
        globalydir = np.cross(normvec, globalxdir)
        globalxdir = globalxdir / npl.norm(globalxdir)
        globalydir = globalydir / npl.norm(globalydir)
        theta = opti_params[5]
        xdirection = globalxdir * np.cos(theta) + globalydir * np.sin(theta)
        xdirection = xdirection / np.linalg.norm(xdirection)

        # board_dist_3d(opti_params, data[zmask][xymask], True)
        # inhole = np.loadtxt('outer.txt').astype(bool)
        # save_pcd('outer.pcd', cloud[inhole], False)

    np.savetxt('bestParam.txt', np.hstack((normvec, origin, xdirection)))
    np.savetxt(dataroot + 'params/' + str(dataidx) + '_bestParamRefined.txt', np.hstack((normvec, origin, xdirection)))
    print('-----------refined model saved-------------')

#################
# Visualization #
#################
def visualize():
    # points = loadmat(datapath)['points']
    points = load_pcd(datapath)
    with open('origin.pcd', 'w') as f:
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
                    if(data - int(data)==0):
                        data = int(data)
                    f.write(str(data) + ' ')
                f.write('\n')
    
    inlieridx = np.loadtxt('inliersIndex.txt').astype('int')
    points = points[inlieridx,:]
    with open('filtered.pcd', 'w') as f:
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
                    if(data - int(data)==0):
                        data = int(data)
                    f.write(str(data) + ' ')
                f.write('\n')

def drawbord():
    params = np.loadtxt('bestParam.txt')
    normvec = params[:3]
    origin = params[3:6]
    xdirection = params[6:9]
    xdirection = xdirection / npl.norm(xdirection)
    ydirection = np.cross(normvec, xdirection)
    ydirection = ydirection / npl.norm(ydirection)
    num = 50
    points = []

    for i in range(num):
        points.append(origin + xdirection * 0.55 + ydirection * 0.55 * i/num)
        points.append(origin + xdirection * 0.55 - ydirection * 0.55 * i/num)
        points.append(origin - xdirection * 0.55 + ydirection * 0.55 * i/num)
        points.append(origin - xdirection * 0.55 - ydirection * 0.55 * i/num)
        points.append(origin + ydirection * 0.55 + xdirection * 0.55 * i/num)
        points.append(origin + ydirection * 0.55 - xdirection * 0.55 * i/num)
        points.append(origin - ydirection * 0.55 + xdirection * 0.55 * i/num)
        points.append(origin - ydirection * 0.55 - xdirection * 0.55 * i/num)

        points.append(origin + xdirection * 0.5 + ydirection * 0.5 * i/num)
        points.append(origin + xdirection * 0.5 - ydirection * 0.5 * i/num)
        points.append(origin - xdirection * 0.5 + ydirection * 0.5 * i/num)
        points.append(origin - xdirection * 0.5 - ydirection * 0.5 * i/num)
        points.append(origin + ydirection * 0.5 + xdirection * 0.5 * i/num)
        points.append(origin + ydirection * 0.5 - xdirection * 0.5 * i/num)
        points.append(origin - ydirection * 0.5 + xdirection * 0.5 * i/num)
        points.append(origin - ydirection * 0.5 - xdirection * 0.5 * i/num)

        points.append(origin + xdirection * 0.3 + ydirection * 0.5 * i/num)
        points.append(origin + xdirection * 0.3 - ydirection * 0.5 * i/num)
        points.append(origin - xdirection * 0.3 + ydirection * 0.5 * i/num)
        points.append(origin - xdirection * 0.3 - ydirection * 0.5 * i/num)
        points.append(origin + xdirection * 0.1 + ydirection * 0.5 * i/num)
        points.append(origin + xdirection * 0.1 - ydirection * 0.5 * i/num)
        points.append(origin - xdirection * 0.1 + ydirection * 0.5 * i/num)
        points.append(origin - xdirection * 0.1 - ydirection * 0.5 * i/num)

        points.append(origin + ydirection * 0.3 + xdirection * 0.5 * i/num)
        points.append(origin + ydirection * 0.3 - xdirection * 0.5 * i/num)
        points.append(origin - ydirection * 0.3 + xdirection * 0.5 * i/num)
        points.append(origin - ydirection * 0.3 - xdirection * 0.5 * i/num)
        points.append(origin + ydirection * 0.1 + xdirection * 0.5 * i/num)
        points.append(origin + ydirection * 0.1 - xdirection * 0.5 * i/num)
        points.append(origin - ydirection * 0.1 + xdirection * 0.5 * i/num)
        points.append(origin - ydirection * 0.1 - xdirection * 0.5 * i/num)
        
        points.append(origin + xdirection * 0.2 + ydirection * 0.5 * i/num)
        points.append(origin + xdirection * 0.2 - ydirection * 0.5 * i/num)
        points.append(origin - xdirection * 0.2 + ydirection * 0.5 * i/num)
        points.append(origin - xdirection * 0.2 - ydirection * 0.5 * i/num)
        points.append(origin + ydirection * 0.2 + xdirection * 0.5 * i/num)
        points.append(origin + ydirection * 0.2 - xdirection * 0.5 * i/num)
        points.append(origin - ydirection * 0.2 + xdirection * 0.5 * i/num)
        points.append(origin - ydirection * 0.2 - xdirection * 0.5 * i/num)

    with open('frame.pcd', 'w') as f:
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
                    # if data == float('nan'):
                    #     print('warning: nan data')
                    #     data = 'nan'
                    # elif(data - int(data)==0):
                    #     data = int(data)
                    f.write(str(data) + ' ')
                f.write('\n')

def generate_feature_points():
    bestparam = np.loadtxt(dataroot + 'params/' + str(dataidx) + '_bestParam.txt')
    normvec = bestparam[:3]
    origin = bestparam[3:6]
    xdirection = bestparam[6:]
    ydirection = np.cross(normvec, xdirection)
    xdirection = xdirection / np.linalg.norm(xdirection)
    ydirection = ydirection / np.linalg.norm(ydirection)
    offsets = [-0.5, -0.3, -0.1, 0.1, 0.3, 0.5]
    points = []
    sorted_points = []
    temp_points = points
    for xoff in offsets:
        for yoff in offsets:
            points.append(xdirection * xoff + ydirection * yoff)
    points = np.array(points)
    counter = 0
    while len(points) > 0:
        if counter > 0:
            mask = points[:, 2] > sorted_points[-1][2]
            temp_points = points[mask]
            left = np.argmax(temp_points[:, 1])
            sorted_points.append(temp_points[left])
            points = np.delete(points, np.arange(len(points))[mask][left], axis=0)
            counter += 1
            if counter == 6:
                counter = 0
        else:
            left = np.argmax(points[:, 1])
            sorted_points.append(points[left])
            points = np.delete(points, left, axis=0)
            counter += 1

    sorted_points = np.array(sorted_points) + origin
    np.savetxt("features.txt", sorted_points)

    sorted_points = sorted_points[:5]
    with open('features.pcd', 'w') as f:
            headers = ["VERSION .7\n",
                       "FIELDS x y z\n",
                       "SIZE 4 4 4\n",
                       "TYPE F F F\n",
                       "COUNT 1 1 1\n",
                       "WIDTH " + str(len(sorted_points)) + '\n',
                       "HEIGHT 1\n",
                       "VIEWPOINT 0 0 0 1 0 0 0\n",
                       "POINTS " + str(len(sorted_points)) + '\n',
                       "DATA ascii\n"]
            f.writelines(headers)
            for value in sorted_points:
                for data in value:
                    f.write(str(data) + ' ')
                f.write('\n')

################
#     Main     #
################

if run_all_frames:
    for i in range(frame_num):
        # if i < 34: continue
        dataidx = i
        print('---------- processing %d ------------' % i)
        try:
            datapath = dataroot + dataprefix + str(dataidx) + '.pcd'
            cloud = load_pcd(datapath)
            clouddata = cloud
            cloud_cut = cloud
            # cloud_cut = cloud[(clouddata[:,0] < 0)&(clouddata[:,1] < -cut_max * clouddata[:,0])&(clouddata[:,1] > cut_min * clouddata[:,0])&(clouddata[:,2] > -1.8)]
            cloud_cut = cloud_cut[np.linalg.norm(cloud_cut.xyz, axis=1) < 20]
            datapath = dataroot + dataprefix + str(dataidx) + '_cut.pcd'
            save_pcd(datapath, cloud_cut)

            if not skip_step_one:
                if direct_use:
                    np.savetxt(dataroot + 'planes/' + str(dataidx) + '_0.idx', np.arange(len(cloud)), '%d')
                elif os.spawnle(os.P_WAIT, r'D:\编程\Lidar\x64\Debug\display.exe', 'display.exe',
                    str(dataidx), dataroot, dataprefix,
                    str(normal_knum), str(grow_knum), str(smooth_thres), str(curvature_thres),
                    os.environ) == 0:
                    print('Region Growing finished')
                displayregions()
            

            if not skip_step_two:
                run_step_two(thresDist=board_ransac_thres)
                copyfile(dataroot + 'params/' + str(dataidx) + '_bestParam.txt', 'bestParam.txt')
                visualize()
                drawbord()
                copyfile('origin.pcd', dataroot + 'visualize/' + str(dataidx) + '_origin.pcd')
                copyfile('filtered.pcd', dataroot + 'visualize/' + str(dataidx) + '_filtered.pcd')
                copyfile('frame.pcd', dataroot + 'visualize/' + str(dataidx) + '_frame.pcd')
            
            if not skip_step_three:
                run_step_three(max_iter=5000, method='optimize')
                drawbord()
                copyfile('boxed.pcd', dataroot + 'visualize/' + str(dataidx) + '_boxed.pcd')
                copyfile('frame.pcd', dataroot + 'visualize/' + str(dataidx) + '_frame_refined.pcd')
            
            generate_feature_points()
            copyfile('features.txt', dataroot + 'params/' + str(dataidx) + '_features.txt')
        except Exception as e:
            print('------- error: {0} --------'.format(e))

else:
    datapath = dataroot + dataprefix + str(dataidx) + '.pcd'
    cloud = load_pcd(datapath)
    clouddata = cloud
    cloud_cut = cloud
    cloud_cut = cloud[(clouddata[:,0] > 0)&(clouddata[:,1] > -cut_max * clouddata[:,0])&(clouddata[:,1] < cut_min * clouddata[:,0])&(clouddata[:,2] > -1.8)]
    cloud_cut = cloud_cut[np.linalg.norm(cloud_cut, axis=1) < 20]
    datapath = dataroot + dataprefix + str(dataidx) + '_cut.pcd'
    save_pcd(datapath, cloud_cut)

    if not skip_step_one:
        if direct_use:
            np.savetxt(dataroot + 'planes/' + str(dataidx) + '_0.idx', np.arange(len(cloud)), '%d')
        elif os.spawnle(os.P_WAIT, r'/home/jacobz/Calibration/reference_code/zhong/cpp/build/detector', 'detector',
                    str(dataidx), dataroot, dataprefix,
                    str(normal_knum), str(grow_knum), str(smooth_thres), str(curvature_thres),
                    os.environ) == 0:
            print('Region Growing finished')
        displayregions()


    if not skip_step_two:
        run_step_two(boarditers=15000, thresDist=board_ransac_thres)
        drawbord()
        visualize()
        copyfile('origin.pcd', dataroot + 'visualize/' + str(dataidx) + '_origin.pcd')
        copyfile('filtered.pcd', dataroot + 'visualize/' + str(dataidx) + '_filtered.pcd')
        copyfile('frame.pcd', dataroot + 'visualize/' + str(dataidx) + '_frame.pcd')
        copyfile('frame.pcd', 'frame_original.pcd')
    
    if not skip_step_three:
        run_step_three(max_iter=15000, method='optimize')
        drawbord()
        copyfile('boxed.pcd', dataroot + 'visualize/' + str(dataidx) + '_boxed.pcd')
        copyfile('frame.pcd', dataroot + 'visualize/' + str(dataidx) + '_frame_refined.pcd')

    generate_feature_points()
    copyfile('features.txt', dataroot + 'params/' + str(dataidx) + '_features.txt')

    # "D:\Program Files\PCL 1.8.0\bin\pcl_viewer_release.exe" origin.pcd filtered.pcd frame.pcd
    os.spawnle(os.P_WAIT, r'pcl_viewer', 'pcl_viewer', 'origin.pcd filtered.pcd frame.pcd frame_original.pcd', os.environ)