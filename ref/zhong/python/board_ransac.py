import os
import numpy as np
import numpy.random as npr
import numpy.linalg as npl
from scipy.io import loadmat, savemat

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

def distToRect(points, normvec, origin, boundp1, boundp2):
    '''
    Compute distance from PROJECTED points to rectangle defined by corner boundp1 and boundp2
    '''
    pass

def ransac(data, thresDist = 0.01, iters = 10000, probability = 0.99):
    num = len(data)
    k = 1.0
    log_probability = np.log(1 - probability)
    one_over_indices = 1.0 / num
    # k is relatively large in this problem

    bestInNum = 0
    bestInIdx = []
    bestParameter = [0, 0, 0]
    bestdist = 0
    realiter = 0
    while realiter < iters:
        # if k > 1 and realiter > k:
        #     break

        # Get Samples
        idx = npr.permutation(num)
        p1, p2, p3 = data[idx[:3],:]

        # Compute model coefficients
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
        origin = (p1+p2+p3) / 3
        # TODO: randomize here does not make the point possible to fill the triangle
        origin += (p1-origin) * randFloat()
        origin += (p2-origin) * randFloat()
        origin += (p3-origin) * randFloat()
        if w1 is min(widths):
            xdirection = p2p1
        elif w3 is min(widths):
            xdirection = p3p1
        else:
            xdirection = p3 - p2
        # Randomize x-direction
        xdirection = xdirection + np.cross(normvec, xdirection)*randFloat()*0.2
        xdirection = xdirection/npl.norm(xdirection)
        parameter = np.hstack((normvec, origin, xdirection))
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
        projection = np.hstack((np.abs(projection.dot(xdirection)), np.abs(projection.dot(ydirection))))
        xydist = np.zeros(projection.shape)
        xydist[projection > 0.55] += projection[projection > 0.55] - 0.55
        xydist[(projection < 0.5) & (projection > 0.3)] += np.fmin(0.5 - projection[(projection < 0.5) & (projection > 0.3)], 
                                                               projection[(projection < 0.5) & (projection > 0.3)] - 0.3)
        xydist[projection < 0.1] += 0.1 - projection[projection < 0.1]
        dist = np.sqrt(np.sum(xydist*xydist, axis=1) + (zdist*zdist).reshape(-1))

        #Count inliers
        inlieridx = np.where(dist < thresDist)[0]
        inliernum = inlieridx.size
        inlierdist = sum(dist[inlieridx])
        if inliernum > bestInNum or (inliernum == bestInNum and inlierdist < bestdist):
            bestInNum = inliernum
            bestInIdx = inlieridx
            bestParameter = parameter
            bestdist = inlierdist

            w = bestInNum * one_over_indices
            k = log_probability / np.log(1-w**3)

        realiter += 1
        print('iteration ' + str(realiter) + '..\tbestnum = ' + str(bestInNum) + ', k=' + str(k))
    
    return bestParameter, bestInNum, bestInIdx.astype('int')

def run(iters = 5000, thresDist = 0.01, probability = 0.999999):
    data = loadmat(r'D:\编程\Lidar\matlab\01_cut.mat')['points']
    print('Points loaded.')
    param, num, idx = ransac(data, iters = iters, thresDist = thresDist, probability = probability)
    print('Found ' + str(num) + ' inliers finally.')
    np.savetxt('bestParam.txt', param)
    np.savetxt('inliersIndex.txt', idx, '%d')

def visualize():
    points = loadmat(r'D:\编程\Lidar\matlab\01_cut.mat')['points']
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

def cut():
    points = loadmat(r'D:\编程\Lidar\matlab\01.mat')['points']
    print('origin size:' + str(points.size))
    points = points[(points[:,0] > 0) & (points[:,0] < 2), :]
    points = points[(points[:,1] > -.5) & (points[:,1] < 1.5), :]
    points = points[(points[:,2] > 7.5) & (points[:,2] < 8.5), :]
    savemat(r'D:\编程\Lidar\matlab\01_cut.mat',{'points':points})
    print('cut size:' + str(points.size))

def drawbord():
    params = np.loadtxt('bestParam.txt')
    normvec = params[:3]
    origin = params[3:6]
    xdirection = params[6:9]
    ydirection = np.cross(normvec, xdirection)
    num = 50
    points = []
    assert abs(npl.norm(xdirection) - 1) < 1e-9
    assert abs(npl.norm(ydirection) - 1) < 1e-9
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

    with open('box.pcd', 'w') as f:
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


cut()
run(1000000, 0.02)
visualize()
drawbord()

# "D:\Program Files\PCL 1.8.0\bin\pcl_viewer_release.exe" origin.pcd filtered.pcd box.pcd