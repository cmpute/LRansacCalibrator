import sys
sys.path.append('D:\\GitCodes\\pypcl')
import pcl
import pcl.segment as ps
import pcl.features as pf

cloud = pcl.io.loadpcd('E:\\Dataset\\cal_20170531\\20170531_0.pcd')

regiongrow = ps.RegionGrowing(cloud)
nestimate = pf.NormalEstimation(cloud)
nestimate.search_k = 10
normals = nestimate.compute()
regiongrow.input_normals = normals
regions = regiongrow.extract()

counter = 0
for region in regions:
    regioncloud = cloud[region]
    pcl.io.savepcd('temp/' + str(counter) + '.pcd', regioncloud)
    counter += 1

