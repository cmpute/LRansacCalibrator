#ifndef NO_COMPILE

#include <iostream>
#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

///const std::string path = "D:\\���\\Lidar\\data\\result\\";
const std::string path = "/home/jacobz/Temp";

int main(int argc, char* argv[])
{
	int counter = 0;
	std::string no = *(new std::string(argv[1]));
	std::string path = *(new std::string(argv[2]));
	std::string prefix = *(new std::string(argv[3]));
	int normalknum = atoi(argv[4]);
	int growknum = atoi(argv[5]);
	double smooththres = atof(argv[6]);
	double curvaturethres = atof(argv[7]);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	std::cout << "Load " << path + prefix + no + "_cut.pcd" << std::endl;
	if (pcl::io::loadPCDFile <pcl::PointXYZ>(path + prefix + no + "_cut.pcd", *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}

	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(normalknum);
	normal_estimator.compute(*normals);

	//pcl::IndicesPtr indices(new std::vector <int>);
	//pcl::PassThrough<pcl::PointXYZ> pass;
	//pass.setInputCloud(cloud);
	//pass.setFilterFieldName("z");
	//pass.setFilterLimits(0.0, 1.0);
	//pass.filter(*indices);

	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(50);
	reg.setMaxClusterSize(1000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(growknum);
	reg.setInputCloud(cloud);
	//reg.setIndices (indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(smooththres / 180.0 * M_PI);
	reg.setCurvatureThreshold(curvaturethres);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);

	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
	//std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
	//std::cout << "These are the indices of the points of the initial" <<
	//	std::endl << "cloud that belong to the first cluster:" << std::endl;
	//while (counter < clusters[0].indices.size())
	//{
	//	std::cout << clusters[0].indices[counter] << ", ";
	//	counter++;
	//	if (counter % 10 == 0)
	//		std::cout << std::endl;
	//}
	//std::cout << std::endl;

	for (counter = 0; counter < clusters.size(); counter++)
	{
		std::stringstream ss;
		ss << path << "planes/" << no << "_" << counter << ".idx";
		cout << ss.str() << endl;
		ofstream clu(ss.str());
		for (auto iter = clusters[counter].indices.begin(); iter != clusters[counter].indices.end(); iter++)
			clu << *iter << endl;
		clu.close();
	}

	//pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	//pcl::visualization::CloudViewer viewer("Cluster viewer");
	//viewer.showCloud(colored_cloud);
	//while (!viewer.wasStopped())
	//{
	//}

}

#endif