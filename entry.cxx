#include <iostream>
#include "calibration/LidarImageCalibrator.h"

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

int main(int argc, char** argv)
{
    pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);

    boost::shared_ptr<cv::Mat> image (new cv::Mat);
    *image = cv::imread(argv[1], cv::IMREAD_UNCHANGED);   // Read the file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<boost::shared_ptr<const cv::Mat>> images { image };
    std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> clouds { cloud };


    pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *cloud);
    pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    oitk::LidarImageCalibrator calibrator(tree);

    calibrator.calibrate(images, clouds,
        // oitk::LidarImageCalibrator::VisualizeType::ImageCorners |
        oitk::LidarImageCalibrator::VisualizeType::PointCloudCorners |
        oitk::LidarImageCalibrator::VisualizeType::PointCloudCorners |
        oitk::LidarImageCalibrator::VisualizeType::PointCloudRegionGrow |
        oitk::LidarImageCalibrator::VisualizeType::PointCloudClusterBoard);
}