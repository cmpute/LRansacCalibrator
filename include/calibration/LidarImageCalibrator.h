#ifndef OITK_LIDARWITHIMAGE_H
#define OITK_LIDARWITHIMAGE_H

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

namespace oitk
{
    class LidarImageCalibrator
    {
    public:
        LidarImageCalibrator(unsigned int pattern_size);
        Eigen::Matrix4f calibrate(const cv::Mat image, const pcl::PointCloud<pcl::PointXYZ> cloud);

    private:
        typedef Eigen::MatrixX2i PointsMatrix;

        unsigned int _pattern_size;
        PointsMatrix& sort_corners(const PointsMatrix& corners);
    };
}

#endif // OITK_LIDARWITHIMAGE_H