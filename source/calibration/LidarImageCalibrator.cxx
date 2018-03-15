#include <iostream>
#include <vector>
#include <unordered_set>
#include <cmath>

#include <opencv2/calib3d.hpp>

#include "calibration/LidarImageCalibrator.h"
#include "calibration/ChessBoardSampleConsensus.hxx"

namespace oitk
{

    LidarImageCalibrator::LidarImageCalibrator(unsigned int pattern_size)
        : _pattern_size(pattern_size) { }

    inline LidarImageCalibrator::PointsMatrix& LidarImageCalibrator::sort_corners(
        const LidarImageCalibrator::PointsMatrix& corners)
    {
        PointsMatrix sorted(corners.rows(), 2);
        PointsMatrix::Index leftist_idx = 0;
        PointsMatrix::Index sorted_idx = 0;
        unsigned int edge_counter = 0;

        std::unordered_set<PointsMatrix::Index> left_filter;
        for (int i = 0; i < corners.rows(); i++) left_filter.emplace(i);

        while (!left_filter.empty())
        {
            auto bottom_bound = sorted(sorted_idx, 1);
            for (auto row : left_filter)
            {
                if (edge_counter > 0 && corners(row, 1) <= bottom_bound)
                    continue;
                if (corners(row, 0) < corners(leftist_idx, 0))
                    leftist_idx = row;
            }

            sorted.row(sorted_idx++) = corners.row(leftist_idx);
            left_filter.erase(leftist_idx);
            if (++edge_counter == _pattern_size) edge_counter = 0;
        }
        return sorted;
    }

    Eigen::Matrix4f LidarImageCalibrator::calibrate(const cv::Mat image, const pcl::PointCloud<pcl::PointXYZ> cloud)
    {
        // Find chessboard in image
        std::vector<cv::Point> image_corners;
        cv::Size pattern(_pattern_size, _pattern_size);
        if(!cv::findChessboardCorners(image, pattern, image_corners))
            std::cerr << "Failed to find the chessboard" << std::endl;

        // Sort corners in image
        PointsMatrix image_corners_matrix(image_corners.size(), 2);
        for (int i = 0; i < image_corners.size(); i++)
        {
            image_corners_matrix(i, 0) = std::ceil(image_corners[i].x);
            image_corners_matrix(i, 1) = std::ceil(image_corners[i].y);
        }
        image_corners_matrix = sort_corners(image_corners_matrix);

        // Find chessboard in point cloud

        // Sort corners in point cloud

        Eigen::Matrix4f m;
        return m;
    }
}
