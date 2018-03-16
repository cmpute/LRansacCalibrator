#ifndef OITK_LIDARWITHIMAGE_H
#define OITK_LIDARWITHIMAGE_H

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <pcl/pcl_base.h>
#include <pcl/search/search.h>
#include <pcl/point_types.h>

namespace oitk
{
    class LidarImageCalibrator
    {
    private:
        typedef pcl::PointXYZ PointType;
        typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;
        typedef pcl::PointCloud<PointType>::ConstPtr ConstPointCloudPtr;
        typedef boost::shared_ptr<cv::Mat> ImagePtr;
        typedef boost::shared_ptr<const cv::Mat> ConstImagePtr;

    public:
        /** \brief LidarImageCalibrator constructor.
          * \param[in] board_size       length of every side of the chessboard (in meters)
          * \param[in] edge_size        thickness of edge of the chessboard (in meters)
          * \param[in] pattern_size     count of corners per side of the chessboard
          * \param[in] normal_knum      count of nearest points to estimate the normals in point cloud
          * \param[in] grow_knum        count of nearest points to grow planes
          * \param[in] estimated_count_thres threshold of range of estimated point count on the chessboard
          * \param[in] plane_inlier_thres    threshold of ratio of plane RANSAC inliers
          * \param[in] board_inlier_thres    threshold of ratio of board RANSAC inliers
          * \param[in] smooth_thres     threshold of smoothness in region growing
          * \param[in] curvature_thres  threshold of curvature in region growing
          * \param[in] res_vertical     angular vertical resolution of LiDAR (in degrees)
          * \param[in] res_horizental   angular horizental resolution of LiDAR (in degrees)
          * \param[in] searcher         method to search nearest points in point cloud
          */
        LidarImageCalibrator(
            float board_size, float edge_size, int pattern_size,
            int normal_knum, int grow_knum, float estimated_count_thres,
            float plane_inlier_thres, float board_inlier_thres,
            float smooth_thres, float curvature_thres,
            float res_vertical, float res_horizental,
            pcl::search::Search<PointType>::Ptr searcher);

        /** \brief LidarImageCalibrator constructor.
          * \param[in] images       image data
          * \param[in] clouds       point cloud data
          * \param[in] visualize    whether show corner points and calibration result
          */
        Eigen::Matrix4f calibrate(const std::vector<ConstImagePtr> images,
            const std::vector<ConstPointCloudPtr> clouds, bool visualize = false);

    private:
        int _pattern_size, _normal_knum, _grow_knum;
        float _board_size, _edge_size, _estimated_count_thres,
              _plane_inlier_thres, _board_inlier_thres,
              _smooth_thres, _curvature_thres,
              _res_vertical, _res_horizental, // stores in radian
              _pcount_coeff;
        pcl::search::Search<PointType>::Ptr _searcher;

        /** \brief Sort corners in certain order.
          * \param[in] corners      corner points in the board plane (can be approximately projected)
          * \param[out] indices     indices of sorted corner points
          */
        void sortCorners(const Eigen::MatrixX2f& corners, std::vector<int> &indices);

        /** \brief Find chessboard corners in point cloud.
          * \param[in] cloud      point cloud data
          * \param[out] corners   corner points founded
          */
        void findChessboardCorners(ConstPointCloudPtr cloud, Eigen::MatrixX3f& corners);

        float estimatePointCount(float board_distance);
    };
}

#endif // OITK_LIDARWITHIMAGE_H