#ifndef OITK_LIDARWITHIMAGE_H
#define OITK_LIDARWITHIMAGE_H

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <pcl/pcl_base.h>
#include <pcl/search/search.h>
#include <pcl/point_types.h>

#define USE_INNER_GRIDS // TODO: Use corner detector in matlab to avoid this

#define DEFINE_ENUM_FLAG_OPERATORS(ENUMTYPE) \
    inline ENUMTYPE operator | (ENUMTYPE a, ENUMTYPE b) { return ENUMTYPE(((int)a) | ((int)b)); } \
    inline ENUMTYPE &operator |= (ENUMTYPE &a, ENUMTYPE b) { return (ENUMTYPE &)(((int &)a) |= ((int)b)); } \
    inline ENUMTYPE operator & (ENUMTYPE a, ENUMTYPE b) { return ENUMTYPE(((int)a) & ((int)b)); } \
    inline ENUMTYPE &operator &= (ENUMTYPE &a, ENUMTYPE b) { return (ENUMTYPE &)(((int &)a) &= ((int)b)); } \
    inline ENUMTYPE operator ~ (ENUMTYPE a) { return ENUMTYPE(~((int)a)); } \
    inline ENUMTYPE operator ^ (ENUMTYPE a, ENUMTYPE b) { return ENUMTYPE(((int)a) ^ ((int)b)); } \
    inline ENUMTYPE &operator ^= (ENUMTYPE &a, ENUMTYPE b) { return (ENUMTYPE &)(((int &)a) ^= ((int)b)); }

namespace oitk
{
    class LidarImageCalibrator
    {
    public:
        typedef pcl::PointXYZ PointType;

        enum class VisualizeType
        {
            None = 0,
            ImageCorners = 1,
            PointCloudCorners = 2,
            PointCloudRegionGrow = 4,
            PointCloudClusterPlane = 8,
            PointCloudClusterBoard = 16,
        };

    private:
        typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;
        typedef pcl::PointCloud<PointType>::ConstPtr ConstPointCloudPtr;
        typedef boost::shared_ptr<cv::Mat> ImagePtr;
        typedef boost::shared_ptr<const cv::Mat> ConstImagePtr;

    public:
        /** \brief LidarImageCalibrator constructor.
          * \param[in] searcher         method to search nearest points in point cloud
          * \param[in] board_size       length of every side of the chessboard (in meters)
          * \param[in] edge_size        thickness of edge of the chessboard (in meters)
          * \param[in] pattern_size     count of corners per side of the chessboard
          * \param[in] normal_knum      count of nearest points to estimate the normals in point cloud
          * \param[in] grow_knum        count of nearest points to grow planes
          * \param[in] estimated_count_thres threshold of range of estimated point count on the chessboard
          * \param[in] plane_inlier_thres    threshold of proportion of plane RANSAC inliers
          * \param[in] board_inlier_thres    threshold of proportion of board RANSAC inliers
          * \param[in] plane_dist_thres    threshold of inlier distance of plane RANSAC inliers
          * \param[in] board_dist_thres    threshold of inlier distance (in plane) of board RANSAC inliers
          * \param[in] smooth_thres     threshold of smoothness in region growing
          * \param[in] curvature_thres  threshold of curvature in region growing
          * \param[in] res_vertical     angular vertical resolution of LiDAR (in degrees)
          * \param[in] res_horizental   angular horizental resolution of LiDAR (in degrees)
          * \param[in] bin_thres        threshold for image binarization
          */
        LidarImageCalibrator(pcl::search::Search<PointType>::Ptr searcher,
            float board_size = 1.1, float edge_size = 0.05, int pattern_size = 6,
            int normal_knum = 20, int grow_knum = 30, float estimated_count_thres = 1.5,
            float plane_inlier_thres = 0.85, float board_inlier_thres = 0.75,
            float plane_dist_thres = 0.03, float board_dist_thres = 0.005,
            float smooth_thres = 6, float curvature_thres = 1.5,
            float res_vertical = 1.33, float res_horizental = 0.2,
            float bin_thres = 60);

        /** \brief LidarImageCalibrator constructor.
          * \param[in] images       image data
          * \param[in] clouds       point cloud data
          * \param[in] visualize    determine what to visualize
          */
        Eigen::Matrix<float, 3, 4>& calibrate(const std::vector<ConstImagePtr> images,
            const std::vector<ConstPointCloudPtr> clouds, VisualizeType visualize = VisualizeType::None);

    private:
        int _pattern_size, _normal_knum, _grow_knum;
        float _board_size, _edge_size, _estimated_count_thres,
            _plane_inlier_thres, _board_inlier_thres,
            _plane_dist_thres, _board_dist_thres,
            _smooth_thres, _curvature_thres,
            _res_vertical, _res_horizental, // stores in radian
            _bin_thres;

        float _pcount_coeff; // cache for estimate count of points on board

        pcl::search::Search<PointType>::Ptr _searcher;

        /** \brief Sort corners in certain order.
          * \param[in] corners      corner points in the board plane (can be approximately projected)
          * \param[out] indices     indices of sorted corner points
          */
        void sortCorners(const Eigen::MatrixX2f& corners, std::vector<int> &indices);

        /** \brief Find chessboard corners in point cloud.
          * \param[in] cloud      point cloud data
          * \param[in] visualize  passed from calibrate()
          * \param[out] corners   corner points founded
          */
        bool findPointCloudCorners(ConstPointCloudPtr cloud, Eigen::MatrixX3f& corners, VisualizeType visualize);

        /** \brief Find chessboard corners in image.
        * \param[in] image      image data
        * \param[in] visualize  passed from calibrate()
        * \param[out] corners   corner points founded
        */
        bool findImageCorners(ConstImagePtr image, Eigen::MatrixX2f& corners, VisualizeType visualize);

        float estimatePointCount(float board_distance);

        Eigen::Matrix<float, 3, 4>& solveHomographyMatrix(Eigen::MatrixX2f& image_corners, Eigen::MatrixX3f& cloud_corners);

        struct InitialOptimizationFunctor;
        struct FinetuneOptimizationFunctor;

    public: /******************** Properties ********************/
        inline void setGrowKNumber(const int k) { _grow_knum = k; };
        inline void setPlaneInliersThreshold(const float proportion) { _plane_inlier_thres = proportion; };
        inline void setPlaneDistanceThreshold(const float distance) { _plane_dist_thres = distance; };
        inline void setBoardInliersThreshold(const float proportion) { _board_inlier_thres = proportion; };
        inline void setBoardDistanceThreshold(const float distance) { _board_dist_thres = distance; };
    };

    DEFINE_ENUM_FLAG_OPERATORS(LidarImageCalibrator::VisualizeType)
}

#endif // OITK_LIDARWITHIMAGE_H