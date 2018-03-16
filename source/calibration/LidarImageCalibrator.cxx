#include <iostream>
#include <vector>
#include <unordered_set>
#include <cmath>

#include <opencv2/calib3d.hpp>
#include <pcl/common/angles.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/region_growing.h>

#include "calibration/LidarImageCalibrator.h"
#include "calibration/SampleConsensusChessBoard.hxx"

namespace oitk
{
    using namespace std;
    using namespace cv;
    using namespace pcl;

    LidarImageCalibrator::LidarImageCalibrator(
        float board_size, float edge_size, int pattern_size,
        int normal_knum, int grow_knum, float estimated_count_thres,
        float plane_inlier_thres, float board_inlier_thres,
        float smooth_thres, float curvature_thres,
        float res_vertical, float res_horizental,
        search::Search<PointType>::Ptr searcher)
        : _board_size(board_size), _edge_size(edge_size), _pattern_size(pattern_size),
        _normal_knum(normal_knum), _grow_knum(grow_knum), _estimated_count_thres(estimated_count_thres),
        _plane_inlier_thres(plane_inlier_thres), _board_inlier_thres(board_inlier_thres),
        _smooth_thres(smooth_thres), _curvature_thres(curvature_thres),
        _res_vertical(res_vertical* (M_PI / 180.0f)), _res_horizental(res_horizental* (M_PI / 180.0f)),
        _searcher(searcher), _pcount_coeff(-1) { }

    inline float LidarImageCalibrator::estimatePointCount(float board_distance)
    {
        if (_pattern_size % 2 != 0)
            cerr << "The shape of the chessboard is not supported!" << endl;

        if (_pcount_coeff < 0)
        {
            float total_area = _board_size * _board_size;
            float patter_length = _pattern_size / 2;
            float hole_length = (_board_size - _edge_size) / patter_length;
            float hole_area = hole_length * hole_length * patter_length * patter_length;
            _pcount_coeff = hole_area / total_area * _board_size * _board_size 
                / (_res_vertical * _res_horizental);
        }
        return _pcount_coeff / (board_distance * board_distance);
    }

    inline void LidarImageCalibrator::findChessboardCorners(
        ConstPointCloudPtr cloud, Eigen::MatrixX3f& corners)
    {
        // Find planes in point cloud
        PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
        NormalEstimationOMP<PointType, Normal> normal_estimator;
        normal_estimator.setSearchMethod(_searcher);
        normal_estimator.setInputCloud(cloud);
        normal_estimator.setKSearch(_normal_knum);
        normal_estimator.compute(*normals);

        vector<PointIndices> clusters;
        RegionGrowing<PointType, Normal> reg;
        reg.setMinClusterSize(50);
        reg.setMaxClusterSize(1000);
        reg.setSearchMethod(_searcher);
        reg.setNumberOfNeighbours(_grow_knum);
        reg.setInputCloud(cloud);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(_smooth_thres / 180.0 * M_PI);
        reg.setCurvatureThreshold(_curvature_thres);
        reg.extract(clusters);

        // Find most possible chessboard plane in the cloud
        RandomSampleConsensus<PointType>::Ptr best_model;
        for (PointIndices cluster : clusters)
        {
            // Filter clusters with much more or much less points
            auto cluster_vec = cluster.indices;
            auto cluster_size = cluster_vec.size();
            Eigen::MatrixX3f cluster_cloud(cluster_size, 3);
            for (int idx = 0; idx < cluster_size; idx++)
            {
                cluster_cloud(idx, 0) = cloud->points[cluster_vec[idx]].x;
                cluster_cloud(idx, 1) = cloud->points[cluster_vec[idx]].y;
                cluster_cloud(idx, 2) = cloud->points[cluster_vec[idx]].z;
            }
            float distance = cluster_cloud.rowwise().squaredNorm().mean();
            float estimated_count = estimatePointCount(distance);
            if (cluster_size > estimated_count * _estimated_count_thres)
                continue;
            if (cluster_size < estimated_count / _estimated_count_thres)
                continue;

            // Filter clusters with defficient planarity
            vector<int> plane_inliers;
            Eigen::VectorXf plane_params;
            SampleConsensusModelPlane<PointType>::Ptr plane_model(
                new SampleConsensusModelPlane<PointType>(cloud, cluster_vec)); // XXX: random?
            RandomSampleConsensus<PointType> plane_sac(plane_model);

            plane_sac.computeModel();
            plane_sac.getInliers(plane_inliers);
            plane_sac.getModelCoefficients(plane_params);
            if (plane_inliers.size() < _plane_inlier_thres * cluster_size)
                continue;

            // Filter clusters with defficient board coherency
            vector<int> board_inliers;
            SampleConsensusChessboard<PointType>::Ptr board_model(
                new SampleConsensusChessboard<PointType>(cloud, plane_inliers,
                    _pattern_size ,_board_size, _edge_size, plane_params)); // XXX: random?
            RandomSampleConsensus<PointType>::Ptr board_sac(
                new RandomSampleConsensus<PointType>(board_model));

            board_sac->computeModel();
            board_sac->getInliers(board_inliers);
            if (board_inliers.size() < _board_inlier_thres * cluster_size)
                continue;
            
            // TODO: Find the best model using metrics
            best_model = board_sac;
        }
        // TODO: Refine model
        // best_model->refineModel();

        // Generate corners
        vector<float> offsets;
        offsets.reserve(_pattern_size);
        float _hole_size = 2 * (_board_size - _edge_size) / _pattern_size;
        for (float i = 0.5; i < _pattern_size / 2; i += 1)
        {
            offsets.push_back(i * _hole_size);
            offsets.push_back(-i * _hole_size);
        }

        Eigen::VectorXf model_params;
        Eigen::Vector3f plane_normal, origin, xdirection, ydirection;
        best_model->getModelCoefficients(model_params);
        decompressChessboardModel(model_params, plane_normal, origin, xdirection);
        ydirection = plane_normal.cross(xdirection);

        int counter = 0;
        for (float offset_x : offsets)
            for (float offset_y : offsets)
                corners.row(counter++) = offset_x * xdirection + offset_y * ydirection;
    }

    inline void LidarImageCalibrator::sortCorners(
        const Eigen::MatrixX2f& corners, vector<int>& indices)
    {
        indices.resize(corners.rows());
        Eigen::MatrixX2f::Index leftist_idx = 0;
        Eigen::MatrixX2f::Index sorted_idx = 0;
        unsigned int edge_counter = 0;

        unordered_set<Eigen::MatrixX2f::Index> left_filter;
        for (int i = 0; i < corners.rows(); i++) left_filter.emplace(i);

        while (!left_filter.empty())
        {
            auto bottom_bound = corners(indices[sorted_idx], 1);
            for (auto row : left_filter)
            {
                if (edge_counter > 0 && corners(row, 1) <= bottom_bound)
                    continue;
                if (corners(row, 0) < corners(leftist_idx, 0))
                    leftist_idx = row;
            }

            indices[sorted_idx++] = leftist_idx;
            left_filter.erase(leftist_idx);
            if (++edge_counter == _pattern_size) edge_counter = 0;
        }
    }

    Eigen::Matrix4f LidarImageCalibrator::calibrate(const std::vector<ConstImagePtr> images,
        const std::vector<ConstPointCloudPtr> clouds, bool visualize)
    {
        if (images.size() != clouds.size())
            cerr << "The counts of images and point clouds don't match" << endl;
        int frame_count = images.size();
        int corners_per_frame = _pattern_size * _pattern_size;

        Eigen::MatrixX2f all_image_corners(frame_count * corners_per_frame, 2);
        Eigen::MatrixX3f all_cloud_corners(frame_count * corners_per_frame, 3);
        for (int frame = 0; frame < frame_count; frame++)
        {
            // Find chessboard in image
            vector<Point> image_corners;
            Size pattern(_pattern_size, _pattern_size);
            if (!cv::findChessboardCorners(*(images[frame]), pattern, image_corners))
                cerr << "Failed to find the chessboard" << endl;

            // Sort corners in image
            Eigen::MatrixX2f image_corners_matrix(image_corners.size(), 2);
            vector<int> image_corners_indices;
            for (int i = 0; i < image_corners.size(); i++)
            {
                image_corners_matrix(i, 0) = ceil(image_corners[i].x);
                image_corners_matrix(i, 1) = ceil(image_corners[i].y);
            }
            sortCorners(image_corners_matrix, image_corners_indices);

            // Add image corners
            for (int i = 0; i < corners_per_frame; i++)
            {
                all_image_corners(frame*corners_per_frame + i, 0) = image_corners[i].x;
                all_image_corners(frame*corners_per_frame + i, 1) = image_corners[i].y;
            }

            // Find Chessboard in point cloud
            Eigen::MatrixX3f cloud_corners(corners_per_frame, 3);
            findChessboardCorners(clouds[frame], cloud_corners);

            // Sort corners in point cloud
            vector<int> cloud_corners_indices;
            // XXX: consider yoz as camera plane, set this as an argument?
            sortCorners(cloud_corners.rightCols(2), cloud_corners_indices);

            // Add cloud corners
            for (int i = 0; i < corners_per_frame; i++)
                all_cloud_corners.row(frame*corners_per_frame + i) =
                    cloud_corners.row(cloud_corners_indices[i]);
        }


        Eigen::Matrix4f m;
        return m;
    }
}
