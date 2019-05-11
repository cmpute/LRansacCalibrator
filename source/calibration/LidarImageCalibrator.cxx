#include <iostream>
#include <vector>
#include <unordered_set>
#include <cmath>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/common/angles.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/region_growing.h>
#include <unsupported/Eigen/NonLinearOptimization>

#include <opencv2/highgui.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include "calibration/LidarImageCalibrator.h"
#include "calibration/SampleConsensusChessBoard.hxx"

namespace oitk
{
    using namespace std;
    using namespace cv;
    using namespace pcl;

    LidarImageCalibrator::LidarImageCalibrator(search::Search<PointType>::Ptr searcher,
        float board_size, float edge_size, int pattern_size,
        int normal_knum, int grow_knum, float estimated_count_thres,
        float plane_inlier_thres, float board_inlier_thres,
        float plane_dist_thres, float board_dist_thres,
        float smooth_thres, float curvature_thres,
        float res_vertical, float res_horizental,
        float bin_thres)
        : _searcher(searcher), _pcount_coeff(-1),
        _board_size(board_size), _edge_size(edge_size), _pattern_size(pattern_size),
        _normal_knum(normal_knum), _grow_knum(grow_knum), _estimated_count_thres(estimated_count_thres),
        _plane_inlier_thres(plane_inlier_thres), _board_inlier_thres(board_inlier_thres),
        _plane_dist_thres(plane_dist_thres), _board_dist_thres(board_dist_thres),
        _smooth_thres(smooth_thres), _curvature_thres(curvature_thres),
        _res_vertical(res_vertical* (M_PI / 180.0f)), _res_horizental(res_horizental* (M_PI / 180.0f)),
        _bin_thres(bin_thres) { }

    inline float LidarImageCalibrator::estimatePointCount(float board_distance)
    {
        if (_pattern_size % 2 != 0)
            cerr << "The shape of the chessboard is not supported!" << endl;

        if (_pcount_coeff < 0)
        {
            float total_area = _board_size * _board_size;
            int hole_count = _pattern_size / 2;
            float hole_length = (_board_size - 2 * _edge_size) / (_pattern_size - 1);
            float hole_area = hole_length * hole_length * hole_count * hole_count;
            _pcount_coeff = hole_area / total_area * _board_size * _board_size 
                / (_res_vertical * _res_horizental);
        }
        return _pcount_coeff / (board_distance * board_distance);
    }

    bool LidarImageCalibrator::findPointCloudCorners(
        ConstPointCloudPtr cloud, Eigen::MatrixX3f& corners, VisualizeType visualize)
    {
        // Find planes in point cloud
        PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
        NormalEstimationOMP<PointType, Normal> normal_estimator;
        normal_estimator.setSearchMethod(_searcher);
        normal_estimator.setInputCloud(cloud);
        normal_estimator.setKSearch(_normal_knum);
        normal_estimator.setNumberOfThreads(0);
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

        if ((visualize & VisualizeType::PointCloudRegionGrow) == VisualizeType::PointCloudRegionGrow)
        {
            visualization::PCLVisualizer::Ptr cluster_viewer(
                new visualization::PCLVisualizer("Region grow result of point cloud"));
            ExtractIndices<PointType> cluster_extract;
            cluster_extract.setInputCloud(cloud);
            int cluster_no = 0;
            for (PointIndices cluster : clusters)
            {
                PointCloudPtr board(new PointCloud<PointType>());
                IndicesPtr indices(new vector<int>); *indices = cluster.indices;
                cluster_extract.setIndices(indices);
                cluster_extract.filter(*board);
                visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud,
                    (cluster_no * 7) % 255, (cluster_no * 11) % 255, (cluster_no * 13) % 255);
                cluster_viewer->addPointCloud<PointXYZ>(board, single_color, "cluster" + to_string(cluster_no));

                PointType pcentroid;
                Eigen::Vector4f centroid;
                compute3DCentroid(*board, centroid);
                pcentroid.getArray4fMap() = centroid;
                cluster_viewer->addText3D("tcluster" + to_string(cluster_no), pcentroid, 0.2);
                cluster_no++;
            }
            cluster_viewer->addCoordinateSystem();
            cluster_viewer->initCameraParameters();
            cluster_viewer->spin();
        }

        // Find most possible chessboard plane in the cloud
        SampleConsensus<PointType>::Ptr best_model;
        #pragma omp parallel for
        for (int cluster_idx = 0; cluster_idx < clusters.size(); cluster_idx++)
        {
            // Filter clusters with much more or much less points
            auto cluster_vec = clusters[cluster_idx].indices;
            auto cluster_size = cluster_vec.size();
            Eigen::MatrixX3f cluster_cloud(cluster_size, 3);
            for (int idx = 0; idx < cluster_size; idx++)
            {
                cluster_cloud(idx, 0) = cloud->points[cluster_vec[idx]].x;
                cluster_cloud(idx, 1) = cloud->points[cluster_vec[idx]].y;
                cluster_cloud(idx, 2) = cloud->points[cluster_vec[idx]].z;
            }
            float distance = cluster_cloud.rowwise().norm().mean();
            float estimated_count = estimatePointCount(distance);
            if (cluster_size > estimated_count * _estimated_count_thres)
            {
                PCL_DEBUG ("Region %d skipped: Too many points.\n", cluster_idx);
                continue;
            }
            if (cluster_size < estimated_count / _estimated_count_thres)
            {
                PCL_DEBUG ("Region %d skipped: Too few points.\n", cluster_idx);
                continue;
            }

            // Filter clusters with defficient planarity
            vector<int> plane_inliers;
            Eigen::VectorXf plane_params;
            SampleConsensusModelPlane<PointType>::Ptr plane_model(
                new SampleConsensusModelPlane<PointType>(cloud, cluster_vec)); // XXX: pass parameter "random" ?
            RandomSampleConsensus<PointType> plane_sac(plane_model);
            plane_sac.setDistanceThreshold(_plane_dist_thres);

            if (!plane_sac.computeModel())
            {
                // TODO: add assert, debug, error macros
                PCL_ERROR ("Region %d skipped: Failed to find the board plane in cluster!\n", cluster_idx);
                continue;
            }
            plane_sac.getInliers(plane_inliers);
            plane_sac.getModelCoefficients(plane_params);
            plane_sac.setProbability(1 - 1e-8);

            if ((visualize & VisualizeType::PointCloudClusterPlane) == VisualizeType::PointCloudClusterPlane)
            {
                ExtractIndices<PointType> plane_extract;
                plane_extract.setInputCloud(cloud);
                visualization::PCLVisualizer::Ptr plane_viewer(new visualization::PCLVisualizer("RANSAC plane"));

                // Draw cluster points
                PointCloudPtr cluster(new PointCloud<PointType>());
                IndicesPtr cluster_indices(new vector<int>); *cluster_indices = cluster_vec;
                plane_extract.setIndices(cluster_indices);
                plane_extract.filter(*cluster);
                visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 255, 0);
                plane_viewer->addPointCloud<PointXYZ>(cluster, single_color, "cluster");

                // Draw plane points
                PointCloudPtr plane(new PointCloud<PointType>());
                IndicesPtr plane_indices(new vector<int>); *plane_indices = plane_inliers;
                plane_extract.setIndices(plane_indices);
                plane_extract.filter(*plane);
                plane_viewer->addPointCloud<PointXYZ>(plane, "plane");

                // Draw plane
                ModelCoefficients plane_coeff;
                plane_coeff.values.assign(plane_params.data(), plane_params.data() + plane_params.rows());
                plane_viewer->addPlane(plane_coeff);

                plane_viewer->addCoordinateSystem();
                plane_viewer->initCameraParameters();
                plane_viewer->spin();
            }

            if (plane_inliers.size() < _plane_inlier_thres * cluster_size)
            {
                PCL_DEBUG ("Region %d skipped: Not enough plane inliers.\n", cluster_idx);
                continue;
            }

            // Filter clusters with defficient board coherency
            vector<int> board_inliers;
            SampleConsensusChessboard<PointType>::Ptr board_model(
                new SampleConsensusChessboard<PointType>(cloud, plane_inliers,
                    _pattern_size ,_board_size, _edge_size, plane_params)); // XXX: pass parameter "random" ?
            RandomSampleConsensus<PointType>::Ptr board_sac(
                new RandomSampleConsensus<PointType>(board_model));
            board_sac->setDistanceThreshold(_board_dist_thres);
            board_sac->setProbability(1 - 1e-16); // Increase trying times

            if (!board_sac->computeModel())
            {
                PCL_ERROR ("Region %d skipped: Failed to find the board in cluster!\n", cluster_idx);
                continue;
            }
            board_sac->getInliers(board_inliers);

            if ((visualize & VisualizeType::PointCloudClusterBoard) == VisualizeType::PointCloudClusterBoard)
            {
                ExtractIndices<PointType> board_extract;
                board_extract.setInputCloud(cloud);
                visualization::PCLVisualizer::Ptr board_viewer(new visualization::PCLVisualizer("RANSAC board"));

                // Draw cluster points
                PointCloudPtr cluster(new PointCloud<PointType>());
                IndicesPtr cluster_indices(new vector<int>); *cluster_indices = cluster_vec;
                board_extract.setIndices(cluster_indices);
                board_extract.filter(*cluster);
                visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 255, 0);
                board_viewer->addPointCloud<PointXYZ>(cluster, single_color, "cluster");

                // Draw board points
                PointCloudPtr board(new PointCloud<PointType>());
                IndicesPtr board_indices(new vector<int>); *board_indices = board_inliers;
                board_extract.setIndices(board_indices);
                board_extract.filter(*board);
                board_viewer->addPointCloud<PointXYZ>(board, "plane");
                
                // Draw board
                Eigen::VectorXf cluster_params;
                board_sac->getModelCoefficients(cluster_params);
                Eigen::Vector3f cluster_normal, cluster_origin, cluster_xdirection, cluster_ydirection;
                decompressChessboardModel(cluster_params, cluster_normal, cluster_origin, cluster_xdirection);
                cluster_ydirection = cluster_normal.cross(cluster_xdirection);
                PointXYZ ilu, ild, iru, ird, olu, old, oru, ord;
                float inlen = _board_size / 2 - _edge_size;
                ilu.getVector3fMap() = cluster_origin + cluster_xdirection * inlen + cluster_ydirection * inlen;
                ild.getVector3fMap() = cluster_origin + cluster_xdirection * inlen - cluster_ydirection * inlen;
                iru.getVector3fMap() = cluster_origin - cluster_xdirection * inlen + cluster_ydirection * inlen;
                ird.getVector3fMap() = cluster_origin - cluster_xdirection * inlen - cluster_ydirection * inlen;
                board_viewer->addLine(ilu, ild, "ilud");
                board_viewer->addLine(iru, ird, "irud");
                board_viewer->addLine(ilu, iru, "ilru");
                board_viewer->addLine(ild, ird, "ilrd");
                float outlen = _board_size / 2;
                olu.getVector3fMap() = cluster_origin + cluster_xdirection * outlen + cluster_ydirection * outlen;
                old.getVector3fMap() = cluster_origin + cluster_xdirection * outlen - cluster_ydirection * outlen;
                oru.getVector3fMap() = cluster_origin - cluster_xdirection * outlen + cluster_ydirection * outlen;
                ord.getVector3fMap() = cluster_origin - cluster_xdirection * outlen - cluster_ydirection * outlen;
                board_viewer->addLine(olu, old, "olud");
                board_viewer->addLine(oru, ord, "orud");
                board_viewer->addLine(olu, oru, "olru");
                board_viewer->addLine(old, ord, "olrd");

                board_viewer->addCoordinateSystem();
                board_viewer->initCameraParameters();
                board_viewer->spin();
            }

            if (board_inliers.size() < _board_inlier_thres * cluster_size)
            {
                PCL_DEBUG ("Region %d skipped: Not enough board inliers.\n", cluster_idx);
                continue;
            }

            // TODO: Find the best model using metrics
            best_model = board_sac;
        }
        if (best_model == nullptr)
            return false;
        // TODO: Refine model
        // best_model->refineModel();

        // Generate corners
        vector<float> offsets;
        offsets.reserve(_pattern_size);
        float _hole_size = (_board_size - 2 * _edge_size) / (_pattern_size - 1);
        #ifdef USE_INNER_GRIDS
        for (float i = 0.5; i < _pattern_size / 2 - 1; i += 1)
        #else
        for (float i = 0.5; i < _pattern_size / 2; i += 1)
        #endif
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
                corners.row(counter++) = origin + offset_x * xdirection + offset_y * ydirection;

        return true;
    }

    bool LidarImageCalibrator::findImageCorners(
        ConstImagePtr image, Eigen::MatrixX2f& corners, VisualizeType visualize)
    {
        Mat binImage;
        vector<Point2f> image_corners;
        #ifdef USE_INNER_GRIDS
        Size pattern(_pattern_size - 2, _pattern_size - 2);
        #else
        Size pattern(_pattern_size, _pattern_size);
        #endif
        threshold(*image, binImage, _bin_thres, 255, THRESH_BINARY);
        if (!cv::findChessboardCorners(binImage, pattern, image_corners, CALIB_CB_FAST_CHECK))
        {
            cerr << "Failed to find the chessboard in image" << endl;
            return false;
        }

        for (int i = 0; i < image_corners.size(); i++)
        {
            corners(i, 0) = image_corners[i].x;
            corners(i, 1) = image_corners[i].y;
        }
        return true;
    }

    inline void LidarImageCalibrator::sortCorners(
        const Eigen::MatrixX2f& corners, vector<int>& indices)
    {
        indices.resize(corners.rows());
        Eigen::MatrixX2f::Index sorted_idx = 0;
        unsigned int edge_counter = 0;
        float bottom_bound;

        unordered_set<Eigen::MatrixX2f::Index> left_filter;
        for (int i = 0; i < corners.rows(); i++) left_filter.emplace(i);

        while (!left_filter.empty())
        {
            Eigen::MatrixX2f::Index leftist_idx = *(left_filter.begin());
            bool found_leftist = false;
            for (auto row : left_filter)
            {
                if (edge_counter > 0 && corners(row, 1) <= bottom_bound) // Mask
                    continue;
                if (!found_leftist)
                {
                    leftist_idx = row;
                    found_leftist = true;
                }
                if (corners(row, 0) < corners(leftist_idx, 0)) // Find leftist
                    leftist_idx = row;
            }

            bottom_bound = corners(leftist_idx, 1);
            indices[sorted_idx++] = leftist_idx;
            left_filter.erase(leftist_idx);
        #ifdef USE_INNER_GRIDS
            if (++edge_counter == _pattern_size - 2) edge_counter = 0;
        #else
            if (++edge_counter == _pattern_size) edge_counter = 0;
        #endif
        }
    }

    /** \brief Functor for the homography matrix initial optimization function */
    struct LidarImageCalibrator::InitialOptimizationFunctor : pcl::Functor<float>
    {
        InitialOptimizationFunctor(const Eigen::MatrixX2f& image_points, const Eigen::MatrixX3f& lidar_points)
            : pcl::Functor<float>(image_points.rows()),
            _image_points(image_points), _lidar_points(lidar_points) {}

        const Eigen::MatrixX2f& _image_points;
        const Eigen::MatrixX3f& _lidar_points;

        int operator() (const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
        {
            // Reshape
            Eigen::Matrix<float, 3, 4> HH =
                Eigen::Matrix<float, 3, 4, Eigen::RowMajor>::Map(x.data());

            // Project
            Eigen::Matrix4Xf homo_points(4, _lidar_points.rows());
            homo_points.topRows(3) = _lidar_points.transpose();
            homo_points.bottomRows(1).setOnes();

            Eigen::Matrix3Xf projected_points = HH * homo_points;
            projected_points.row(0) = projected_points.row(0).cwiseQuotient(projected_points.row(2));
            projected_points.row(1) = projected_points.row(1).cwiseQuotient(projected_points.row(2));

            // Compute errors
            Eigen::Matrix2Xf err = (_image_points.transpose() - projected_points.topRows(2));
            fvec = err.colwise().norm();

            return 0;
        }
    };

    /** \brief Functor for the homography matrix finetune optimization function 
      *        If H is provided, then the H part of input parameters is fixed.
      */
    struct LidarImageCalibrator::FinetuneOptimizationFunctor : pcl::Functor<float>
    {
        FinetuneOptimizationFunctor(const Eigen::MatrixX2f& image_points, const Eigen::MatrixX3f& lidar_points,
            int image_rows, int image_columns, const Eigen::VectorXf& proportion,
            const Eigen::Matrix<float, 3, 4>& H)
            : pcl::Functor<float>(image_points.rows()),
            _image_points(image_points), _lidar_points(lidar_points),
            _image_rows(image_rows), _image_columns(image_columns),
            _proportion(proportion), _HH(H), _fixed_H(true) {}

        FinetuneOptimizationFunctor(const Eigen::MatrixX2f& image_points, const Eigen::MatrixX3f& lidar_points,
            int image_rows, int image_columns, const Eigen::VectorXf& proportion)
            : pcl::Functor<float>(image_points.rows()),
            _image_points(image_points), _lidar_points(lidar_points),
            _image_rows(image_rows), _image_columns(image_columns),
            _proportion(proportion), _HH(), _fixed_H(false) {}

        const Eigen::MatrixX2f& _image_points;
        const Eigen::MatrixX3f& _lidar_points;
        const int _image_rows, _image_columns;
        const Eigen::VectorXf& _proportion;
        const Eigen::Matrix<float, 3, 4> _HH;
        const bool _fixed_H;

        int operator() (const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
        {
            Eigen::VectorXf P = x.cwiseProduct(_proportion);

            // Reshape and initialize
            Eigen::Matrix<float, 3, 4> HH;
            float u0, v0, k1, k2, p1, p2;
            if (_fixed_H)
            {
                HH = _HH;
                u0 = P[12]; v0 = P[13];
                k1 = P[14]; k2 = P[15];
                p1 = P[16]; p2 = P[17];
            }
            else
            {
                HH = Eigen::Matrix<float, 3, 4, Eigen::RowMajor>::Map(x.head(12).data());
                u0 = P[0]; v0 = P[1];
                k1 = P[2]; k2 = P[3];
                p1 = P[4]; p2 = P[5];
            }

            // Project
            Eigen::Matrix4Xf homo_points(4, _lidar_points.rows());
            homo_points.topRows(3) = _lidar_points.transpose();
            homo_points.bottomRows(1).setOnes();

            Eigen::Matrix3Xf projected_points = HH * homo_points;
            projected_points.row(0) = projected_points.row(0).cwiseQuotient(projected_points.row(2));
            projected_points.row(1) = projected_points.row(1).cwiseQuotient(projected_points.row(2));

            // Transform into camera coordinate
            auto ux = projected_points.row(0).array() - u0;
            auto uy = projected_points.row(1).array() - v0;
            auto r2 = ux.square() + uy.square();
            /*auto nux = ux*(1+k1*r2+k2*r2.square(), )*/

            // Compute errors
            Eigen::Matrix2Xf err = (_image_points.transpose() - projected_points.topRows(2));
            fvec = err.colwise().norm();

            return 0;
        }
    };

    Eigen::Matrix<float, 3, 4>& LidarImageCalibrator::solveHomographyMatrix(
        Eigen::MatrixX2f& image_corners, Eigen::MatrixX3f& cloud_corners)
    {
        // Construct non-homogeneous linear equation group
        int num_sample = cloud_corners.rows(); // TODO: assert image_corners.rows() == cloud_corners.rows()
        Eigen::MatrixXf A = Eigen::MatrixXf::Zero(3 * num_sample, 12 + num_sample);
        for (int i = 0, j = 0; i < num_sample; i++, j += 3)
        {
            A.block<1, 3>(j, 0) = cloud_corners.row(i);
            A.block<1, 3>(j + 1, 4) = cloud_corners.row(i);
            A.block<1, 3>(j + 2, 8) = cloud_corners.row(i);
            A(j, 3) = A(j, 7) = A(j, 11) = 1;

            A.block<2, 1>(j, 12 + i) = -image_corners.row(i);
            A(j + 2, 12 + i) = -1;
        }

        // Solve equations Ax = 0
        auto b = -A.rightCols(1);
        auto acut = A.cols() - 1;
        // https://stackoverflow.com/a/13924079
        Eigen::VectorXf Hc = A.leftCols(acut).jacobiSvd(
            Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
        Eigen::VectorXf H = Hc.head(12);

        // Optimize projection matrix
        InitialOptimizationFunctor functor(image_corners, cloud_corners);
        Eigen::NumericalDiff<InitialOptimizationFunctor> num_diff(functor);
        Eigen::LevenbergMarquardt<Eigen::NumericalDiff<InitialOptimizationFunctor>, float> lm(num_diff);
        lm.parameters.maxfev = 40000;
        Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(H); // https://stackoverflow.com/a/18536310

        // Reshape
        Eigen::Matrix<float, 3, 4> HH =
            Eigen::Matrix<float, 3, 4, Eigen::RowMajor>::Map(H.data());
        return HH;
    }

    Eigen::Matrix<float, 3, 4>& LidarImageCalibrator::calibrate(const std::vector<ConstImagePtr> images,
        const std::vector<ConstPointCloudPtr> clouds, VisualizeType visualize)
    {
        if (images.size() != clouds.size())
            cerr << "The counts of images and point clouds don't match" << endl;
        int frame_count = images.size();
        #ifdef USE_INNER_GRIDS
        int corners_per_frame = (_pattern_size - 2) * (_pattern_size - 2);
        #else
        int corners_per_frame = _pattern_size * _pattern_size;
        #endif

        Eigen::MatrixX2f all_image_corners(frame_count * corners_per_frame, 2);
        Eigen::MatrixX3f all_cloud_corners(frame_count * corners_per_frame, 3);
        #pragma omp parallel for
        for (int frame = 0; frame < frame_count; frame++)
        {
            // Find chessboard in image
            Eigen::MatrixX2f image_corners(corners_per_frame, 2);
            findImageCorners(images[frame], image_corners, visualize);

            // Sort corners in image
            vector<int> image_corners_indices;
            Eigen::MatrixX2f image_corners_vreverse = image_corners;
            image_corners_vreverse.rightCols(1).array() *= -1;
            sortCorners(image_corners_vreverse, image_corners_indices);

            // Add image corners
            for (int i = 0; i < corners_per_frame; i++)
                all_image_corners.row(frame*corners_per_frame + i) =
                    image_corners.row(image_corners_indices[i]);

            // Visualize image corners
            if ((visualize & VisualizeType::ImageCorners) == VisualizeType::ImageCorners)
            {
                Mat painted = *images[frame], resized;
                int counter = 1;
                for (auto corner_tag : image_corners_indices)
                {
                    cv::Point corner_center(image_corners(corner_tag, 0), image_corners(corner_tag, 1));
                    cv::Scalar pcolor(255, 255, 0), tcolor(0, 255, 0);
                    cv::circle(painted, corner_center, 30, pcolor, 5);
                    cv::putText(painted, to_string(counter++), corner_center,
                        cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 2, tcolor, 5);
                }

                resize(painted, resized, Size(400 * painted.cols / painted.rows, 400));
                namedWindow("Chessboard corners in image");
                imshow("Chessboard corners in image", resized);
                waitKey(0); destroyWindow("Chessboard corners in image");
            }

            // Find Chessboard in point cloud
            Eigen::MatrixX3f cloud_corners(corners_per_frame, 3);
            if (!findPointCloudCorners(clouds[frame], cloud_corners, visualize))
                cerr << "Failed to find the chessboard in point cloud" << endl;

            // Sort corners in point cloud
            vector<int> cloud_corners_indices;
            // XXX: assume yoz is the camera plane, set this as an argument?
            sortCorners(cloud_corners.rightCols(2), cloud_corners_indices);

            // Add cloud corners
            for (int i = 0; i < corners_per_frame; i++)
                all_cloud_corners.row(frame*corners_per_frame + i) =
                    cloud_corners.row(cloud_corners_indices[i]);

            // Visualize cloud corners
            if ((visualize & VisualizeType::PointCloudCorners) == VisualizeType::PointCloudCorners)
            {
                visualization::PCLVisualizer::Ptr corners_viewer(new visualization::PCLVisualizer("corners"));
                corners_viewer->addPointCloud<PointXYZ>(clouds[frame], "cluster");

                int counter = 1;
                for (auto corner_tag : cloud_corners_indices)
                {
                    PointType pcentroid;
                    pcentroid.getVector3fMap() = cloud_corners.row(corner_tag);
                    corners_viewer->addSphere(pcentroid, 0.02, 255, 255, 0, "cp" + to_string(corner_tag));
                    corners_viewer->addText3D("c" + to_string(counter++)
                        + "(" + to_string(cloud_corners(corner_tag, 1)) + "," + to_string(cloud_corners(corner_tag, 2)) + ")"
                        , pcentroid, 0.02, 0, 255, 0);
                }

                corners_viewer->addCoordinateSystem();
                corners_viewer->initCameraParameters();
                corners_viewer->spin();
            }
        }

        auto initial_H = solveHomographyMatrix(all_image_corners, all_cloud_corners);

        Eigen::Matrix<float, 6, 1> initial_P = Eigen::Matrix<float, 6, 1>::Zero();
        // XXX: Add image size as parameter or use assert to check every image ?
        initial_P[0] = images[0]->cols;
        initial_P[1] = images[0]->rows;
        Eigen::Matrix<float, 6, 1> proportion = Eigen::Matrix<float, 6, 1>::Ones();

        // TODO: optimize
        return initial_H;
    }
}
