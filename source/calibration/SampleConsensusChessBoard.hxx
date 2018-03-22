#ifndef OITK_CHESSBOARDSAMPLECONSENSUS_H
#define OITK_CHESSBOARDSAMPLECONSENSUS_H

#include <cmath>
#include <Eigen/Core>
#include <pcl/sample_consensus/sac_model.h>

namespace oitk
{
    inline void compressChessboardModel(Eigen::VectorXf &model,
        const Eigen::Vector3f &plane, const Eigen::Vector3f &origin, const Eigen::Vector3f &xdirection)
    {
        model.resize(6);
        model.head(3) = origin;
        // model.data()[3] = plane.data()[0] / plane.data()[2];
        model[3] = plane[0] / plane[2];
        model[4] = plane[1] / plane[2];

        Eigen::Vector3f absolute;
        absolute << 1, 0, 0;
        Eigen::Vector3f globalxdir = plane.cross(absolute);
        Eigen::Vector3f globalydir = plane.cross(globalxdir);
        
        model[5] = atan2f(xdirection.dot(globalydir) / globalydir.norm(),
                          xdirection.dot(globalxdir) / globalxdir.norm());
    }

    inline void decompressChessboardModel(const Eigen::VectorXf &model,
        Eigen::Vector3f &plane, Eigen::Vector3f &origin, Eigen::Vector3f &xdirection)
    {
        origin = model.head(3);
        plane[0] = model[3];
        plane[1] = model[4];
        plane[2] = 1;
        plane /= plane.norm();

        Eigen::Vector3f absolute;
        absolute << 1, 0, 0;
        Eigen::Vector3f globalxdir = plane.cross(absolute);
        Eigen::Vector3f globalydir = plane.cross(globalxdir);
        xdirection = globalxdir * cos(model[5]) + globalydir * sin(model[5]);
        xdirection /= xdirection.norm();
    }

    template <typename PointT>
    class SampleConsensusChessboard : public pcl::SampleConsensusModel<PointT>
    {
    public:
        typedef typename SampleConsensusModel<PointT>::PointCloud PointCloud;
        typedef typename SampleConsensusModel<PointT>::PointCloudPtr PointCloudPtr;
        typedef typename SampleConsensusModel<PointT>::PointCloudConstPtr PointCloudConstPtr;
        typedef boost::shared_ptr<SampleConsensusChessboard> Ptr;

    private:
        int _pattern_size;
        float _board_size, _edge_size, _rand_rate, _hole_size;
        Eigen::Vector3f _plane_coefficients;

        virtual bool isSampleGood(const std::vector<int> &samples) const override
        {
            if (samples.size() < 2)
                return false;

            // Get the values at the two points
            Vector3fMapConst p0 = input_->points[samples[0]].getVector3fMap();
            Vector3fMapConst p1 = input_->points[samples[1]].getVector3fMap();
            Eigen::Vector3f segment = (p1 - p0);

            constexpr float sq2 = 1.5; // a bit larger than sqrt(2)
            if (segment.norm() > _board_size * sq2)
                return false;

            return true;
        }

        inline float rndHalf()
        {
            constexpr int mod = 1024;
            return (rnd() % mod - (mod >> 1)) / (float)mod;
        }

        inline float distanceToHoleEdge(const float &relative_coord)
        {
            // XXX: Assert relative_coord is positive
            float times = relative_coord / (_hole_size * 2)
                + ((_pattern_size / 2 + 1) % 2 == 0 ? 0.25 : -0.25);
            times = times - (int)times;
            if (times < 0.25)
                return times * _hole_size * 2;
            else if (times < 0.5)
                return (0.5 - times) * _hole_size * 2;
            else return 0;
        }

        inline float distanceToBoard(const float &board_x, const float &board_y)
        {
            // XXX: Assert board_x and board_y is positive
            float outer_edge = _board_size / 2;
            float dx = board_x - outer_edge;
            float dy = board_y - outer_edge;
            if (dx > 0)
            {
                if (dy > 0)
                    return sqrt(dx * dx + dy * dy);
                else
                    return dx;
            }
            else if (board_y > outer_edge)
                return dy;
            else
                return min(distanceToHoleEdge(board_x),
                           distanceToHoleEdge(board_y));
        }

    protected:
        using SampleConsensusModel<PointT>::model_name_;
        using SampleConsensusModel<PointT>::input_;
        using SampleConsensusModel<PointT>::indices_;
        using SampleConsensusModel<PointT>::error_sqr_dists_;

    private:
        /** \brief Functor for the optimization function */
        struct OptimizationFunctor : pcl::Functor<float>
        {
            OptimizationFunctor(int n_data_points, SampleConsensusChessboard<PointT> *model) :
                pcl::Functor<float>(n_data_points), model_(model) {}

            int operator() (const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
            {
                // TODO: Implement
            }

            SampleConsensusChessboard<PointT> *model_;
        };

    public:
        SampleConsensusChessboard(const PointCloudConstPtr &cloud,
            int pattern_size, float board_size, float edge_size,
            Eigen::VectorXf plane_coeffs, bool random = false)
            : SampleConsensusModel<PointT>(cloud, random),
            _pattern_size(pattern_size), _board_size(board_size), _edge_size(edge_size),
            _hole_size((board_size - 2 * edge_size) / (pattern_size - 1)), _rand_rate(1)
        {
            model_name_ = "SampleConsensusChessboard";
            sample_size_ = 2;
            model_size_ = 6;

            if (plane_coeffs.size() < 3)
                cerr << "Wrong plane coefficients input. Please estimate the plane before finding the board!" << endl;
            for (int i = 0; i < 3; i++)
                _plane_coefficients[i] = plane_coeffs[i];
            _plane_coefficients /= _plane_coefficients.norm();
        }

        SampleConsensusChessboard(const PointCloudConstPtr &cloud, const std::vector<int> &indices,
            int pattern_size, float board_size, float edge_size,
            Eigen::VectorXf plane_coeffs, bool random = false)
            : SampleConsensusModel<PointT>(cloud, indices, random),
            _pattern_size(pattern_size), _board_size(board_size), _edge_size(edge_size),
            _hole_size((board_size - 2 * edge_size) / (pattern_size - 1)), _rand_rate(1)
        {
            model_name_ = "SampleConsensusChessboard";
            sample_size_ = 2;
            model_size_ = 6;

            if (plane_coeffs.size() < 3)
                cerr << "Wrong plane coefficients input. Please estimate the plane before finding the board!" << endl;
            for (int i = 0; i < 3; i++)
                _plane_coefficients[i] = plane_coeffs[i];
            _plane_coefficients /= _plane_coefficients.norm();
        }

        virtual bool computeModelCoefficients(const std::vector<int> &samples,
            Eigen::VectorXf &model_coefficients) override
        {
            pcl::Array3fMapConst p0 = input_->points[samples[0]].getArray3fMap();
            pcl::Array3fMapConst p1 = input_->points[samples[1]].getArray3fMap();

            Eigen::Array3f origin = (p0 + p1) / 2;
            origin += (p0 - origin) * rndHalf() * _rand_rate;
            Eigen::Vector3f diameter = p1 - p0;
            Eigen::Vector3f adiameter = _plane_coefficients.cross(diameter);

            Eigen::Vector3f xdirection = diameter / diameter.norm() * rndHalf() * _rand_rate
                                       + adiameter / adiameter.norm() * rndHalf() * _rand_rate;
            xdirection /= xdirection.norm();

            compressChessboardModel(model_coefficients, _plane_coefficients, origin, xdirection);
            return true;
        }

        virtual void optimizeModelCoefficients(const std::vector<int> &inliers,
            const Eigen::VectorXf &model_coefficients,
            Eigen::VectorXf &optimized_coefficients) override
        {
            // TODO: Implement
            cerr << "Not Implemented yet" << endl;
        }

        virtual void getDistancesToModel(const Eigen::VectorXf &model_coefficients,
            std::vector<double> &distances) override
        {
            distances.resize(indices_->size());

            Eigen::Vector3f plane_normal, origin, xdirection, ydirection;
            decompressChessboardModel(model_coefficients, plane_normal, origin, xdirection);
            ydirection = plane_normal.cross(xdirection);

            for (int idx = 0; idx < indices_->size(); idx++)
            {
                // Project to plane
                Eigen::Vector3f direct_dist = 
                    input_->points[(*indices_)[idx]].getArray3fMap() - origin.array();
                float zdist = direct_dist.dot(plane_normal);
                direct_dist -= zdist * plane_normal;
                float board_x = abs(direct_dist.dot(xdirection));
                float board_y = abs(direct_dist.dot(ydirection));

                // Calculate distance
                float bdist = distanceToBoard(board_x, board_y);
                // distances[idx] = sqrt(bdist * bdist + zdist * zdist);
                distances[idx] = bdist; // Just consider the distance in board
            }
        }

        virtual void selectWithinDistance(const Eigen::VectorXf &model_coefficients,
            const double threshold, std::vector<int> &inliers) override
        {
            std::vector<double> distances;
            getDistancesToModel(model_coefficients, distances);

            int counter = 0;
            inliers.resize(indices_->size());
            error_sqr_dists_.resize(indices_->size());
            for (int idx = 0;idx < distances.size(); idx++)
            {
                if (distances[idx] >= threshold) continue;
                inliers[counter] = (*indices_)[idx];
                error_sqr_dists_[counter] = distances[idx];
                counter++;
            }
            inliers.resize(counter);
            error_sqr_dists_.resize(counter);
        }

        virtual int countWithinDistance(const Eigen::VectorXf &model_coefficients,
            const double threshold) override
        {
            std::vector<double> distances;
            getDistancesToModel(model_coefficients, distances);

            int counter = 0;
            for (double distance : distances)
                if (distance < threshold) counter++;
            return counter;
        }

        virtual void projectPoints(const std::vector<int> &inliers,
            const Eigen::VectorXf &model_coefficients,
            PointCloud &projected_points,
            bool copy_data_fields = true) override
        {
            // TODO: Implement
            cerr << "Not Implemented yet" << endl;
        }

        virtual bool doSamplesVerifyModel(const std::set<int> &indices,
            const Eigen::VectorXf &model_coefficients,
            const double threshold) override
        {
            // TODO: Implement
            cerr << "Not Implemented yet" << endl;
            return false;
        }

        virtual pcl::SacModel getModelType() const override
        {
            cerr << "SampleConsensusChessboard is not a sample consensus type built in pcl!" << endl;
            return pcl::SACMODEL_PLANE;
        }

        /** \brief Set the randomization range of the board location
          * \param[in] rate the randomization range (0~1)
          */
        inline void setRandomRate(const float &rate)
        {
            _rand_rate = rate;
        }

        /** \brief Get the randomization range of the board location
          * \param[out] rate the randomization range (0~1)
          */
        inline void getRandomRate(float &rate)
        {
            rate = _rand_rate;
        }
    };
}


#endif // OITK_CHESSBOARDSAMPLECONSENSUS_H