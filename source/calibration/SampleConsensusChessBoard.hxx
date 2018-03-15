#ifndef OITK_CHESSBOARDSAMPLECONSENSUS_H
#define OITK_CHESSBOARDSAMPLECONSENSUS_H

#include <pcl/sample_consensus/sac_model.h>

namespace oitk
{
    template <typename PointT>
    class SampleConsensusChessboard : public pcl::SampleConsensusModel<PointT>
    {
    public:
        typedef typename SampleConsensusModel<PointT>::PointCloud PointCloud;
        typedef typename SampleConsensusModel<PointT>::PointCloudPtr PointCloudPtr;
        typedef typename SampleConsensusModel<PointT>::PointCloudConstPtr PointCloudConstPtr;
        typedef boost::shared_ptr<SampleConsensusChessboard> Ptr;

        SampleConsensusModelCircle2D(const PointCloudConstPtr &cloud, bool random = false)
            : SampleConsensusModel<PointT>(cloud, random), tmp_inliers_()
        {
            model_name_ = "SampleConsensusChessboard";
            sample_size_ = 3;
            model_size_ = 6;
        }

        SampleConsensusChessboard(const PointCloudConstPtr &cloud,
                                  const std::vector<int> &indices,
                                  bool random = false)
            : SampleConsensusModel<PointT>(cloud, indices, random)
        {
            model_name_ = "SampleConsensusChessboard";
            sample_size_ = 3;
            model_size_ = 6;
        }

        virtual bool computeModelCoefficients(const std::vector<int> &samples,
            Eigen::VectorXf &model_coefficients) override
        {
            // TODO: Implement
            return false;
        }

        virtual void getDistancesToModel(const Eigen::VectorXf &model_coefficients,
            std::vector<double> &distances) override
        {
            // TODO: Implement
        }

        virtual void selectWithinDistance(const Eigen::VectorXf &model_coefficients,
            const double threshold, std::vector<int> &inliers) override
        {
            // TODO: Implement
        }

        virtual int countWithinDistance(const Eigen::VectorXf &model_coefficients,
            const double threshold) override
        {
            // TODO: Implement
        }
    };
}


#endif // OITK_CHESSBOARDSAMPLECONSENSUS_H