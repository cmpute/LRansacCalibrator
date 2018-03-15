#ifndef OITK_LIDARWITHIMAGE_H
#define OITK_LIDARWITHIMAGE_H

#include <Eigen/Core>

namespace oitk
{
    class LidarImageCalibrator
    {
    public:
        LidarImageCalibrator();
        Eigen::Matrix4f calibrate();
    };
}

#endif // OITK_LIDARWITHIMAGE_H