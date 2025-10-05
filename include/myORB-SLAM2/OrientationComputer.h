#ifndef ORIENTATIONCOMPUTER_H
#define ORIENTATIONCOMPUTER_H

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

using namespace cv;
using namespace std;

namespace my_ORB_SLAM2 {

class OrientationComputer {
    public:
        /*
        @brief Before calculating orientation, it’s important to set a radius.
        
        @param[in] iRadius: Radius of the circular area. */
        OrientationComputer(int iRadius);
        ~OrientationComputer() {};
        
        // Radius of the circular area.
        int miRadius;

        // Half the number of points per unit along the y-axis of the circular area.
        int nHalfPointsPerY[16] = {15,15,15,15,14,14,14,13,13,12,11,10,9,8,6,3};

        /*
        @brief Compute a keypoint's orientation.

        @param[in] keyPoint: The target keypoint.
        @param[in] image: The image that contains the keypoint. */
        inline float getOrientation(KeyPoint &keyPoint, const Mat &image);
        
        /*
        @brief Compute orientation for each keypoint from an image pyramid.

        @param[in] vvKeyPointsPerLevel: Keypoints detected at each pyramid level.
        @param[in] vImagePerLevel: Images at each pyramid level. */
        void compute(
            vector<vector<KeyPoint>> &vvKeyPointsPerLevel,
            const vector<Mat> &vImagePerLevel
        );
};

}

#endif