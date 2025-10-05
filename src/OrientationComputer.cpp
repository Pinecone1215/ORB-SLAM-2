#include "myORB-SLAM2/OrientationComputer.h"

namespace my_ORB_SLAM2 {

/*
@brief Before calculating orientation, it’s important to set a radius.

@param[in] iRadius: Radius of the circular area. */
OrientationComputer::OrientationComputer(int iRadius): miRadius(iRadius) {}

/*
@brief Compute a keypoint's orientation.

@param[in] keyPoint: The target keypoint.
@param[in] image: The image that contains the keypoint. */
inline float OrientationComputer::getOrientation(KeyPoint &keyPoint, const Mat &image) {
    // Get the pointer to the center of the circular area.
    const uchar* pCenter = &image.at<uchar>(keyPoint.pt.y, keyPoint.pt.x);

    // Width of the image.
    int nCols = image.cols;

    // Sum of weighted coordinates.
    int weightedSumY = 0;
    int weightedSumX = 0;

    // Accumulate weighted coordinates along the center row.
    for(int iX = -nHalfPointsPerY[0]; iX <= nHalfPointsPerY[0]; ++iX) {
        weightedSumX += pCenter[iX] * iX;
    }

    // Process the upper and lower halves of the circular area.
    for(int iY = 1; iY <= miRadius; ++iY) {
        // Accumulate the weights temporarily along row iY and -iY.
        int tempSumY = 0;

        // Byte offset from center row to row iY.
        int nMove = nCols * iY;

        // Accumulate weighted coordinates symmetrically along rows iY and -iY.
        for(int iX = -nHalfPointsPerY[iY]; iX <= nHalfPointsPerY[iY]; ++iX) {
            // Intensity values at symmetric pixels.
            int weightDown = pCenter[iX - nMove];
            int weightUp = pCenter[iX + nMove];

            // The code below is equivalent to accumulating "weightUp * [iY, iX] + weightDown * [-iY, iX]".
            // Here, (weightUp - weightDown) is not multiplied by iY in each loop iteration.
            tempSumY += weightUp - weightDown; // Origin code: tempSumY += iY * weightUp - iY * weightDown;
            weightedSumX += iX * (weightUp + weightDown);
        }
        // Multiply only once to accelerate computation.
        weightedSumY += tempSumY * iY;
    }
    
    // Return the orientation(0 ~ 360 degrees).
    return fastAtan2(weightedSumY, weightedSumX);
}

/*
@brief Compute orientation for each keypoint from an image pyramid.

@param[in] vvKeyPointsPerLevel: Keypoints detected at each pyramid level.
@param[in] vImagePerLevel: Images at each pyramid level. */
void OrientationComputer::compute(
    vector<vector<KeyPoint>> &vvKeyPointsPerLevel, 
    const vector<Mat> &vImagePerLevel
) {
    // Iterate over each level of the pyramid.
    for(int iLevel = 0; iLevel < vImagePerLevel.size(); ++iLevel) {
        // The image and keypoints at this level.
        const Mat &image = vImagePerLevel[iLevel];
        vector<KeyPoint> &vKeyPoints = vvKeyPointsPerLevel[iLevel];

        // The number of keypoints.
        int nKeyPoints = vKeyPoints.size();

        // Compute the orientation for each keypoint.
        for(int i = 0; i < nKeyPoints; ++i) 
            vKeyPoints[i].angle = getOrientation(vKeyPoints[i], image);
    }
}

} // my_ORB_SLAM2
