#include "myORB-SLAM2/Frame.h"

namespace my_ORB_SLAM2 {

// Initialize the ID of next frame to be created.
unsigned long Frame::mNextID = 0;

/*
@brief Constructor for a regular frame object.

@param[in] vfScaleFactors: Scale factor of each level used to map coordinates back to level 0. 
@param[in] pvvKeyPointsPerLevel: Pointer to the pyramid that contains keypoints. 
@param[in] pvvDescriptorsPerLevel: Pointer to the pyramid that contains descriptors. */
Frame::Frame(
    const vector<float>& vfScaleFactors,
    vector<vector<KeyPoint>>* pvvKeyPointsPerLevel,
    vector<vector<Descriptor>>* pvvDescriptorsPerLevel
) {
    // Assign the ID of this frame
    mID = mNextID++;

    // Record the creation time of this frame.
    mTimePoint = chrono::steady_clock::now();

    // Copy scale factors, pvvKeyPointsPerLevel and pvvDescriptorsPerLevel.
    mvfScaleFactors = vfScaleFactors;
    mpvvKeyPointsPerLevel = pvvKeyPointsPerLevel;
    mpvvDescriptorsPerLevel = pvvDescriptorsPerLevel;

    // Iterator over keypoints of the pyramid.
    int iLevel = 0;
    for(vector<KeyPoint>& mvKeyPoints : *mpvvKeyPointsPerLevel) {
        // No need to process keypoints at level 0.
        if(iLevel == 0) { iLevel++; continue; }

        // Map each keypoint's coordinates to level 0 according to its level in the pyramid.
        for(KeyPoint& mKeyPoint : mvKeyPoints)
            mKeyPoint.pt *= mvfScaleFactors[mKeyPoint.octave];
    }
}

} // my_ORB_SLAM2