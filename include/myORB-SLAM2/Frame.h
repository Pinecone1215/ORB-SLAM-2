#ifndef FRAME_H
#define FRAME_H

#include <opencv2/features2d.hpp>
#include <chrono>

using namespace cv;
using namespace std;

// Define the 32-byte Descriptor data type.
typedef array<unsigned char, 32> Descriptor;

// Define the time point type.
typedef chrono::steady_clock::time_point TimePoint;

namespace my_ORB_SLAM2 {

class Frame {
    public:
        /*
        @brief Constructor for a regular frame object.
        
        @param[in] vfScaleFactors: Scale factor of each level used to map coordinates back to level 0. 
        @param[in] vvKeyPointsPerLevel: Keypoints in the pyramid. 
        @param[in] vvDescriptorsPerLevel: Descriptors in the pyramid. */
        Frame(
            const vector<float>& vfScaleFactors,
            vector<vector<KeyPoint>>* pvvKeyPointsPerLevel,
            vector<vector<Descriptor>>* pvvDescriptorsPerLevel
        );

        // Delete mvvKeyPointsPerLevel and mvvDescriptorsPerLevel because of dynamic allocation.
        ~Frame() {
            delete mpvvKeyPointsPerLevel;
            delete mpvvDescriptorsPerLevel;
        };

        // ID for this frame and for the next frame to be created
        static unsigned long mNextID;
        unsigned long mID;

        // Frame pose.
        cv::Mat mTcw;

        // Frame time stamp
        TimePoint mTimePoint;

        // Scale Factors, Keypoints and descriptors of this frame
        vector<float> mvfScaleFactors;
        vector<vector<KeyPoint>>* mpvvKeyPointsPerLevel;
        vector<vector<Descriptor>>* mpvvDescriptorsPerLevel;
        
        // Divide this frame into 64 * 48 grids.
        // Each grid contains the indices of the keypoints within its area.
        vector<int> mGrid[64][48]; // (cols, rows)
};

} // my_ORB_SLAM2

#endif