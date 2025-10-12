#include "myORB-SLAM2/DescriptorComputer.h"

namespace my_ORB_SLAM2 {
    /*
    @brief Compute a descriptor for a keypoint based on its image.
    
    @param[in, out] descriptor: The 256-bit descriptor to be written.
    @param[in] keyPoint: The keypoint used to compute the descriptor based on its coordinates. 
    @param[in] image: The image that contains the keypoint mentioned above. */
    inline void DescriptorComputer::computeDescriptor(
        Descriptor& descriptor,
        const KeyPoint& keyPoint,
        const Mat& image
    ) { 
        // Convert keypoint's angle from degrees to radians.
        float fRadian = keyPoint.angle * mfDeg2Rad;

        // Calculate cosine and sine in advance of rotating coordinates.
        float fCos = cosf(fRadian);
        float fSin = sinf(fRadian);

        // Obtain the number of bytes of an image row, 
        // and the pointer to the pixel at the keypoint's coordinates.
        int nCols = image.cols;
        const uchar* pCenter = &image.at<uchar>(cvRound(keyPoint.pt.y), cvRound(keyPoint.pt.x));

        // Rotate the coordinates based on the keypoint's orientation, 
        // and calculate the memory offset relative to the keypoint’s pixel pointer.
        #define getOffset(i) \
            cvRound(pPattern[i] * fSin + pPattern[i + 1] * fCos) * nCols + \
                cvRound(pPattern[i] * fCos - pPattern[i + 1] * fSin)

        // The pointer to the first x coordinate in the pattern array.
        int* pPattern = &mPattern[0];
        for(int i = 0; i < mnBytesPerDescriptor; ++i, pPattern += 32) {
            // Generate one byte in the descriptor.
            uchar I0, I1, byte = 0;

            // Obtain the intensities of point 1 and point 2.
            I0 = pCenter[getOffset(0)]; I1 = pCenter[getOffset(2)];
            byte = I0 < I1; // Set one bit based on comparison of the two intensities

            // Obtain the intensities of point 3 and point 4.
            I0 = pCenter[getOffset(4)]; I1 = pCenter[getOffset(6)];
            byte |= (I0 < I1) << 1; // Set one bit based on comparison of the two intensities

            // Repeat this comparison for all remaining pairs.
            I0 = pCenter[getOffset(8)]; I1 = pCenter[getOffset(10)];
            byte |= (I0 < I1) << 2;
            I0 = pCenter[getOffset(12)]; I1 = pCenter[getOffset(14)];
            byte |= (I0 < I1) << 3;
            I0 = pCenter[getOffset(16)]; I1 = pCenter[getOffset(18)];
            byte |= (I0 < I1) << 4;
            I0 = pCenter[getOffset(20)]; I1 = pCenter[getOffset(22)];
            byte |= (I0 < I1) << 5;
            I0 = pCenter[getOffset(24)]; I1 = pCenter[getOffset(26)];
            byte |= (I0 < I1) << 6;
            I0 = pCenter[getOffset(28)]; I1 = pCenter[getOffset(30)];
            byte |= (I0 < I1) << 7;

            // Write this byte into the 32-byte descriptor.
            descriptor[i] = byte;
        }

        // Remove macro
        #undef getOffset
    }

    /*
    @brief Compute the descriptors for all the keypoints in the image pyramid.
    
    @param[in, out] vvDescriptorsPerLevel: Stores all the descriptors at each pyramid level.
    @param[in] vvKeyPointsPerLevel: Stores all the keypoints at each pyramid level.
    @param[in] vImagePerLevel: Stores all the images at each pyramid level. */
    void DescriptorComputer::compute(
        vector<vector<Descriptor>>& vvDescriptorsPerLevel,
        const vector<vector<KeyPoint>>& vvKeyPointsPerLevel,
        const vector<Mat>& vImagePerLevel
    ) {
        // Obtain the number of levels in the image pyramid.
        int nLevels = vImagePerLevel.size();

        // Resize this vector to match the number of levels.
        vvDescriptorsPerLevel.resize(nLevels);

        // Iterate over the levels in the image pyramid.
        for(int iLevel = 0; iLevel < nLevels; ++iLevel) {
            // Image and keypoints at this level.
            const Mat& image = vImagePerLevel[iLevel];
            const vector<KeyPoint>& vKeyPoints = vvKeyPointsPerLevel[iLevel];

            // The number of keypoints at this level.
            int nKeyPoints = vKeyPoints.size();

            // This vector stores all the descriptors to be computed at this level.
            vector<Descriptor>& vDescriptors = vvDescriptorsPerLevel[iLevel];
            vDescriptors.resize(nKeyPoints);

            // Blurring the image helps reduce noise.
            Mat blurredImage = image.clone();
            GaussianBlur(image, blurredImage, Size(7, 7), 2, 2);

            // Compute descriptor for each keypoint at this level.
            for(int i = 0; i < nKeyPoints; ++i)
                computeDescriptor(vDescriptors[i], vKeyPoints[i], blurredImage);
        }
    }

} // my_ORB_SLAM2