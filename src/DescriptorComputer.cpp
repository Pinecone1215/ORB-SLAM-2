#include "myORB-SLAM2/DescriptorComputer.h"

namespace my_ORB_SLAM2 {

    inline void DescriptorComputer::computeDescriptor(
        Descriptor& descriptor,
        const KeyPoint& keyPoint,
        const Mat& image
    ) {
        float fRadian = keyPoint.angle * mfDeg2Rad;
        float fCos = cosf(fRadian);
        float fSin = sinf(fRadian);

        int nCols = image.cols;
        const uchar* pCenter = &image.at<uchar>(cvRound(keyPoint.pt.y), cvRound(keyPoint.pt.x));

        #define getOffset(i) \
            cvRound(pPattern[i] * fSin + pPattern[i + 1] * fCos) * nCols + \
                cvRound(pPattern[i] * fCos - pPattern[i + 1] * fSin)

        int* pPattern = &mPattern[0];
        int p = 0;
        for(int i = 0; i < mnBytesPerDescriptor; ++i, pPattern += 32) {

            uchar I0, I1, byte = 0;

            I0 = pCenter[getOffset(0)]; I1 = pCenter[getOffset(2)];
            byte = I0 < I1;

            I0 = pCenter[getOffset(4)]; I1 = pCenter[getOffset(6)];
            byte |= (I0 < I1) << 1;

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

            descriptor[i] = byte;
        }

        #undef getOffset
    }

    void DescriptorComputer::compute(
        vector<vector<Descriptor>>& vvDescriptorsPerLevel,
        const vector<vector<KeyPoint>>& vvKeyPointsPerLevel,
        const vector<Mat>& vImagePerLevel
    ) {
        int nLevels = vImagePerLevel.size();
        vvDescriptorsPerLevel.resize(nLevels);

        for(int iLevel = 0; iLevel < nLevels; ++iLevel) {
            const Mat& image = vImagePerLevel[iLevel];
            const vector<KeyPoint>& vKeyPoints = vvKeyPointsPerLevel[iLevel];

            int nKeyPoints = vKeyPoints.size();

            vector<Descriptor>& vDescriptors = vvDescriptorsPerLevel[iLevel];
            vDescriptors.resize(nKeyPoints);

            Mat blurredImage = image.clone();
            GaussianBlur(image, blurredImage, Size(7, 7), 2, 2);

            for(int i = 0; i < nKeyPoints; ++i)
                computeDescriptor(vDescriptors[i], vKeyPoints[i], blurredImage);
        }
    }

} // my_ORB_SLAM2