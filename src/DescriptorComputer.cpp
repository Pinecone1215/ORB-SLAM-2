#include "myORB-SLAM2/DescriptorComputer.h"

namespace my_ORB_SLAM2 {

    void DescriptorComputer::computeDescriptor(
        const KeyPoint& keyPoint,
        const Mat& image,
        Descriptor& descriptor
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
        for(int i = 0; i < mnBytesPerDescriptor; ++i, pPattern += 32) {

            uchar I0, I1, byte = 0;

            I0 = pCenter[getOffset(i)]; I1 = pCenter[getOffset(i + 2)];
            byte = I0 < I1;

            I0 = pCenter[getOffset(i + 4)]; I1 = pCenter[getOffset(i + 6)];
            byte |= (I0 < I1) << 1;

            I0 = pCenter[getOffset(i + 8)]; I1 = pCenter[getOffset(i + 10)];
            byte |= (I0 < I1) << 2;

            I0 = pCenter[getOffset(i + 12)]; I1 = pCenter[getOffset(i + 14)];
            byte |= (I0 < I1) << 3;

            I0 = pCenter[getOffset(i + 16)]; I1 = pCenter[getOffset(i + 18)];
            byte |= (I0 < I1) << 4;

            I0 = pCenter[getOffset(i + 20)]; I1 = pCenter[getOffset(i + 22)];
            byte |= (I0 < I1) << 5;

            I0 = pCenter[getOffset(i + 24)]; I1 = pCenter[getOffset(i + 26)];
            byte |= (I0 < I1) << 6;

            I0 = pCenter[getOffset(i + 28)]; I1 = pCenter[getOffset(i + 30)];
            byte |= (I0 < I1) << 7;

            descriptor[i] = byte;
        }

        #undef getOffset
    }
}