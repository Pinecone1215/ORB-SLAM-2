#ifndef KEYPOINTEXTRACTOR_H
#define KEYPOINTEXTRACTOR_H

#include <vector>

#include <opencv2/features2d.hpp>

using namespace cv;
using namespace std;

namespace my_ORB_SLAM2 {

class KeyPointExtractor {
    public:
        /*
        @brief 影像金字塔概念的關鍵點提取器
        
        @param[in] mnLevels 影像金字塔的層數
        @param[in] mfDefaultGridSize 預設每個小格子的尺寸
        @param[in] mnPaddingPixels 邊緣向內部填充的 pixels 數量
        @param[in] mnKeyPoints 總共要提取的關鍵點數量
        @param[in] miMaxTh 一開始提取關鍵點時使用的閾值
        @param[in] miMinTh 放寬標準後，提取關鍵點的閾值 */
        KeyPointExtractor(int mnLevels, float mfDefaultGridSize, int mnPaddingPixels, int mnKeyPoints, int miMaxTh, int miMinTh);
        ~KeyPointExtractor() {};

        /*
        @brief 提取關鍵點的 function
        
        @param[in, out] vvKeyPointsPerLevel 儲存影像金字塔中，每層影像的關鍵點
        @param[in] vImagePerLevel 影像金字塔的每一層影像 */
        void extract(
            vector<vector<KeyPoint>> &vvKeyPointsPerLevel, 
            const vector<Mat> &vImagePerLevel
        );
    
    private:
        int mnLevels, mnPaddingPixels, mnKeyPoints, miMaxTh, miMinTh;
        float mfDefaultGridSize;
};

}
#endif