#ifndef DISTRIBUTOR_H
#define DISTRIBUTOR_H

#include "myORB-SLAM2/RegionalQuadTree.h"

namespace my_ORB_SLAM2 {

class Distributor {
    public:
        /*
        @brief 影像金字塔概念的關鍵點均勻器  */
        Distributor() {};
        ~Distributor() {};

        /*
        @brief 用於均勻化，對應於影像金字塔中每一層的關鍵點
        
        @param[in, out] vvDistributedKeyPointsPerLevel 用於取得均勻化的關鍵點容器
        @param[in] vvKeyPointsPerLevel 對應於影像金字塔中每一層的關鍵點
        @param[in] vnFeaturesPerLevel 對應於影像金字塔中每一層的待提取關鍵點數
        @param[in] vImagePerLevel 影像金字塔中每一層的影像 */
        void distribute(
            vector<vector<KeyPoint>> &vvDistributedKeyPointsPerLevel,
            vector<vector<KeyPoint>> &vvKeyPointsPerLevel, 
            vector<int> &vnFeaturesPerLevel,
            vector<Mat> &vImagePerLevel
        );
};

}

#endif