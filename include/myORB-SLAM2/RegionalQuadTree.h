#ifndef REGIONALQUADTREE_H
#define REGIONALQUADTREE_H

#include <list>
#include <vector>
#include <opencv2/features2d.hpp>

using namespace cv;
using namespace std;

namespace my_ORB_SLAM2 {

// 關鍵點區域四叉樹節點類
class RegionalQuadTreeNode {
    public:
        /*
        @brief 設定關鍵點區域四叉樹節點 */
        RegionalQuadTreeNode() {};
        ~RegionalQuadTreeNode() {};
        
        vector<KeyPoint*> mvpKeyPoints; // 該節點內包含的關鍵點指標
        int miMinX, miMinY, miMaxX, miMaxY, miMidX, miMidY; // 該節點在影像中的框框座標
        bool mbLocked = false; // 用於判斷該節點是否能夠進行任何更動
};

class RegionalQuadTree {
    public:
        /*
        @brief 設定關鍵點區域四叉樹 
        
        @param[in] iWidth 影像寬度 
        @param[in] iHeight 影像高度 
        @param[in] vKeyPoints 一組關鍵點 */
        RegionalQuadTree(int iWidth, int iHeight, vector<KeyPoint> &vKeyPoints);
        ~RegionalQuadTree() {};

        list<RegionalQuadTreeNode> mlNodes; // 該四叉樹的節點串列
        vector<KeyPoint> &mvKeyPoints; // 四叉樹的所有 Key Points

        /*
        @brief 根據指定的迭代器，分裂對應的節點
            
        @param[in] lNodesIterator 指定要分裂的節點的迭代器 */
        list<RegionalQuadTreeNode>::iterator divide(list<RegionalQuadTreeNode>::iterator &lNodesIterator);
};

}

#endif