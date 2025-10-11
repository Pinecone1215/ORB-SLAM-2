#include "myORB-SLAM2/Distributor.h"

namespace my_ORB_SLAM2 {
    /*
    @brief 用於均勻化，對應於影像金字塔中每一層的關鍵點
    
    @param[in, out] vvDistributedKeyPointsPerLevel 用於取得均勻化的關鍵點容器
    @param[in] vvKeyPointsPerLevel 對應於影像金字塔中每一層的關鍵點
    @param[in] vnFeaturesPerLevel 對應於影像金字塔中每一層的待提取關鍵點數
    @param[in] vImagePerLevel 影像金字塔中每一層的影像 */
    void Distributor::distribute(
        vector<vector<KeyPoint>> &vvDistributedKeyPointsPerLevel,
        vector<vector<KeyPoint>> &vvKeyPointsPerLevel, 
        vector<int> &vnFeaturesPerLevel,
        vector<Mat> &vImagePerLevel
    ) {
        // 依照影像金字塔的層數，建立同樣層數的容器，且該容器每一層儲存一組經過均勻化的關鍵點
        // 對應於影像金字塔中的每一層影像
        vvDistributedKeyPointsPerLevel.resize(vImagePerLevel.size());

        // 遍歷影像金字塔
        for(int iLevel = 0; iLevel < vImagePerLevel.size(); iLevel++) {
            // 設定關鍵點區域四叉樹
            int iWidth, iHeight; // 取得該層影像的長寬
            iWidth = vImagePerLevel[iLevel].cols;
            iHeight = vImagePerLevel[iLevel].rows;
            vector<KeyPoint> &vKeyPoints = vvKeyPointsPerLevel[iLevel];
            RegionalQuadTree quadTree(iWidth, iHeight, vKeyPoints);

            // 開始分裂四叉樹節點
            while(true) {
                // 取得四叉樹的 Nodes 迭代器
                list<RegionalQuadTreeNode>::iterator lit = 
                quadTree.mlNodes.begin();

                // 計算目前有幾個 Node 可以分裂
                int nDistribution = 0;
                while(lit != quadTree.mlNodes.end()) {
                    if(lit->mvpKeyPoints.size() > 1) { nDistribution++; }
                    lit++;
                }

                // 如果目前沒有可以分裂的 Node，就結束 Loop
                if(nDistribution == 0) { break; }
                
                // 如果目前有可以分裂的 Node
                // 就計算下一輪分裂完之後，此四叉樹大概會有幾個 Nodes
                int nNextNodes = quadTree.mlNodes.size() + nDistribution * 3;

                // 如果下一輪的 Nodes 數量小於該層總共要提取的關鍵點數量
                // 那就把除了關鍵點數量為 1 的 Node之外，剩餘的所有 Nodes 都分裂一次
                if(nNextNodes < vnFeaturesPerLevel[iLevel]) {
                    lit = quadTree.mlNodes.begin();
                    while(lit != quadTree.mlNodes.end())
                        lit = quadTree.divide(lit);
                }
                // 否則就分裂最後一次
                else {
                    // 判斷 Nodes 數是否達標
                    bool bComplete = false;

                    // 先依照關鍵點數量由大到小排序
                    quadTree.mlNodes.sort([](const RegionalQuadTreeNode &n1, const RegionalQuadTreeNode &n2) { return n1.mvpKeyPoints.size() > n2.mvpKeyPoints.size();});
                    
                    // 從最多關鍵點的 Node 開始分裂
                    lit = quadTree.mlNodes.begin();
                    while(lit != quadTree.mlNodes.end()) {
                        lit = quadTree.divide(lit);

                        // 如果 Noeds 數量達標，就停止分裂
                        if(quadTree.mlNodes.size() >= vnFeaturesPerLevel[iLevel]) {
                            bComplete = true;
                            break;
                        }
                    } 
                    
                    // 結束整個分裂流程
                    if(bComplete) { break; }
                }
            }

            // 預先分配該層關鍵點的記憶體空間
            vvDistributedKeyPointsPerLevel[iLevel].reserve(quadTree.mlNodes.size());
            for(RegionalQuadTreeNode &node : quadTree.mlNodes) {
                // 取得該 Node 中品質最耗好的關鍵點
                float fMaxResponse = -INFINITY;
                KeyPoint* pKeyPoint = nullptr;
                vector<KeyPoint*> &keyPoints = node.mvpKeyPoints;
                for(vector<KeyPoint*>::iterator vit = keyPoints.begin(); vit != keyPoints.end(); ++vit) {
                    if((*vit)->response > fMaxResponse) {
                        fMaxResponse = (*vit)->response;
                        pKeyPoint = (*vit);
                    }
                }

                // 根據對應的金字塔層級儲存該關鍵點
                vvDistributedKeyPointsPerLevel[iLevel].push_back(*pKeyPoint);
            }
        }
    };

}