#include <myORB-SLAM2/RegionalQuadTree.h>

namespace my_ORB_SLAM2 {
    /*
    @brief 設定關鍵點區域四叉樹 
    
    @param[in] iWidth 影像寬度 
    @param[in] iHeight 影像高度 
    @param[in] vKeyPoints 關鍵點 */
    RegionalQuadTree::RegionalQuadTree(int iWidth, int iHeight, vector<KeyPoint> &vKeyPoints):
    mvKeyPoints(vKeyPoints) {
        // 取得總體的關鍵點數量
        int nKeyPoints = mvKeyPoints.size();
        
        // 如果至少有一個關鍵點
        if(nKeyPoints >= 1) {
            // 設定該四叉樹的第一個 Node
            mlNodes.resize(1);
            RegionalQuadTreeNode &node = mlNodes.front();

            // 設定該 Node 的 Box 座標
            node.miMinX = 0;
            node.miMinY = 0;
            node.miMaxX = iWidth;
            node.miMaxY = iHeight;
            node.miMidX = int(0.5 * (float)iWidth);
            node.miMidY = int(0.5 * (float)iHeight);

            // 設定該 Node 的關鍵點容量
            node.mvpKeyPoints.reserve(nKeyPoints);

            // 把所有關鍵點指標儲存到 Node
            for (KeyPoint &keyPoint : mvKeyPoints) {
                node.mvpKeyPoints.push_back(&keyPoint);
            }

            // 檢查關鍵點數是否等於 1，等於 1 的話就封鎖
            node.mbLocked = (nKeyPoints == 1);
        }
    }

    /*
    @brief 根據指定的迭代器，分裂對應的節點 
        
    @param[in] lNodesIterator 指定要分裂的節點的迭代器 */
    list<RegionalQuadTreeNode>::iterator 
    RegionalQuadTree::divide(list<RegionalQuadTreeNode>::iterator &lNodesIterator) {
        // 如果該 Node 只有一個關鍵點，就直接去下一個 Node
        if(lNodesIterator->mbLocked) { return ++lNodesIterator; }
        
        // 根據父節點座標分割出四個子傑點，並計算它們的 Box 座標
        RegionalQuadTreeNode node00, node01, node10, node11;
        node00.miMinX = lNodesIterator->miMinX; 
        node00.miMinY = lNodesIterator->miMinY; 
        node00.miMaxX = lNodesIterator->miMidX; 
        node00.miMaxY = lNodesIterator->miMidY; 
        node00.miMidX = int((float)(node00.miMaxX + node00.miMinX) * 0.5);
        node00.miMidY = int((float)(node00.miMaxY + node00.miMinY) * 0.5);

        node01.miMinX = lNodesIterator->miMidX; 
        node01.miMinY = lNodesIterator->miMinY; 
        node01.miMaxX = lNodesIterator->miMaxX; 
        node01.miMaxY = lNodesIterator->miMidY; 
        node01.miMidX = int((float)(node01.miMaxX + node01.miMinX) * 0.5);
        node01.miMidY = int((float)(node01.miMaxY + node01.miMinY) * 0.5);

        node10.miMinX = lNodesIterator->miMinX; 
        node10.miMinY = lNodesIterator->miMidY; 
        node10.miMaxX = lNodesIterator->miMidX; 
        node10.miMaxY = lNodesIterator->miMaxY; 
        node10.miMidX = int((float)(node10.miMaxX + node10.miMinX) * 0.5);
        node10.miMidY = int((float)(node10.miMaxY + node10.miMinY) * 0.5);

        node11.miMinX = lNodesIterator->miMidX; 
        node11.miMinY = lNodesIterator->miMidY; 
        node11.miMaxX = lNodesIterator->miMaxX; 
        node11.miMaxY = lNodesIterator->miMaxY; 
        node11.miMidX = int((float)(node11.miMaxX + node11.miMinX) * 0.5);
        node11.miMidY = int((float)(node11.miMaxY + node11.miMinY) * 0.5);

        // 預先分配 4 個子 Node 的關鍵點容量，避免後續動態擴增造成的性能消耗
        int nKeyPoints = lNodesIterator->mvpKeyPoints.size();
        node00.mvpKeyPoints.reserve(nKeyPoints);
        node01.mvpKeyPoints.reserve(nKeyPoints);
        node10.mvpKeyPoints.reserve(nKeyPoints);
        node11.mvpKeyPoints.reserve(nKeyPoints);
        
        // 把父 Node 的所有關鍵點，依照父 Node 中心點座標，分配給 4 個子 Node
        int iMidX, iMidY;
        iMidX = lNodesIterator->miMidX;
        iMidY = lNodesIterator->miMidY;
        for(KeyPoint* &keyPoint : lNodesIterator->mvpKeyPoints) {
            if(keyPoint->pt.x < iMidX && keyPoint->pt.y < iMidY) {
                node00.mvpKeyPoints.push_back(keyPoint);
            }
            else if(keyPoint->pt.x >= iMidX && keyPoint->pt.y < iMidY) {
                node01.mvpKeyPoints.push_back(keyPoint);
            }
            else if(keyPoint->pt.x < iMidX && keyPoint->pt.y >= iMidY) {
                node10.mvpKeyPoints.push_back(keyPoint);
            }
            else { node11.mvpKeyPoints.push_back(keyPoint); }
        }

        // 設定，如果 Node 的關鍵點數為 1，則不可分裂 (封鎖)
        node00.mbLocked = (node00.mvpKeyPoints.size() == 1);
        node01.mbLocked = (node01.mvpKeyPoints.size() == 1);
        node10.mbLocked = (node10.mvpKeyPoints.size() == 1);
        node11.mbLocked = (node11.mvpKeyPoints.size() == 1);
        
        // 如果 Node 的關鍵點數大於 0，就儲存到 Node List 中
        if(node00.mvpKeyPoints.size() > 0)
            mlNodes.push_front(node00);
        
        if(node01.mvpKeyPoints.size() > 0)
            mlNodes.push_front(node01);

        if(node10.mvpKeyPoints.size() > 0)
            mlNodes.push_front(node10);

        if(node11.mvpKeyPoints.size() > 0)
            mlNodes.push_front(node11);

        // 返回下一個迭代器
        return mlNodes.erase(lNodesIterator);
    }
}