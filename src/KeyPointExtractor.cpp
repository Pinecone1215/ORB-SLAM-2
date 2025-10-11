#include "myORB-SLAM2/KeyPointExtractor.h"

namespace my_ORB_SLAM2 {
    /*
    @brief 設定關鍵點提取器
        
    @param[in] mnLevels 影像金字塔的層數
    @param[in] mfDefaultGridSize 預設每個小格子的尺寸
    @param[in] mnPaddingPixels 邊緣向內部填充的 pixels 數量
    @param[in] mnKeyPoints 總共要提取的關鍵點數量
    @param[in] miMaxTh 一開始提取關鍵點時使用的閾值
    @param[in] miMinTh 放寬標準後，提取關鍵點的閾值 */
    KeyPointExtractor::KeyPointExtractor(int mnLevels, float mfDefaultGridSize, int mnPaddingPixels, int mnKeyPoints, int miMaxTh, int miMinTh) {
        this->mnLevels = mnLevels;
        this->mfDefaultGridSize = mfDefaultGridSize;
        this->mnPaddingPixels = mnPaddingPixels;
        this->mnKeyPoints = mnKeyPoints;
        this->miMaxTh = miMaxTh;
        this->miMinTh = miMinTh;
    };

    /*
    @brief 提取關鍵點的 function
    
    @param[in, out] vvKeyPointsPerLevel 儲存影像金字塔中，每層影像的關鍵點
    @param[in] vImagePerLevel 影像金字塔的每一層影像 */
    void KeyPointExtractor::extract(
        vector<vector<KeyPoint>> &vvKeyPointsPerLevel, 
        const vector<Mat> &vImagePerLevel
    ) {
        // 設定 vvKeyPointsPerLevel 大小為影像金字塔層數
        vvKeyPointsPerLevel.resize(mnLevels);

        // 遍歷每一層影像 #pragma omp parallel for
        for(int iLevel = 0; iLevel < mnLevels; iLevel++) {
            // 設定可提取關鍵點的邊界
            int iMinBorderX = mnPaddingPixels - 3;
            int iMinBorderY = iMinBorderX;
            int iMaxBorderX = vImagePerLevel[iLevel].cols - mnPaddingPixels + 3;
            int iMaxBorderY = vImagePerLevel[iLevel].rows - mnPaddingPixels + 3;
            
            // keyPoints 用於儲存該層影像的關鍵點，並預留比預計提取之關鍵點數量多 10 倍的空間
            // 避免動態擴增空間造成的性能消耗
            vector<KeyPoint> vKeyPoints;
            vKeyPoints.reserve(mnKeyPoints * 10);
            
            // 計算關鍵點提取範圍的長寬
            float fWidth = float(iMaxBorderX - iMinBorderX);
            float fHeight = float(iMaxBorderY - iMinBorderY);
            
            // 計算目前的關鍵點提取範圍可以畫分成幾個行、列
            int nCols = int(fWidth / mfDefaultGridSize);
            int nRows = int(fHeight / mfDefaultGridSize);

            // 計算每個小格子的寬、高
            int iGridWidth = ceil(fWidth / nCols);
            int iGridHeight = ceil(fHeight / nRows);

            // 遍歷每個 Grid
            for(int iRow = 0; iRow < nRows; iRow++) {
                // 計算 FAST 關鍵點需要半徑為 3 的圓形範圍
                // 所以每個 Grid 的邊界需要暫時向外擴增 3 單位
                // 才可以確保沒丟失邊界的關鍵點訊息
                int iMinY = iMinBorderY + iRow * iGridHeight;
                int iMaxY = iMinY + iGridHeight + 3;

                // 如果 ymin 超過 maxBorderY - 6，代表沒有足夠的半徑 3 可以計算關鍵點，跳過
                if(iMinY > iMaxBorderY - 6) { continue; }
                
                // 限制 ymax 不要超過關鍵點的提取範圍
                if(iMaxY > iMaxBorderY) { iMaxY = iMaxBorderY; }
                
                // 前置步驟和上面一樣
                for(int iCol = 0; iCol < nCols; iCol++) {
                    int iMinX = iMinBorderX + iCol * iGridWidth;
                    int iMaxX = iMinX + iGridWidth + 3;
                    if(iMinX > iMaxBorderX - 6) { continue; }
                    if(iMaxX > iMaxBorderX) { iMaxX = iMaxBorderX; }

                    // 提取該 Grid 的關鍵點
                    vector<KeyPoint> vGridKeyPoints;
                    vGridKeyPoints.reserve(64);
                    FAST(
                        vImagePerLevel[iLevel].rowRange(iMinY, iMaxY).colRange(iMinX, iMaxX), 
                        vGridKeyPoints, 
                        miMinTh
                    );

                    // 如果沒有檢測到關鍵點，就降低閾值在提取一次
                    if(vGridKeyPoints.empty()) {
                        FAST(
                            vImagePerLevel[iLevel].rowRange(iMinY, iMaxY).colRange(iMinX, iMaxX), 
                            vGridKeyPoints, 
                            miMinTh
                        );
                    }

                    // 因為目前提取到的關鍵點座標是相對於該 Grid 的，所以需要恢復到相對於整張影像的座標
                    if(!vGridKeyPoints.empty()) {
                        for(vector<KeyPoint>::iterator vit = vGridKeyPoints.begin(); vit != vGridKeyPoints.end(); vit++) {
                            vit->pt.x += iCol * iGridWidth;
                            vit->pt.y += iRow * iGridHeight;
                            vKeyPoints.push_back(*vit);
                        }
                    }
                }
            }

            // 修飾該層最後提取到的關鍵點
            for(vector<KeyPoint>::iterator vit = vKeyPoints.begin(); vit != vKeyPoints.end(); vit++) {
                vit->pt.x += iMinBorderX; // 增加 Offset
                vit->pt.y += iMinBorderY; // 增加 Offset
                vit->octave = iLevel; // 設定關鍵點所屬的金字塔層級
            }
            
            // 儲存該層的 KeyPoints
            vvKeyPointsPerLevel[iLevel] = vKeyPoints;
        }
    }
}