# 建立一個共享庫（動態連結庫），名稱為 myORB-SLAM2，並包含 ImagePyramid.cpp 檔案的編譯
add_library(
    myORB-SLAM2 SHARED
    ImagePyramid.cpp
    KeyPointExtractorPyramid.cpp
    KeyPointsRegionalQuadTree.cpp
    KeyPointDistributorPyramid.cpp
    OrientationComputer.cpp
    DescriptorComputer.cpp
)

# 將第三方庫連結到 myORB-SLAM2 共享庫上，確保編譯和連結時能找到所需的外部依賴
target_link_libraries(myORB-SLAM2 ${THIRD_PARTY_LIBS})