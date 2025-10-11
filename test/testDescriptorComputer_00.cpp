#include "myORB-SLAM2/ImagePyramid.h"
#include "myORB-SLAM2/KeyPointExtractor.h"
#include "myORB-SLAM2/Distributor.h"
#include "myORB-SLAM2/OrientationComputer.h"
#include "myORB-SLAM2/DescriptorComputer.h"

#include <opencv2/opencv.hpp>
#include <chrono>

#include <dirent.h>

using namespace my_ORB_SLAM2;
using namespace std;
using namespace cv;

int main(int argc, char **argv) {
    // Directory path for the image sequence.
    string dirPath = argv[1];

    // Save the file name of each image.
    std::vector<std::string> vImageFileNames;
    vImageFileNames.reserve(4071);
    for (int i = 0; i < 4071; i++) {
        std::ostringstream ss;
        ss << std::setw(6) << std::setfill('0') << i << ".png";
        vImageFileNames.push_back(ss.str());
    }

    // Initialize Image Pyramid, KeyPoint Extractor, Distributor, and Orientation Computer.
    ImagePyramid imagePyramid(3, 1.2, 1200);
    vector<vector<KeyPoint>> vvKeyPointsPerLevel;
    KeyPointExtractor keyPointExtractor(imagePyramid.mnLevels, 30.0f, 19, imagePyramid.mnFeatures, 20, 7);
    Distributor distributor;
    OrientationComputer orientationComputer(15);
    DescriptorComputer descriptorComputer;
    
    // Iterate over each image.
    for(string imageFileName : vImageFileNames) {
        Mat image = imread(dirPath + "/" + imageFileName, 0);

        // Extract and distribute all the keypoints.
        imagePyramid.setImage(image);
        vector<Mat> &vImagePerLevel = imagePyramid.mvImages;
        keyPointExtractor.extract(vvKeyPointsPerLevel, vImagePerLevel);
        vector<vector<KeyPoint>> vvDistributedKeyPointsPerLevel;
        distributor.distribute(
            vvDistributedKeyPointsPerLevel,
            vvKeyPointsPerLevel,
            imagePyramid.mvnFeaturesPerLevel,
            vImagePerLevel
        );

        orientationComputer.compute(vvDistributedKeyPointsPerLevel, imagePyramid.mvImages);

        // Measure the time taken to compute descriptors.
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        vector<vector<Descriptor>> vvDescriptorsPerLevel;
        descriptorComputer.compute(vvDescriptorsPerLevel, vvDistributedKeyPointsPerLevel, imagePyramid.mvImages);
        
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = 
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        printf("\nComputing descriptors for this frame took: %f s.\n", time_used.count());

        cv::Mat outImage;
        cv::drawKeypoints(image, vvDistributedKeyPointsPerLevel[0], outImage, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DEFAULT);
        string windowName = "image";
        cv::imshow(windowName, outImage);

        if(cv::waitKey(0) == 'q') { break; }
    }
    
    cv::destroyAllWindows();
}