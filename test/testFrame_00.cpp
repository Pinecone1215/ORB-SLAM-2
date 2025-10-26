#include "myORB-SLAM2/ImagePyramid.h"
#include "myORB-SLAM2/KeyPointExtractor.h"
#include "myORB-SLAM2/Distributor.h"
#include "myORB-SLAM2/OrientationComputer.h"
#include "myORB-SLAM2/DescriptorComputer.h"
#include "myORB-SLAM2/Frame.h"

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
    KeyPointExtractor keyPointExtractor(imagePyramid.mnLevels, 30.0f, 19, imagePyramid.mnFeatures, 20, 7);
    Distributor distributor;
    OrientationComputer orientationComputer(15);
    DescriptorComputer descriptorComputer;

    // vector to store each frame object.
    vector<Frame*> vpFrames;
    vpFrames.reserve(4071);

    // Calculate the average time taken per frame to compute ORB features.
    double avgTimeTaken = 0;

    // Iterate over each image.
    for(string imageFileName : vImageFileNames) {
        Mat image = imread(dirPath + "/" + imageFileName, 0);
        
        TimePoint t1 = chrono::steady_clock::now();

        // Extract keypoints.
        vector<vector<KeyPoint>> vvKeyPointsPerLevel;
        imagePyramid.setImage(image);
        vector<Mat> &vImagePerLevel = imagePyramid.mvImages;
        keyPointExtractor.extract(vvKeyPointsPerLevel, vImagePerLevel);

        // Distribute keypoints.
        vector<vector<KeyPoint>>* pvvDistributedKeyPointsPerLevel = new vector<vector<KeyPoint>>;
        distributor.distribute(
            *pvvDistributedKeyPointsPerLevel,
            vvKeyPointsPerLevel,
            imagePyramid.mvnFeaturesPerLevel,
            vImagePerLevel
        );

        // Compute orientations for each keypoint.
        orientationComputer.compute(*pvvDistributedKeyPointsPerLevel, imagePyramid.mvImages);

        // Compute descriptors for each keypoint.
        vector<vector<Descriptor>>* pvvDescriptorsPerLevel = new vector<vector<Descriptor>>;
        descriptorComputer.compute(*pvvDescriptorsPerLevel, *pvvDistributedKeyPointsPerLevel, imagePyramid.mvImages);

        // Create a frame object.
        vpFrames.push_back(
            new Frame(
                imagePyramid.mvfScaleFactors,
                pvvDistributedKeyPointsPerLevel,
                pvvDescriptorsPerLevel
            )
        );

        // Accumulate the duration.
        TimePoint t2 = chrono::steady_clock::now();
        chrono::duration<double> timeTaken = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        avgTimeTaken += timeTaken.count();

        // Display file name of the image.
        cout << imageFileName << endl;
    }

    cout << "The average time taken per frame to compute ORB features: " << avgTimeTaken/4071.0 << " s." << endl;

    // Iterate over each image.
    for(int i = 0; i < 4071; i++) {
        // Read image.
        Mat image = imread(dirPath + "/" + vImageFileNames[i], 0);
        Frame& frame = *vpFrames[i];

        // Paint keypoints from each pyramid level
        cvtColor(image, image, COLOR_GRAY2BGR);
        for(vector<KeyPoint>& vKeyPoints : *frame.mpvvKeyPointsPerLevel)
            cv::drawKeypoints(image, vKeyPoints, image, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DEFAULT);

        // Display image with keypoints.
        cv::imshow("Image", image);
        if(cv::waitKey(0) == 'q') { break; }
    }

    // Delete remaining data
    for(Frame* pFrame : vpFrames)
        delete pFrame;
    
    // Destroy all windows.
    cv::destroyAllWindows();
}