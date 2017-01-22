#include "Libfreenect2OpenCV.h"

#include <opencv2/opencv.hpp>

using namespace cv;

int main()
{
    libfreenect2opencv::Libfreenect2OpenCV libfreenect2OpenCV;
    int key = 0;

    // starts the class thread gathering pipline info from
    // the kinect2

    libfreenect2OpenCV.start();

    while (key <= 0) {

        // grab the current RGB matrix
        cv::Mat img = libfreenect2OpenCV.getRGBMat();
        cv::Mat edges = img;

        // opencv to gray scale
        cv::cvtColor(edges, edges, CV_BGR2GRAY);
        cv::Canny(edges, edges, 120, 175);

        // Show canny edges
        cv::imshow("Canny Edges", edges);

        // Show original RGB Image
        cv::imshow("RGB Matrix", img);

        // Show depth matrix
        cv::imshow("Depth Matrix", libfreenect2OpenCV.getDepthMat());

        // Show depth matrix
        cv::imshow("IR Matrix", libfreenect2OpenCV.getIRMat());

        key = cv::waitKey(1);
    }

    libfreenect2OpenCV.stop();
}
