#ifndef PLANARSEGMENT_H
#define PLANARSEGMENT_H
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
//#include <opencv2/highgui.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
typedef struct Line_Info{
    int nType;//0:horizon, 1:vertical, 2:leftDig, 3:rightDig
    int label;
    bool bMerged;
    cv::Point startPt;
    int nPixels;
}Line_Info;
int segment_depth(cv::Mat &depthMat, cv::Mat &edgeMat, float slopeThresh, cv::Mat &labelMat);
#endif // PLANARSEGMENT_H
