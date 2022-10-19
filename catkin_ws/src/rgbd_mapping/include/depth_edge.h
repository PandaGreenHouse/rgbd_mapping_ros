#ifndef DEPTH_EDGE_H
#define DEPTH_EDGE_H
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>

void jump_edge(cv::Mat &depthMat, cv::Mat &edgeMap, int n, float jumpThresh);
void corner_edge(cv::Mat &depthMat, cv::Mat &edgeMap, int n, float slopeThresh1,float slopeThresh2);
void curvedEdge(cv::Mat &depthMat, cv::Mat &edgeMat, int nPixels, float straThresh);
void extremumEdge(cv::Mat &depthMat, cv::Mat &edgeMat, int nPixels, float fExtremum);
#endif // DEPTH_EDGE_H
