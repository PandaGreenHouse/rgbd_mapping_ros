#ifndef MAIN_PROCESS_H
#define MAIN_PROCESS_H
#include <stdlib.h>
#include <ros/ros.h>
#include "util.h"
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <pcl/features/integral_image_normal.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/pcl_config.h>
#include <pcl/surface/poisson.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
//#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/pca.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <stereo_msgs/DisparityImage.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <image_geometry/pinhole_camera_model.h>
#include <image_geometry/stereo_camera_model.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <nav_msgs/Odometry.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>

#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <CQuad.h>
#include <CBoundingBox.h>
#include <iostream>
#include <sstream>  // Required for stringstreams
#include <string>

void initRosApp();
void initRosPlugin();
void initialize();
//generating point clouds
void cpyImageMsg(const sensor_msgs::ImageConstPtr& msg, sensor_msgs::ImagePtr &img, cv::Mat &cv_image);
bool cloudFromDepthRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pclCloud, cv::Mat& rgbMat, cv::Mat& depthMat);
void filteringCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pclCloud, pcl::IndicesPtr & indices);
//clustering point clouds and computing quads and bounding boxes
void primary_process(cv::Mat &rgbMat, cv::Mat &depthMat, const geometry_msgs::Pose &msg);
//publishing lines and quads
void publishWallQuads();
void publishWallLines();
void publishObjectBoxVertices();
void publishBoxLines();
void publishRGBImage(cv::Mat &rgbMat);
void publishUnityCameraPose(const geometry_msgs::Pose &pose);
int timer_callback();
#endif // MAIN_PROCESS_H
