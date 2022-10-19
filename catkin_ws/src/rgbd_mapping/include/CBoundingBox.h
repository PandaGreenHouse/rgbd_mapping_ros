#ifndef CBOUNDINGBOX_H
#define CBOUNDINGBOX_H
//#include <stdlib.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include "util.h"
#include "CQuad.h"
class CBoundingBox
{
public:
    CBoundingBox();
void setPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud);
void constructOBB(std::vector<Eigen::Vector3f> obb_vertices);
void construct(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, Eigen::Vector3f &vNormal, int nFrameId);
bool isOverlapped(CBoundingBox &box);
bool merge(CBoundingBox &box, Eigen::Vector3f &vNormal_Wall, int toleranceFrames);
void getVertices(std::vector<Eigen::Vector3f> &vertices);
void getWireFrame(std::vector<Eigen::Vector3f> &vertices);
bool isInBox(Eigen::Vector3f &vertex);
void getCenter(Eigen::Vector3f &vCenter);
void correctingVertices(float z_camera, float camera_height, float wall_height);
float getDepth();
float getTop(float zBottom);
float getBottom();
float getIntersection(Eigen::Vector3f &vRayPos, Eigen::Vector3f &vRaydir, Eigen::Vector3f &InterPos);
pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudPtr();
int isStand();
float getMaxDimension();
float getMinDimension();
void getBackQuad(Eigen::Vector3f &cam_position, std::vector<Eigen::Vector3f> &out_vertices, Eigen::Vector3f &vOutNormal);
bool checkIntersectWall(CQuad &quad);
int getFrameId(){return _frame_id;};
float getDensity();
float getHeight();
protected:
void computingQuadNormals();
//proverties
Eigen::Vector3f vertices[8];
Eigen::Vector3f min_point;
Eigen::Vector3f max_point;
protected:
Eigen::Vector3f vNormals[6];
int             quad_list[24];
int             line_list[48];
pcl::PointCloud<pcl::PointXYZ>::Ptr m_pCloud;
int m_edges[24];
int _frame_id;
int _count;
};

#endif // CBOUNDINGBOX_H
