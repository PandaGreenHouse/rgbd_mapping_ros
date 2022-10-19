#ifndef CQUAD_H
#define CQUAD_H
#include <stdlib.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include "util.h"
class CQuad
{
public:
    CQuad();
    CQuad(float z_bottom, float height);
    void setCoefficients(float c1, float c2, float c3, float c4);
    void construct(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, Eigen::Vector3f &vNormal);
    bool merge(CQuad &quad, float dot_thresh_between_quads, float dist_thresh_between_quads, Eigen::Vector3f &vOffSet);
    float getWidth();
    float getHeight();
    void getVertices(std::vector<Eigen::Vector3f> &vertices);
    void getLineVertices(std::vector<Eigen::Vector3f> &vertices);
    void getCenter(Eigen::Vector3f &vCenter);
    void getNormal(Eigen::Vector3f &vNormal);
    void update(float bottom, float height);
    bool correct(std::vector<Eigen::Vector3f> &vertices, Eigen::Vector3f vNormal, float dot_thresh);
    void correctingVertices(float z_camera, float camera_height, float wall_height);
    void set(std::vector<Eigen::Vector3f> &vertices, Eigen::Vector3f &vNormal);
    bool correct(CQuad &quad, float delta);
    float getDistanceFromPoint(Eigen::Vector3f &pt);
    bool getIntersection(Eigen::Vector3f &p1, Eigen::Vector3f &p2, Eigen::Vector3f &p3);
    bool isOverlappedQuads(CQuad &quad, float dot_thresh_between_quads, float dist_thresh_between_quads);
    bool checkAlignment(CQuad &quad, float lower, float upper);
    void setPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud){m_pCloud = pCloud;};
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud(){return m_pCloud;};
    float getDensity();
    void constructGround(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud);
    float getBottom();
protected:
    float distanceToQuad(std::vector<Eigen::Vector3f> &vertices);
    float distanceFromPointToQuad(Eigen::Vector3f &v, std::vector<Eigen::Vector3f> &vertices);
//properties
protected:
    std::vector<Eigen::Vector3f> m_Vertices;
    std::vector<int> m_Line_list;
    Eigen::Vector3f m_vNormal;
    float m_coefficients[4];
    Eigen::Vector3f vDisp;
    float _z_bottom;
    float _height;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_pCloud;
};

#endif // CQUAD_H
