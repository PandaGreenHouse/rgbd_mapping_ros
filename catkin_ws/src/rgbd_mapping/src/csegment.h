#ifndef CSEGMENT_H
#define CSEGMENT_H
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
#include <CQuad.h>
#include <CBoundingBox.h>
class CSegment
{
public:
    CSegment();
    void setClusteringParams(float objClustTolerance, float cluster_tolerance, int cluster_max_size,
                             int cluster_min_size, float toleranceForFilter);
    void setPlaneSegmentParams(int maxIteration, float distThresh, float rate_rest_thresh);
    void setWallSegmentParams(float wallWidth, float wallHeight, float tolerance);
    void setGroundSegmentParams(float groundWidth, float groundHeight, float tolerance);
    void setParamsForWallMerge(float dotToleranceForMerge, float distToleranceForMerge, float distToleranceForConnect);
    void setGroundAndWallHeight(float z_bottom, float wallHeight);
    void setObjParams(float min_thresh, float max_thresh, float top_thresh, int toleranceFrames);
    void setWalls(std::vector<CQuad> &walls);
    void setBoxes(std::vector<CBoundingBox> &boxes);
    void setPlaneClusterParams(float planeWidth, float planeHeight, float dotTolerance);
    void getWallsAndBoxes(std::vector<CQuad> &walls, std::vector<CBoundingBox> &boxes);
    void removingCeilingAndGround(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pCloud1,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pCloud2);
    int main_process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pCloud,
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudObject);
    void getOffset(Eigen::Vector3f &vOffset1, Eigen::Vector3f &vOffset2);
    bool checkIntersectWallAndBox(CBoundingBox &box, std::vector<CQuad> &walls);
protected:
    void getMaxCluster(float tolerance,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out);
    void clustering(float tolerance,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pCloud,
                    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters);
    void extractPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
                        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &planarClouds,
                        std::vector<Eigen::Vector4f> &vCoeffs,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &nonPlanarCloud);
    int extractWalls(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &planarClouds,
                         std::vector<Eigen::Vector4f> &planes,
                         std::vector<CQuad> &walls,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudWalls,
                         std::vector<CQuad> &candidateWalls,
                         std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &candidateWallClouds,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr &objClouds,
                         std::vector<Eigen::Vector4f> &horizonPlanes,
                         std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &horizonPlanarClouds);
    //bool checkError(CQuad &quad);
    int checkPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud,
                             float c0, float c1, float c2, float c3);
    void refineSegment(std::vector<CQuad> &walls1,
                       std::vector<CQuad> &walls2,
                       std::vector<CQuad> &candidateWalls,
                       std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &candidateWallClouds,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudObj);
    void mergeWalls(std::vector<CQuad> &walls);
    void updateGround(std::vector<Eigen::Vector3f> &vertices);
    void createBoxes(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusterObjs,
                               std::vector<CBoundingBox> &boxes);
    bool mergeBox(CBoundingBox &box);
    bool isWall(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud, float c0, float c1, float c2, float c3);
    bool isGround(std::vector<Eigen::Vector3f> &vertices);
    void getWallNormals(Eigen::Vector3f &vDir);
    bool isObjectBox(CBoundingBox &box);
    void setWallNormals(Eigen::Vector3f &vDir);
    void correctingQuads();
    void extractObjects(std::vector<Eigen::Vector4f> &horizenPlanes,
                       std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &planeClouds,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr objCloud);
    void updateBottom(std::vector<CQuad> &quads,std::vector<CBoundingBox> &boxes);
//attributes
protected:
    //for segmenting walls
    float _widthForWallSegment;
    float _heightForWallSegment;
    float _toleranceForWallDotUp;
    float _toleranceForWallFilter;
    //for segmenting ground
    float _widthForGroundSegment;
    float _heightForGroundSegment;
    float _toleranceForGroundDotUp;
    float _toleranceForGroundRefineSegment;
    //for Eucleadian segmenting
    float _toleranceForClustering;
    int   _cluster_max_size;
    int   _cluster_min_size;
    float _objTolerance;
    //for plane segmenting
    int   _maxIterationForPlaneSegment;//for plane segment
    float _distThreshForPlaneSegment;//for plane segment
    float _rateForPlaneSegment;//for plane segment
    //for merging walls
    float _toleranceForDotBetweenWalls;
    float _toleranceForDistBetweenWalls;
    float _connect_dist;
    std::vector<CQuad> _walls;
    std::vector<CQuad> _grounds;
    std::vector<CBoundingBox> _boxes;
    Eigen::Vector3f _wall_normals[2];
    int _toleranceFrames;
    //
    float _wallHeight;
    float _z_bottom;
    float _object_min_thresh;
    float _object_max_thresh;
    float _object_top_thresh;
    int _tolerSizeForCluster;
    float _upper_dist;
    float _lower_dist;
    Eigen::Vector3f _offset;
    Eigen::Vector3f _vOffset1;
    Eigen::Vector3f _vOffset2;
    float _densityForWall;
    float _densityForObj;
    Eigen::Vector4f _groundPlane;
    int _frameCount;
};

#endif // CSEGMENT_H
