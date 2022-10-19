#include "main_process.h"
#include "csegment.h"
#include "depth_edge.h"
#include "planarSegment.h"
pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pCloud;
pcl::PointCloud<pcl::PointXYZRGB> _total_cloud;
pcl::PointCloud<pcl::PointXYZRGB> _cloud_wall;
pcl::PointCloud<pcl::PointXYZRGB> _cloud_floor;
pcl::PointCloud<pcl::PointXYZRGB> _cloud_inner;

double fx, fy, cx, cy;
double maxDepth_ = 3.0;
double minDepth_ = 1.0;
double voxelSize_ = 0.03;
int    decimation_ = 4;
double noiseFilterRadius_ = 0.1;
int    noiseFilterMinNeighbors_ = 1;
int    normalK_ = 0;
double normalRadius_ = 0.0;

float _leaf_size = 0.01;
float _distThreshold = 0.01;
int _maxIteration = 1000;

float _rate_rest_thresh = 0.3f;
float _dot_up_wall_thresh = 0.1f;
float _dot_quad_merged_thresh = 0.8f;
float _dist_quad_thresh = 0.1f;
float _quad_dot_thresh = 0.9f;
float _quad_width_thresh = 1.0;
float _quad_height_thresh = 1.0;

float _wall_height = 3;
float _z_bottom = 0.0f;
//check object
float _object_top = 2;
float _object_depth = 0.4;
float _max_object_size = 1.5;
float _min_object_size = 0.1f;
//EuclideanClustering
float _object_cluster_tolerance = 0.025;
int   _min_object_cluster_size = 50;
int   _max_object_cluster_size = 15000;
float _cluster_tolerance = 0.02;//2cm
int   _min_cluster_size = 100;
int   _max_cluster_size = 25000;

ros::Publisher _cameraPosePub;
ros::Publisher _unityCameraPosePub;
ros::Publisher _cloudPub;
ros::Publisher _cloudPubForOctoMap;
ros::Publisher _wall_lines_pub;
ros::Publisher _wall_vertices_pub;
ros::Publisher _box_lines_pub;
ros::Publisher _box_vertices_pub;
ros::Publisher _tf_pub;

image_transport::Publisher _rgbPub;
image_transport::Publisher _rawDepthImgPub;
image_transport::Publisher _smoothDepthImgPub;
image_transport::Publisher _edgeImgPub;
image_transport::Publisher _labelImgPub;

message_filters::Subscriber<nav_msgs::Odometry> *_subscriberOdometry;
message_filters::Subscriber<sensor_msgs::Image> *_subscriberRGB;
message_filters::Subscriber<sensor_msgs::Image> *_subscriberDepth;
//message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image,nav_msgs::Odometry> > *_sync_depth_rgb;
message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> > *_sync_depth_rgb;

std::vector<CQuad> _my_quads;
std::vector<CBoundingBox> _my_boxes;
Eigen::Vector3f _wall_normals[2];

float _connect_dist = 0.4f;
float _delta_plane = 0.1f;
std::string _plyLocation;
int _toleranceFrames = 80;
bool _bGot = false;
CSegment _segment;
Eigen::Vector3f _vOffset;
geometry_msgs::Pose _camPose;
int _octomap = 0;
float _depth_scale = 0.01f;
float _depthMin = 20;
int _maxBlackSize = 3000;
int _filterSize = 3;
int _nloop = 3;
std::string _imgPath;
cv::Mat _avata;
cv::Mat _rgbMat;
cv::Mat _depthMat;
bool _bOcclusion = false;
ros::Subscriber _subKey;
ros::Timer _blend_timer;
int _ax = 0;
int _ay = 240;
int _az = 150;
int _s = 10;
bool _blendFlg = false;
float _kx = 1.0f;
float _ky = 0.0f;
int _origin_x = 0;
int _origin_y = 240;
int _nSmoothMode = 0;
//depth_edge
float _jumpThresh = 1.0f;
int _nStringSize = 20;
float _slopeThresh1 = 0.02f;
float _slopeThresh2 = 0.04f;
void blendCallback(const ros::TimerEvent&)
{
    /*static int count = 0;
    static bool bStart = false;
    if(_bGot==true)
    {
        count = 0;
        savePointClouds();
        bStart = true;
    }
    else if(bStart==true)
        ++count;
    if(bStart==true && count > 4)
    {
        savePointClouds();
        ROS_INFO("save point clouds");
        return 1;
    }
    _bGot = false;*/
    cv::Mat rgbMat = _rgbMat.clone();
    cv::Mat depthMat = _depthMat.clone();
    cv::Mat avata = _avata.clone();
    blendImages(rgbMat, avata, depthMat, _ax, _ay, _az);
    publishRGBImage(rgbMat);
    if(_ax > (_rgbMat.cols - _avata.cols/2 -1))
    {
       //_s = -10;
       _ax = _avata.cols;
       _ay = _origin_y;
    }
    /*if(_ax < (_avata.cols/2+1))
    {
       _s = 10;
    }*/
    _ax += _s;
    _ay = _origin_y - _kx*_ax;
    if(_ay < _avata.rows/2)
    {
        _ax = _avata.cols;
        _ay = _origin_y;
    }
    ROS_INFO("occlusion depth:%d", _az);
}

void keyCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
    if(msg->data=="O" || msg->data.c_str()=="O")
    {
        if(_blendFlg==false)
        {
            ros::NodeHandle n;
            _blend_timer = n.createTimer(ros::Duration(0.5), blendCallback);
            ROS_INFO("blendTimer");
            _blendFlg = true;
        }
    }
    if(msg->data=="A" || msg->data.c_str()=="A")
    {
       _az++;
    }
    if(msg->data=="D" || msg->data.c_str()=="D")
    {
        _az--;
        if(_az < 0)
           _az = 0;
    }
    if(msg->data=="W" || msg->data.c_str()=="W")
    {
        _origin_y--;
        if(_origin_y < _avata.rows)
           _origin_y = _avata.rows;
    }
    if(msg->data=="Z" || msg->data.c_str()=="Z")
    {
        _origin_y++;
        if(_origin_y > (_depthMat.rows - _avata.rows/2))
           _origin_y = _depthMat.rows - _avata.rows/2;
    }
    if(msg->data=="I" || msg->data.c_str()=="I")
    {
        _blend_timer.start();
    }
    if(msg->data=="P" || msg->data.c_str()=="P")
    {
        _blend_timer.stop();
    }
    /*if(msg->data=='o' || msg->data.c_str()=='o')
    {
        ros::NodeHandle n;
        _proc_timer = n.createTimer(ros::Duration(1.0), callback);
        ROS_INFO("process timer1");
    }*/
    /*if(msg->data=="O" || msg->data.c_str()=="O")
    {
        ros::NodeHandle n;
        _proc_timer = n.createTimer(ros::Duration(1.0), callback);
        ROS_INFO("process timer2");
    }*/
}

void mainCallback(const sensor_msgs::ImageConstPtr& msgRGB,
                   const sensor_msgs::ImageConstPtr& msgDepth,
                   const nav_msgs::OdometryConstPtr& msgOdom)
{
    /*_bGot = true;
    cv::Mat rgbMat, depthMat;
    sensor_msgs::ImagePtr color_msg, depth_msg;
    cpyImageMsg(msgRGB, color_msg, rgbMat);
    cpyImageMsg(msgDepth, depth_msg, depthMat);
    primary_process(rgbMat, depthMat, msgOdom->pose.pose);
    _cameraPosePub.publish(msgOdom);
    publishRGBImage(rgbMat);
    publishUnityCameraPose(msgOdom->pose.pose);*/
}

void imageCallback(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgDepth)
{
    ROS_INFO("start image callback");
    _rgbPub.publish(msgRGB);
    cv::Mat depthMat;
    sensor_msgs::ImagePtr depth_msg;
    cpyImageMsg(msgDepth, depth_msg, depthMat);
    int width = depthMat.cols;
    int height = depthMat.rows;
    cv::Mat rawDepthMat(cvSize(width,height), CV_8UC1, cvScalar(0));
    switch(depthMat.type())
    {
    case CV_16UC1:
        for(int y=0; y < height; ++y)
        {
            for(int x=0; x < width; ++x)
            {
                rawDepthMat.at<unsigned char>(y,x) = 0.1f*depthMat.at<unsigned short>(y,x);
            }
        }
        ROS_INFO("depth:16UC1");
        break;
    case CV_32FC1:
        for(int y=0; y < height; ++y)
        {
            for(int x=0; x < width; ++x)
            {
                rawDepthMat.at<unsigned char>(y,x) = 100*depthMat.at<float>(y,x);
            }
        }
        ROS_INFO("depth:32FC1");
        break;
    }
    sensor_msgs::ImagePtr msg_depth = cv_bridge::CvImage(std_msgs::Header(), "mono8", rawDepthMat).toImageMsg();
    _rawDepthImgPub.publish(msg_depth);
    //_rawDepthImgPub.publish(msgDepth);
    cv::Mat edgeMat(cvSize(width,height),CV_8UC1,cvScalar(0));
    //jump_edge(depthMat, edgeMat, _nStringSize, _jumpThresh);
    corner_edge(depthMat, edgeMat, _nStringSize, _slopeThresh1, _slopeThresh2);
    //curvedEdge(depthMat, edgeMat, 4, 0.04f);
    //extremumEdge(depthMat, edgeMat, _nStringSize, 1.0f);
    ROS_INFO("edge extracting is completed.");
    //cv::Mat labelMat(cvSize(width,height),CV_16UC1,cvScalar(0));
    //int nSegments = segment_depth(depthMat,edgeMat,0.02,labelMat);
    //ROS_INFO("number of segments:%d",nSegments);
    for(int y=0; y < height; ++y)
    {
        for(int x=0; x < width; ++x)
        {
           //edgeMat.at<unsigned char>(y,x) = 255*edgeMat.at<unsigned char>(y,x);
            if(edgeMat.at<unsigned char>(y,x)==1)
                edgeMat.at<unsigned char>(y,x) = 255;
            //else
                //edgeMat.at<unsigned char>(y,x) = rawDepthMat.at<unsigned char>(y,x);
        }
    }
    sensor_msgs::ImagePtr msgEdge = cv_bridge::CvImage(std_msgs::Header(), "mono8", edgeMat).toImageMsg();
    _edgeImgPub.publish(msgEdge);
    /*if(nSegments==0)
        return;
    std::vector<unsigned char> label_reds;
    std::vector<unsigned char> label_blues;
    std::vector<unsigned char> label_greens;
    label_reds.resize(nSegments);
    label_blues.resize(nSegments);
    label_greens.resize(nSegments);
    for(int i=0; i < nSegments; ++i)
    {
        label_reds[i] = rand()%255;
        label_blues[i] = rand()%255;
        label_greens[i] = rand()%255;
    }
    cv::Mat rgbLabelMat(cvSize(width, height),CV_8UC3,cvScalar(0));
    for(int y=0; y < height; ++y)
    {
        for(int x=0; x < width; ++x)
        {
            unsigned short label = labelMat.at<unsigned short>(y,x);
            if(label==0)
                continue;
            --label;
            rgbLabelMat.at<cv::Vec3b>(y,x)[0] = label_blues[label];
            rgbLabelMat.at<cv::Vec3b>(y,x)[1] = label_greens[label];
            rgbLabelMat.at<cv::Vec3b>(y,x)[2] = label_reds[label];
        }
    }
    sensor_msgs::ImagePtr labelImgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbLabelMat).toImageMsg();
    _labelImgPub.publish(labelImgMsg);*/
    //occlusion version
    /*cv::Mat rgbMat(cvSize(width,height),CV_8UC3);
    for(int y=0; y < height; ++y)
    {
        for(int x = 0; x < width; ++x)
        {
            rgbMat.at<cv::Vec3b>(y,x)[0] = grayMat.at<unsigned char>(y,x);
            rgbMat.at<cv::Vec3b>(y,x)[1] = grayMat.at<unsigned char>(y,x);
            rgbMat.at<cv::Vec3b>(y,x)[2] = grayMat.at<unsigned char>(y,x);
        }
    }
    _rgbMat = rgbMat;
    publishRGBImage(rgbMat);
    filteringDepthMap(depthMat);
    cv::Mat rawDepthMat(cvSize(width,height), CV_8UC1);
    for(int y=0; y < height; ++y)
    {
        for(int x=0; x < width; ++x)
        {
           rawDepthMat.at<unsigned char>(y,x) = depthMat.at<unsigned short>(y,x);
        }
    }
    sensor_msgs::ImagePtr msg_depth = cv_bridge::CvImage(std_msgs::Header(), "mono8", rawDepthMat).toImageMsg();
    _rawDepthPub.publish(msg_depth);
    fillHoles(depthMat, _depthMin, _maxBlackSize);
    //refineDepthMap(depthMat, _filterSize, _nloop);
    cv::Mat depthSmoothed(cvSize(width, height), CV_16UC1);
    smoothing(depthMat, _filterSize, _nSmoothMode, depthSmoothed);
    _depthMat = depthSmoothed.clone();
    cv::Mat depthf(cvSize(width, height),CV_8UC1);
    for(int y=0; y < height; ++y)
    {
        for(int x=0; x < width; ++x)
        {
           depthf.at<unsigned char>(y,x) = _depthMat.at<unsigned short>(y,x);
        }
    }
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", depthf).toImageMsg();
    _rgbPub1.publish(msg);
    ROS_INFO("finish image callback");*/
}

void initRosApp()
{
    _subscriberOdometry = new message_filters::Subscriber<nav_msgs::Odometry>;//2017/12/26
    _subscriberRGB = new message_filters::Subscriber<sensor_msgs::Image>;
    _subscriberDepth = new message_filters::Subscriber<sensor_msgs::Image>;
    ros::NodeHandle node;
    _subscriberOdometry->subscribe(node,node.resolveName("odom_pose"),15);
    _subscriberRGB->subscribe(node,node.resolveName("image_rgb"),15);
    _subscriberDepth->subscribe(node,node.resolveName("image_depth"),15);
    /*_sync_depth_rgb = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, nav_msgs::Odometry> >
    (message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, nav_msgs::Odometry>(15),
            *_subscriberRGB,*_subscriberDepth,*_subscriberOdometry);
    _sync_depth_rgb->registerCallback(boost::bind(&mainCallback, _1,  _2, _3));*/

    _sync_depth_rgb = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> >
    (message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>(15),
            *_subscriberRGB,*_subscriberDepth);
    _sync_depth_rgb->registerCallback(boost::bind(&imageCallback, _1,  _2));

    initialize();
    _segment.setClusteringParams(_object_cluster_tolerance, _cluster_tolerance, _max_cluster_size, _min_cluster_size, _delta_plane);
    _segment.setPlaneClusterParams(_quad_width_thresh, _quad_height_thresh, _quad_dot_thresh);
    _segment.setGroundAndWallHeight(_z_bottom, _wall_height);
    _segment.setObjParams(_min_object_size, _max_object_size, _object_top, _toleranceFrames);
    _segment.setWalls(_my_quads);
    _segment.setBoxes(_my_boxes);
    _vOffset[0] = 0.0f;
    _vOffset[1] = 0.0f;
    _vOffset[2] = 0.0f;

    ros::NodeHandle n;
    _subKey = n.subscribe("key", 1000, keyCallback);
}

void initialize()
{
    ros::NodeHandle nodeLocal("~");
    bool loadSuccess = true;
    loadSuccess &= nodeLocal.getParam("depthFx",fx);
    loadSuccess &= nodeLocal.getParam("depthFy",fy);
    loadSuccess &= nodeLocal.getParam("depthCx",cx);
    loadSuccess &= nodeLocal.getParam("depthCy",cy);

    loadSuccess &= nodeLocal.getParam("maxDepth", maxDepth_);
    loadSuccess &= nodeLocal.getParam("minDepth", minDepth_);
    loadSuccess &= nodeLocal.getParam("voxelSize", voxelSize_);
    loadSuccess &= nodeLocal.getParam("decimation", decimation_);
    loadSuccess &= nodeLocal.getParam("noiseFilterRadius", noiseFilterRadius_);
    loadSuccess &= nodeLocal.getParam("noiseFilterMinNeighors", noiseFilterMinNeighbors_);
    loadSuccess &= nodeLocal.getParam("normalK", normalK_);
    loadSuccess &= nodeLocal.getParam("normalRadius", normalRadius_);

    loadSuccess &= nodeLocal.getParam("leafSize",_leaf_size);
    loadSuccess &= nodeLocal.getParam("distThreshold",_distThreshold);
    loadSuccess &= nodeLocal.getParam("maxIteration",_maxIteration);

    loadSuccess &= nodeLocal.getParam("rate_rest",_rate_rest_thresh);
    loadSuccess &= nodeLocal.getParam("dot_up_wall_thresh",_dot_up_wall_thresh);
    loadSuccess &= nodeLocal.getParam("dot_quad_merged_thresh",_dot_quad_merged_thresh);
    loadSuccess &= nodeLocal.getParam("dist_quad_thresh", _dist_quad_thresh);
    loadSuccess &= nodeLocal.getParam("wall_height",_wall_height);
    loadSuccess &= nodeLocal.getParam("quad_dot_thresh",_quad_dot_thresh);
    loadSuccess &= nodeLocal.getParam("quad_width_thresh",_quad_width_thresh);
    loadSuccess &= nodeLocal.getParam("quad_height_thresh",_quad_height_thresh);
    loadSuccess &= nodeLocal.getParam("z_bottom",_z_bottom);
    //Euclidean clustering parameters
    loadSuccess &= nodeLocal.getParam("object_cluster_tolerance", _object_cluster_tolerance);
    loadSuccess &= nodeLocal.getParam("min_object_cluster_size", _min_object_cluster_size);
    loadSuccess &= nodeLocal.getParam("max_object_cluster_size", _max_object_cluster_size);
    loadSuccess &= nodeLocal.getParam("object_top", _object_top);
    loadSuccess &= nodeLocal.getParam("object_depth", _object_depth);
    loadSuccess &= nodeLocal.getParam("max_object_size", _max_object_size);
    loadSuccess &= nodeLocal.getParam("min_object_size", _min_object_size);
    loadSuccess &= nodeLocal.getParam("toleranceFrames", _toleranceFrames);
    loadSuccess &= nodeLocal.getParam("cluster_tolerance", _cluster_tolerance);
    loadSuccess &= nodeLocal.getParam("min_cluster_size", _min_cluster_size);
    loadSuccess &= nodeLocal.getParam("max_cluster_size", _max_cluster_size);
    loadSuccess &= nodeLocal.getParam("connect_dist", _connect_dist);
    loadSuccess &= nodeLocal.getParam("delta_plane", _delta_plane);
    loadSuccess &= nodeLocal.getParam("plyLocation", _plyLocation);
    loadSuccess &= nodeLocal.getParam("octomap", _octomap);
    loadSuccess &= nodeLocal.getParam("depth_scale", _depth_scale);
    loadSuccess &= nodeLocal.getParam("depthMin", _depthMin);
    loadSuccess &= nodeLocal.getParam("maxBlackSize", _maxBlackSize);
    loadSuccess &= nodeLocal.getParam("filterSize", _filterSize);
    loadSuccess &= nodeLocal.getParam("nloop", _nloop);
    loadSuccess &= nodeLocal.getParam("imgPath", _imgPath);
    loadSuccess &= nodeLocal.getParam("nSmoothMode", _nSmoothMode);
    //depth edge
    loadSuccess &= nodeLocal.getParam("jumpThresh", _jumpThresh);
    loadSuccess &= nodeLocal.getParam("nStringSize", _nStringSize);

    loadSuccess &= nodeLocal.getParam("slopeThresh1", _slopeThresh1);
    loadSuccess &= nodeLocal.getParam("slopeThresh2", _slopeThresh2);

    _avata = cv::imread(_imgPath.c_str());
    _ax = _avata.cols/2 + 1;
    ROS_INFO("avata width:%d, avata height:%d", _avata.cols, _avata.rows);
    ros::NodeHandle nh;
    _cloudPub = nh.advertise<sensor_msgs::PointCloud2>("global_cloud", 1);
    _cloudPubForOctoMap = nh.advertise<sensor_msgs::PointCloud2>("global_cloud", 1);
    _wall_lines_pub = nh.advertise<visualization_msgs::Marker>("WallQuad/lines", 15);
    _wall_vertices_pub = nh.advertise<visualization_msgs::Marker>("WallQuad/points", 15);
    _box_lines_pub = nh.advertise<visualization_msgs::Marker>("BoundingBox/lines", 15);
    _box_vertices_pub = nh.advertise<visualization_msgs::Marker>("BoundingBox/points", 15);
    image_transport::ImageTransport it(nh);
    _rgbPub = it.advertise("camera/image", 15);
    _labelImgPub = it.advertise("camera/labelImage", 15);
    _edgeImgPub = it.advertise("camera/edgeImage", 15);
    _rawDepthImgPub = it.advertise("camera/rawDepthImage", 15);
    _smoothDepthImgPub = it.advertise("camera/smoothDepthImage", 15);
    _cameraPosePub = nh.advertise<nav_msgs::Odometry>("camera/pose", 15);
    _unityCameraPosePub = nh.advertise<visualization_msgs::Marker>("camera/unityPose", 15);
    _tf_pub = nh.advertise<tf::tfMessage>("tf", 15);
}
//generating point cloud
void cpyImageMsg(const sensor_msgs::ImageConstPtr& msg,
                 sensor_msgs::ImagePtr &img,
                 cv::Mat &cv_image)
{
    boost::shared_ptr<void const> tracked_object;
    try
    {
      cv_image = cv_bridge::toCvShare(*msg, tracked_object, msg->encoding)->image.clone();
    }
    catch (cv::Exception &e)
    {
      ROS_ERROR("Could not convert from '%s' to '%s'.", msg->encoding.c_str(), msg->encoding.c_str());
      return;
    }
    img = cv_bridge::CvImage(msg->header, msg->encoding, cv_image).toImageMsg();/*"bgr8"*/
}

bool cloudFromDepthRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pclCloud, cv::Mat& rgbMat, cv::Mat& depthMat)
{
    pcl::IndicesPtr indices(new std::vector<int>);
    pclCloud = cloudFromDepthRGB(rgbMat, depthMat, fx, cx, fy, cy, decimation_, maxDepth_, minDepth_, indices.get());
    ROS_WARN("cloud from depth finish");
    if(!pclCloud)
        return false;
    ROS_INFO("pclCloud size:%d", pclCloud->size());
    ROS_INFO("validate cloud size:%d", indices->size());
    if(pclCloud->size()==0)
        return false;
    if(indices->size()==0)
        return false;
    filteringCloud(pclCloud, indices);
    return true;
}

void filteringCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pclCloud, pcl::IndicesPtr & indices)
{
    if(indices->size() && voxelSize_ > 0.0)
    {
        pclCloud = voxelize(pclCloud, indices, voxelSize_);
    }
    // Do radius filtering after voxel filtering ( a lot faster)
    if(pclCloud->size() && noiseFilterRadius_ > 0.0 && noiseFilterMinNeighbors_ > 0)
    {
        if(pclCloud->is_dense)
        {
            ROS_INFO("befroe dense filter pclCloud size:%d", pclCloud->size());
            indices = radiusFiltering(pclCloud, noiseFilterRadius_, noiseFilterMinNeighbors_);
            ROS_INFO("after dense filter pclCloud size:%d", pclCloud->size());
        }
        else
        {
            ROS_INFO("before non-dense filter pclCloud size:%d", pclCloud->size());
           indices = radiusFiltering(pclCloud, indices, noiseFilterRadius_, noiseFilterMinNeighbors_);
            ROS_INFO("after non-dense filter pclCloud size:%d", pclCloud->size());
        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*pclCloud, *indices, *tmp);
        pclCloud = tmp;
        ROS_INFO("before remove NanPoint pclCloud size:%d", pclCloud->size());
        pclCloud = removeNaNFromPointCloud(pclCloud);
        ROS_INFO("after remove NanPoint pclCloud size:%d", pclCloud->size());
    }
}
/*
void getIniCameraRot()
{
    if(_my_quads.size()!=1)
        return;
    Eigen::Vector3f vNormal;
    _my_quads[0].getNormal(vNormal);
    Eigen::Vector3f vForward(1.0f,0.0f,0.0f);
    Eigen::Vector3f vRight(0.0f,1.0f,0.0f);
    float dot1 = fabs(vForward.dot(vNormal));
    float dot2 = fabs(vRight.dot(vNormal));
    if(dot1 > dot2)
    {
        _init_cam_rot = Eigen::Quaternionf::FromTwoVectors(vNormal,vForward);
    }
    else
        _init_cam_rot = Eigen::Quaternionf::FromTwoVectors(vNormal,vRight);
}*/

void processForOctomap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pCloud)
{
    /*pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i=0; i < pCloud->points.size(); ++i)
    {
        if(pCloud->points[i].z < 1.5f)
        {
            cloudOut->points.push_back(pCloud->points[i]);
        }
    }*/
    sensor_msgs::PointCloud2 rosCloud;
    pcl::toROSMsg(*pCloud, rosCloud);
    rosCloud.header.frame_id = "cloud";
    _cloudPubForOctoMap.publish(rosCloud);
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(_camPose.position.x, _camPose.position.y, _camPose.position.z) );
    tf::Quaternion q(_camPose.orientation.x,_camPose.orientation.y,_camPose.orientation.z,_camPose.orientation.w);
    //q.setRPY(0, 0, msg->theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "cloud"));
}

void primary_process(cv::Mat &rgbMat, cv::Mat &depthMat, const geometry_msgs::Pose &pose)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud;
    if(cloudFromDepthRGB(pCloud, rgbMat, depthMat)==false)
        return;
    _camPose.position.x = pose.position.x;
    _camPose.position.y = pose.position.y;
    _camPose.position.z = pose.position.z;
    _camPose.orientation.x = pose.orientation.x;
    _camPose.orientation.y = pose.orientation.y;
    _camPose.orientation.z = pose.orientation.z;
    _camPose.orientation.w = pose.orientation.w;

    if(_octomap==1)
    {
        processForOctomap(pCloud);
    }
    else
    {   //new version
        /*transformPointCloud(pCloud, pose);
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
        //downSampling(pCloud, _leaf_size, cloud_downsampled);
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudObject(new pcl::PointCloud<pcl::PointXYZRGB>);;
        //int nState = _segment.main_process(pCloud, cloudObject);
        //if(nState==-1)
            //return;
        sensor_msgs::PointCloud2 rosCloud;
        _total_cloud += *pCloud;//*cloudObject;
        pcl::toROSMsg(_total_cloud, rosCloud);
        rosCloud.header.frame_id = "world";
        _cloudPub.publish(rosCloud);
        //_segment.getWallsAndBoxes(_my_quads, _my_boxes);
        //publishWallLines();
        //publishBoxLines();*/
    }
    /*pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
    downSampling(pCloud, _leaf_size, cloud_downsampled);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    cutingCloud(cloud_downsampled, cloud_filtered);
    main_process(cloud_filtered, pose);
    sensor_msgs::PointCloud2 rosCloud;
    _total_cloud += *cloud_filtered;
    pcl::toROSMsg(_total_cloud, rosCloud);
    rosCloud.header.frame_id = "world";
    _cloudPub.publish(rosCloud);
    publishWallLines();
    publishBoxLines();
    publishWallQuads();
    publishObjectBoxVertices();*/
}

void publishWallQuads()
{
    visualization_msgs::Marker points;//line_list;
    points.header.frame_id = "world";
    points.type = visualization_msgs::Marker::POINTS;
    points.action = visualization_msgs::Marker::ADD;
    //line_list.pose.orientation.w = 1.0;
    points.scale.x = 0.02;
    points.color.g = 1.0;
    points.color.a = 1.0;
    geometry_msgs::Point p;
    for(int i=0; i < _my_quads.size(); ++i)
    {
        std::vector<Eigen::Vector3f> vertices;
        _my_quads[i].getVertices(vertices);
        for(int j=0; j <4; ++j)
        {
            p.x = vertices[j][0];
            p.y = vertices[j][1];
            p.z = vertices[j][2];// + _camera_height;
            points.points.push_back(p);
        }
    }
    _wall_vertices_pub.publish(points);
}

void publishWallLines()
{
    if(_my_quads.size()==0)
        return;
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "world";
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    //line_list
    line_list.scale.x = 0.02;
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    geometry_msgs::Point p;
    for(int i=0; i < _my_quads.size(); ++i)
    {
        std::vector<Eigen::Vector3f> vertices;
        _my_quads[i].getLineVertices(vertices);
        for(int j=0; j < 8; ++j)
        {
            p.x = vertices[j][0];
            p.y = vertices[j][1];
            p.z = vertices[j][2];
            line_list.points.push_back(p);
        }
    }
    _wall_lines_pub.publish(line_list);
}

void publishObjectBoxVertices()
{
    visualization_msgs::Marker points;
    points.header.frame_id = "world";
    points.type = visualization_msgs::Marker::LINE_LIST;
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.scale.x = 0.02;
    points.color.g = 1.0;
    points.color.a = 1.0;
    geometry_msgs::Point p;
    for(int i=0; i < _my_boxes.size(); ++i)
    {
        std::vector<Eigen::Vector3f> vertices;
        _my_boxes[i].getVertices(vertices);
        for(int j=0; j <8; ++j)
        {
            p.x = vertices[j][0];
            p.y = vertices[j][1];
            p.z = vertices[j][2];// + _camera_height;
            points.points.push_back(p);
        }
    }
    _box_vertices_pub.publish(points);
}

void publishBoxLines()
{
    if(_my_boxes.size()==0)
        return;
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "world";
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    //line_list
    line_list.scale.x = 0.02;
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;
    geometry_msgs::Point p;
    for(int i=0; i < _my_boxes.size(); ++i)
    {
        std::vector<Eigen::Vector3f> wire_lines;
        _my_boxes[i].getWireFrame(wire_lines);
        for(int j=0; j < 48; ++j)
        {
            p.x = wire_lines[j][0];
            p.y = wire_lines[j][1];
            p.z = wire_lines[j][2];
            line_list.points.push_back(p);
        }
    }
    _box_lines_pub.publish(line_list);
}

void publishRGBImage(cv::Mat &rgbMat)
{
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbMat).toImageMsg();
    _rgbPub.publish(msg);
}

void publishUnityCameraPose(const geometry_msgs::Pose &pose)
{
    Eigen::Vector3f forward(1.0f, 0.0f, 0.0f);
    Eigen::Vector3f position(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaternionf rot(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
    forward = rot.toRotationMatrix()*forward;
    Eigen::Vector3f upward(0.0f,0.0f,1.0f);
    upward = rot.toRotationMatrix()*upward;
    visualization_msgs::Marker points;//line_list;
    points.header.frame_id = "world";
    points.type = visualization_msgs::Marker::POINTS;
    points.action = visualization_msgs::Marker::ADD;
    //line_list.pose.orientation.w = 1.0;
    points.scale.x = 0.02;
    points.color.g = 1.0;
    points.color.a = 1.0;
    geometry_msgs::Point p;
    p.x = position[0];
    p.y = position[1];
    p.z= position[2];
    points.points.push_back(p);
    p.x = forward[0];
    p.y = forward[1];
    p.z = forward[2];
    points.points.push_back(p);
    p.x = upward[0];
    p.y = upward[1];
    p.z = upward[2];
    points.points.push_back(p);
    _unityCameraPosePub.publish(points);
}


std::string intToString ( int number )
{
  std::ostringstream oss;

  // Works just like cout
  oss<< number;

  // Return the underlying string
  return oss.str();
}

void savePointClouds()
{
    /*for(int i=0; i < _my_boxes.size(); ++i)
    {
        std::string plyPath = _plyLocation + intToString(i) + ".ply";
        pcl::io::savePLYFileBinary(plyPath, *_my_boxes[i].getCloudPtr());
    }*/
    std::string plyPath = _plyLocation + "panda" + ".ply";
    pcl::io::savePLYFileBinary(plyPath, _total_cloud);
}

void setOcclusion(bool flg)
{
    _bOcclusion = flg;
}


