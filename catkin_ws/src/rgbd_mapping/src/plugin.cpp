#include "main_process.h"
#include "plugin.h"
#include <visualization_msgs/Marker.h>
const int width =  224;
const int height = 376;
unsigned char *rgb_bytes;
std::vector<Eigen::Vector3f> _quadVertices;
std::vector<Eigen::Vector3f> _boxVertices;
image_transport::Subscriber color_sub;
ros::Subscriber sub_pose;
ros::Subscriber sub_box_points;
ros::Subscriber sub_quad_points;
cv::Mat _rgbMat;
bool _bColorCaptured = false;
float _pose[7];
/*
the pose can not be synced with marker messages computed in rgbd_mapping node.
so wire frame vertices must be transformed into camera space in rgbd_mapping node.
*/
//callback functions
void color_imageCallback(const sensor_msgs::ImageConstPtr& msgRGB)
{
    _bColorCaptured = true;
    sensor_msgs::ImagePtr rgb_msg;
    cpyImageMsg(msgRGB, rgb_msg, _rgbMat);
}

void boxCallback(const visualization_msgs::Marker::ConstPtr& markerMsg)
{
    _boxVertices.clear();
    for(int i=0; i < markerMsg->points.size(); ++i)
    {
        Eigen::Vector3f p;
        p[0] = markerMsg->points[i].x;
        p[1] = markerMsg->points[i].y;
        p[2] = markerMsg->points[i].z;
        _boxVertices.push_back(p);
    }
}

void quadCallback(const visualization_msgs::Marker::ConstPtr& markerMsg)
{
    _quadVertices.clear();
    for(int i=0; i < markerMsg->points.size(); ++i)
    {
        Eigen::Vector3f p;
        p[0] = markerMsg->points[i].x;
        p[1] = markerMsg->points[i].y;
        p[2] = markerMsg->points[i].z;
        _quadVertices.push_back(p);
    }
}

bool getRGBBytes(unsigned char *rgb_bytes, int width, int height)
{
    if(_bColorCaptured==false)
        return false;
    int imgfillcount=0;
    for(int i = 0;i<width;i++)
    {
        for(int j = 0; j < height; j++)
        {
            int b = _rgbMat.at<cv::Vec3b>(j,i)[0];
            int g = _rgbMat.at<cv::Vec3b>(j,i)[1];
            int r = _rgbMat.at<cv::Vec3b>(j,i)[2];
            rgb_bytes[imgfillcount]=(unsigned char)r;
            imgfillcount++;
            rgb_bytes[imgfillcount]=(unsigned char)g;
            imgfillcount++;
            rgb_bytes[imgfillcount]=(unsigned char)b;
            imgfillcount++;
        }
    }
    _bColorCaptured = false;
    return true;
}

void poseCallback(const nav_msgs::OdometryConstPtr& msgOdom)
{
    _pose[0] = msgOdom->pose.pose.position.x;
    _pose[1] = msgOdom->pose.pose.position.y;
    _pose[2] = msgOdom->pose.pose.position.z;
    _pose[3] = msgOdom->pose.pose.orientation.w;
    _pose[4] = msgOdom->pose.pose.orientation.x;
    _pose[5] = msgOdom->pose.pose.orientation.y;
    _pose[6] = msgOdom->pose.pose.orientation.z;
}

//plugin functions (unity has access to)
void initplugin(const char *thepath)
{
  rgb_bytes=new unsigned char[width*height*3];
  int charcount=0;
  for(int i=0;thepath[i]!='\0';i++){
     charcount++;
  }
  char *argv=new char[charcount+1];
  for(int i=0;i<charcount+1;i++){
     argv[i]=thepath[i];
  }
  int argc=1;
  ros::init(argc, &argv, "rosbagunityreadplugin");
  ros::NodeHandle sub_nh;
  image_transport::ImageTransport sub_it(sub_nh);
  color_sub = sub_it.subscribe("camera/image", 15, color_imageCallback);//"/asus/rgb/image_raw"
  sub_box_points = sub_nh.subscribe("BoundingBox/points", 15, boxCallback);
  sub_quad_points = sub_nh.subscribe("WallQuad/points", 15, quadCallback);
  sub_pose = sub_nh.subscribe("/asus/odometry", 15, poseCallback);
}

void getpose(float *pose)
{
   for(int i=0; i < 7; ++i)
       pose[i] = _pose[i];
}

void getNewData(int *imgBuf, float *quadBuf, float *boxBuf, int *numVertices, float *poseBuf)
{
    if(!getRGBBytes(rgb_bytes, width, height))
    {
        ros::spinOnce();
        return;
    }
    for(int i=0;i<width*height*3;i++)
    {
       imgBuf[i]=(int)rgb_bytes[i];
    }
    numVertices[0] = _quadVertices.size();
    numVertices[1] = _boxVertices.size();
    if(numVertices[0]==0 || numVertices[1]==0)
        return;
    int count = 0;
    for(int i=0; i < _quadVertices.size(); ++i)
    {
        quadBuf[count] = _quadVertices[i][0];
        quadBuf[count++] = _quadVertices[i][1];
        quadBuf[count++] = _quadVertices[i][2];
        count++;
    }
    count = 0;
    for(int i=0; i < _boxVertices.size(); ++i)
    {
        boxBuf[count] = _boxVertices[i][0];
        boxBuf[count++] = _boxVertices[i][1];
        boxBuf[count++] = _boxVertices[i][2];
        count++;
    }
    for(int i=0; i < 7; ++i)
    {
        poseBuf[i] = _pose[i];
    }
    ros::spinOnce();
}

int getwidth()
{
    return width;
}

int getheight()
{
    return height;
}


