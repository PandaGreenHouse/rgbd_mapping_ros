#include "main_process.h"
//bool flg = false;
//ros::Timer timer;
//void callback(const ros::TimerEvent&)
//{
    //if(flg==true)
     //  timer_callback();
    /*int i = timer_callback();
    if(i==1)
    {
       timer.stop();
       ROS_INFO("timer stop");
    }*/
//}

/*void keyCallback(const std_msgs::String::ConstPtr& msg)
{
   if(msg->data.c_str()=="o")
   {
       flg = true;
   }
   else
       flg = false;
   ROS_INFO("I heard: [%s]", msg->data.c_str());
}*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_mapping");
    initRosApp();
    //ros::NodeHandle n;
    //ros::Subscriber sub = n.subscribe("key", 1000, keyCallback);
    //timer = n.createTimer(ros::Duration(1.0), callback);
    ros::spin();
    return 0;
}
