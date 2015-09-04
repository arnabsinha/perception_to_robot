#ifndef _PERCEPTION_TO_ROBOT_NODE_H_
#define _PERCEPTION_TO_ROBOT_NODE_H_

#include <ros/ros.h>

#include <cstring>
#include <vector>
#include <fstream>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <url3Msg/url3Msg.h>

using namespace std;
using namespace cv;
using namespace sensor_msgs;

class perception{
    private:
        ros::NodeHandle nh_, nh;
        ros::Subscriber pclSub;
        ros::Publisher pubRobComm;
        ros::Publisher pubPcl;
        tf::TransformListener *listener;
        vector<Point3f> points;
        string input_topic;
		  bool doIt;

        void cloudCallBack(const sensor_msgs::PointCloud2ConstPtr& input);
        
    public:
        perception();
        ~perception();
};

#endif
