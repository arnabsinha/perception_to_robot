#include "perception_to_robot_node.h"

template<typename T>
T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue)
{
        T v;
        if (n.getParam(name, v))
        {
                ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
                return v;
        }
        else
                ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
        return defaultValue;
}

perception::perception()
    :nh("~")
{
    string input_pcl_topic = getParam<string>(nh, "input_pcl_topic", "/pickIn/pose_estimate_ASTAR/objectPoints");
    input_topic = getParam<string>(nh,"goal","start");
    pclSub = nh.subscribe(input_pcl_topic,1,&perception::cloudCallBack, this);
    listener = new tf::TransformListener();
	 doIt = true;
    string output_topic = getParam<string>(nh, "output_topic", "/robot/move/command");
    pubRobComm = nh.advertise<url3Msg::url3Msg> (output_topic,10);
    
    string output_pcl_topic = getParam<string>(nh, "output_pcl_topic", "/robot/move/command");
    pubPcl = nh.advertise<pcl::PointCloud <pcl::PointXYZ> > (output_pcl_topic,10);
}

void perception::cloudCallBack(const sensor_msgs::PointCloud2ConstPtr& input){
    sensor_msgs::PointCloud2 trC;
    pcl::PointCloud<pcl::PointXYZ> pcl_out, pcl_tr;
    fromROSMsg(*input,pcl_out);
    pcl_out.header.frame_id = "/kinect2_rgb_optical_frame";
    pcl_tr.header.frame_id = "/robot_base";
    listener->waitForTransform("/robot_base", (*input).header.frame_id, (*input).header.stamp, ros::Duration(5.0));
    pcl_ros::transformPointCloud("/robot_base", pcl_out, pcl_tr, *listener);
    points.clear();
	 doIt = true;
    for(int i=0;i<3;i++){
        Point3f pt;
        pt.x = pcl_tr.points[i].x;
        pt.y = pcl_tr.points[i].y;
        pt.z = pcl_tr.points[i].z;
		  if(pcl_out.points[i].z == 0.0)
				doIt = false;
        points.push_back(pt);
//        cout<<pcl_out.points[i].x<<" "<<pcl_out.points[i].y<<" "<<pcl_out.points[i].z<<endl;
    }
	if(doIt){
        pcl::PointCloud<pcl::PointXYZ> clPub;
        for(int i=0;i<3;i++){
            pcl::PointXYZ pt;
            pt.x = points[i].x;
            pt.y = points[i].y;
            pt.z = points[i].z;
            clPub.points.push_back(pt);
        }
	
    Point3f px = points[2] - points[0];
    Point3f py = points[1] - points[0];
	 Point3f pO = points[0];
    double tx = norm(px);
    double ty = norm(py);
    points[0] = px * (1.0/tx);
    points[1] = py * (1.0/ty);
    points[2] = points[0].cross(points[1]);
	 points[2] = points[2] * (1.0/norm(points[2]));

	px = pO + 0.01 * points[0];
    cout<<input_topic<<endl;
	cout<<px<<endl;
    Point3f Vx = points[0];
    Point3f Wx;
    Wx.x = 1.0;
    Wx.y = 0.0;
    Wx.z = 0.0;
    Point3f Wz;
    Wz.x = 0.0;
    Wz.y = 0.0;
    Wz.z = 1.0;
    Point3f C = Vx.cross(Wx);
    double angle = atan2(C.ddot(Wz), Vx.ddot(Wx));
    cout<<angle<<endl;
        url3Msg::url3Msg robotMsg;
        robotMsg.x = px.x;
        robotMsg.y = px.y;
        robotMsg.z = px.z;
        robotMsg.a = 0.0;
        robotMsg.b = 0.0;
        robotMsg.c = angle;
        robotMsg.name = input_topic;
        pubRobComm.publish(robotMsg);

        pcl::PointXYZ pt;
        pt.x = px.x;
        pt.y = px.y;
        pt.z = px.z+1.2;
        clPub.points.push_back(pt);
        clPub.header.frame_id = "/robot_base";
        pubPcl.publish(clPub);
	}
}

perception::~perception(){

}

int main(int argc, char* argv[]){
    ros::init(argc,argv,"perception_node");
    perception TT;
    ros::spin();
    return 0;
}
