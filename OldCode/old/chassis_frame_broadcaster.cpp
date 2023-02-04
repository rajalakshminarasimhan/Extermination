// Authors: Justin L
// Purpose: Broadcaster takes messages from /position and broadcasts it to frame "chassis"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	static tf2_ros:: TransformBroadcaster br;
	geometry_msgs::TransformStamped ts;
	
	//frame transform information
	ts.header.stamp = msg->header.stamp;
	ts.header.frame_id = "world";
	ts.child_frame_id = "chassis";
	
	ts.transform.translation.x = msg->pose.position.x;
	ts.transform.translation.y = msg->pose.position.y;
	ts.transform.translation.z = msg->pose.position.z;
	ts.transform.rotation.x = msg->pose.orientation.x;
	ts.transform.rotation.y = msg->pose.orientation.y;
	ts.transform.rotation.z = msg->pose.orientation.z;
	ts.transform.rotation.w = msg->pose.orientation.w;
	
	br.sendTransform(ts); // TODO this will fail if the world frame has not been created yet
                              // should wrap this in a try {}catch(){} block
}

int main(int argc, char** argv){
	//initialize node with name chassis_broadcaster
	ros::init(argc, argv, "chassis_broadcaster");
	
	//node is subscribed to topic position and broadcasts transforms with buffer of 1000
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/chassis_pose", 1000, &poseCallback);
	
	ros::spin();
	return 0;
};
