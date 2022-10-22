//********************************************************************//
// Author: Justin Lim
// Purpose: Create an arrow at the position of the left sprayer that
// 	points to weeds (simulates movement of left sprayer)
//********************************************************************//


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

int main( int argc, char** argv )
{
    ros::init(argc, argv, "left_sprayer_arrow");
    ros::NodeHandle n;
    ros::Rate r(5);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);


    //create a listener for testweed location relative to left_sprayer
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    while (ros::ok())
    {
	geometry_msgs::TransformStamped weed_pos;
	try{
	    weed_pos = tfBuffer.lookupTransform("left_sprayer", "testweed", ros::Time(0));
	}
	catch(tf2::TransformException &ex){
	    ROS_WARN("%s", ex.what());
	    ros::Duration(1.0).sleep();
	    continue;
	}

        //ROS_INFO("%f, %f, %f", weed_pos.transform.translation.x, weed_pos.transform.translation.y, weed_pos.transform.translation.z);

        //calculate angles using formula from sprayer.cpp
        double angle2 = atan(abs(weed_pos.transform.translation.y)/abs(weed_pos.transform.translation.z));
        double angle1 = atan(weed_pos.transform.translation.x/sqrt(pow(weed_pos.transform.translation.z,2) + pow(weed_pos.transform.translation.y,2)));

        //find the quaternion values
        tf2::Quaternion q_orig, q_rot, q_new;
        //apply outer rotation first (angle2)
        q_orig.setRPY(0, M_PI/2, 0);
        q_rot.setRPY(angle2, 0, 0);
        q_orig = q_rot*q_orig;
        //apply inner rotation (angle1)
        q_rot[0] = 0.0;
        q_rot[1] = sin(angle1/2)*-1*cos(angle2);
        q_rot[2] = sin(angle1/2)*-1*sin(angle2);
        q_rot[3] = cos(angle1/2);
        q_new = q_rot*q_orig;
        q_new.normalize();


        visualization_msgs::Marker marker;
        //setting marker info
	//the frame at which the marker is relative to:
        marker.header.frame_id = "left_sprayer";
	marker.header.stamp = ros::Time::now();

	//IDs, shape, and action:
	marker.ns = "sprayer_sim";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	//positioned exactly at the sprayer:
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	tf2::convert(q_new, marker.pose.orientation);
	//this sets the size of the arrow with x being the length:
	marker.scale.x = 0.5;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	//green and not transparent:
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;
	//lasts as long as possible:
	marker.lifetime = ros::Duration();

	//publish the marker
	marker_pub.publish(marker);
	ros::spinOnce();
	r.sleep();
    }

    return 0;
}

