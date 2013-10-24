/*
 * rotation_reader.cpp
 *
 *  Created on: 24.10.2013
 *      Author: phil
 */

#include <ros/ros.h>
//#include </opt/ros/fuerte/stacks/simulator_gazebo/gazebo_msgs/srv_gen/cpp/include/gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/tf.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rotation_reader_node");
	ros::NodeHandle nh;
	ros::ServiceClient positionReaderClient = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	gazebo_msgs::GetModelState robotReq;
	robotReq.request.model_name = "rescue_robot";
	ros::Rate rate(10);
	while(ros::ok())
	{
		if(!positionReaderClient.call(robotReq))
		{
			std::cout << __PRETTY_FUNCTION__ << " call of service failed!\n";
			continue;
		}
		else
		{
			std::cout << "\nPosition reader got robot position:\nx = " << robotReq.response.pose.position.x
			               << "\ny = " << robotReq.response.pose.position.y << "\nz = "
			               << robotReq.response.pose.position.z << "\n";
			tf::Quaternion quat(robotReq.response.pose.orientation.x, robotReq.response.pose.orientation.y,
			    robotReq.response.pose.orientation.z, robotReq.response.pose.orientation.w);
			tf::Matrix3x3 m(quat);
			double roll = 0.0;
			double pitch = 0.0;
			double yaw = 0.0;
			m.getRPY(roll, pitch, yaw);
			std::cout << "\nPosition reader got robot angles:\nroll = " << roll
			                           << "\npitch = " << pitch << "\nyaw = "
			                           << yaw << "\n";
		}
		rate.sleep();
	}
}


