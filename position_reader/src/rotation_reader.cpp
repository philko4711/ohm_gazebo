/*
 * rotation_reader.cpp
 *
 *  Created on: 24.10.2013
 *      Author: phil
 */

#include <ros/ros.h>
//#include </opt/ros/fuerte/stacks/simulator_gazebo/gazebo_msgs/srv_gen/cpp/include/gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetModelState.h>

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
			               << robotReq.response.pose.position.z << "\nanglejj = "  << robotReq.response.pose.orientation.;
		}
		rate.sleep();
	}
}


