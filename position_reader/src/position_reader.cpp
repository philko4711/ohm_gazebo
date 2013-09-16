/*
 * position_reader.cpp
 *
 *  Created on: 16.09.2013
 *      Author: phil
 */


#include <ros/ros.h>
#include <gazebo/GetModelState.h>
#include <geometry_msgs/Point.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_reader");
  ros::NodeHandle nh;
  ros::ServiceClient positionReader;
  gazebo::GetModelState req;
  req.request.model_name = "/rescue_robot";
  ros::Rate rate(10);
  while(ros::ok())
  {
    positionReader.call(req);
    std::cout << "\nPosition reader got position:\nx = " << req.response.pose.position.x
              << "\ny = " << req.response.pose.position.y << "\nz = "
              << req.response.pose.position.y << "\n";
    rate.sleep();
  }
}

//rosservice call /gazebo/get_model_state '{model_name: rescue_robot}'
