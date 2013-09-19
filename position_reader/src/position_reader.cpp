/*
 * position_reader.cpp
 *
 *  Created on: 16.09.2013
 *      Author: phil
 */

#include <vector>
#include <string>

#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>

void positionCallback(const gazebo_msgs::ModelStates& modelState);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_reader");
  ros::NodeHandle nh;
  ros::ServiceClient positionReaderClient = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
 // ros::Subscriber positionReader = nh.subscribe("/gazebo/model_states", 10, &positionCallback);

  gazebo_msgs::GetModelState req;
  req.request.model_name = "rescue_robot";
  //req.request.relative_entity_name = "plane_model";
  ros::Rate rate(10);
  while(ros::ok())
  {
     //ros::spinOnce();
    positionReaderClient.call(req);
    std::cout << "\nPosition reader got position:\nx = " << req.response.pose.position.x
              << "\ny = " << req.response.pose.position.y << "\nz = "
              << req.response.pose.position.z << "\n";
    rate.sleep();
  }
}

void positionCallback(const gazebo_msgs::ModelStates& modelState)
{
   unsigned int idx = 0;
   for(std::vector<std::string>::const_iterator iter = modelState.name.begin(); iter != modelState.name.end(); iter++)
   {
      if(*iter == "rescue_robot")
         break;
      else
      {
      //   std::cout << "Model: " << *iter << "\n";
         ++idx;
      }
   }
   std::cout << "\nPosition reader got position:\nx = " << modelState.pose[idx].position.x
                << "\ny = " << modelState.pose[idx].position.y << "\nz = "
                << modelState.pose[idx].position.z << "\n";//Success ?!: " << static_cast<unsigned char>(req.response.success) << "\n";
}

//rosservice call /gazebo/get_model_state '{model_name: rescue_robot}'
