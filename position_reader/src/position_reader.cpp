/*
 * position_reader.cpp
 *
 *  Created on: 16.09.2013
 *      Author: phil
 */


/*#include <vector>
#include <string>
#include <cmath>*/
#include "PositionReader.h"

#include <ros/ros.h>
/*#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>

#include "sound_player/AudioType.h"

#define FOUND 0.6
#define STEP 0.45
#define STEPS 7*/

//void positionCallback(const gazebo_msgs::ModelStates& modelState);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_reader");
  PositionReader positionReader;
  positionReader.start();

  /*ros::NodeHandle nh;
  ros::ServiceClient positionReaderClient = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  ros::ServiceClient victPosReaderClient = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  ros::Publisher typePub = nh.advertise<sound_player::AudioType>("audio_type", 1);
  sound_player::AudioType audioType;
 // ros::Subscriber positionReader = nh.subscribe("/gazebo/model_states", 10, &positionCallback);

  gazebo_msgs::GetModelState req;
  gazebo_msgs::GetModelState vicReq;

  req.request.model_name = "rescue_robot";
  vicReq.request.model_name = "vicbox";
  //req.request.relative_entity_name = "r2d2";
  //req.request.relative_entity_name = "plane_model";
  ros::Rate rate(10);
  double dist = 0.0;
  while(ros::ok())
  {
     //ros::spinOnce();
    positionReaderClient.call(req);
    victPosReaderClient.call(vicReq);
    dist = std::sqrt(
        (req.response.pose.position.x - vicReq.response.pose.position.x) * (req.response.pose.position.x - vicReq.response.pose.position.x) +
        (req.response.pose.position.y - vicReq.response.pose.position.y) * (req.response.pose.position.y - vicReq.response.pose.position.y)// +
        //(req.response.pose.position.z - vicReq.response.pose.position.z) * (req.response.pose.position.z - vicReq.response.pose.position.z)
        );
    std::cout << "\nPosition reader got robot position:\nx = " << req.response.pose.position.x
              << "\ny = " << req.response.pose.position.y << "\nz = "
              << req.response.pose.position.z << "\n";
    std::cout << "\nPosition reader got victim position:\nx = " << vicReq.response.pose.position.x
                  << "\ny = " << vicReq.response.pose.position.y << "\nz = "
                  << vicReq.response.pose.position.z << "\n";
    std::cout << __PRETTY_FUNCTION__ << " distance = " << dist << "\n";
    if(dist > FOUND)
    {
      audioType.type = audioType.TYPE_APPROACH;
      if((dist > (FOUND + STEP * 1)) && (dist < (FOUND + STEP * 2)))
        audioType.factor = 1;
      else if((dist > (FOUND + STEP * 2)) && (dist < (FOUND + STEP * 3)))
        audioType.factor = 2;
      else if((dist > (FOUND + STEP * 3)) && (dist < (FOUND + STEP * 4)))
        audioType.factor = 3;
      else if((dist > (FOUND + STEP * 4)) && (dist < (FOUND + STEP * 5)))
        audioType.factor = 4;
      else if((dist > (FOUND + STEP * 5)) && (dist < (FOUND + STEP * 6)))
        audioType.factor = 5;
      else if((dist > (FOUND + STEP * 6)) && (dist < (FOUND + STEP * 7)))
        audioType.factor = 6;
      else if((dist > (FOUND + STEP * 7)) && (dist < (FOUND + STEP * 8)))
        audioType.factor = 7;
      else
        continue;
    }
    else if(dist < FOUND)
      audioType.type = audioType.TYPE_FOUND;
    typePub.publish(audioType);
    std::cout << __PRETTY_FUNCTION__ << " to be published: factor = " << audioType.factor << "\n";
    sleep(3);
    //rate.sleep();
  }*/
}

/*void positionCallback(const gazebo_msgs::ModelStates& modelState)
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
}*/

//rosservice call /gazebo/get_model_state '{model_name: rescue_robot}'
