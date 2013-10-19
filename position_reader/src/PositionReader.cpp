/*
 * PositionReader.cpp
 *
 *  Created on: 19.10.2013
 *      Author: phil
 */

#include "PositionReader.h"

#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>

#include "sound_player/AudioType.h"

#define FOUND 0.7   //toDo: Add launch file parameter and member variable
#define STEP 0.45
#define STEPS 7

PositionReader::PositionReader() :
_rate(NULL)
{
   ros::NodeHandle prvNh("~");
   double rateVar;
   std::string getModelSrvTopic;
   std::string audioTypeTopic;
   prvNh.param("robot_model_name", _robotModel, std::string("rescue_robot"));
   prvNh.param("victim_model_name", _victModel, std::string("vicbox"));
   prvNh.param("get_model_state_topic", getModelSrvTopic, std::string("/gazebo/get_model_state"));
   prvNh.param("audio_type_topic", audioTypeTopic, std::string("audio_type"));
   prvNh.param<double>("loop_rate", rateVar, 1.0);
   prvNh.param<int>("empty_cycles", _emptyCycles, 3);
   _rate = new ros::Rate(rateVar);

   _positionReaderClient = _nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
   _audioTypePub         = _nh.advertise<sound_player::AudioType>(audioTypeTopic, 1);
}

PositionReader::~PositionReader()
{
   delete _rate;
}

void PositionReader::run(void)
{
   gazebo_msgs::GetModelState robotReq;
   gazebo_msgs::GetModelState victimReq;

   robotReq.request.model_name  = _robotModel;
   victimReq.request.model_name = _victModel;
   bool found = false;
   int loopCtr = 0;
   while(ros::ok())
   {
//      if((loopCtr++) % _emptyCycles == 0)
//      {

         //request robotPose
         if(!_positionReaderClient.call(robotReq))
         {
            std::cout << __PRETTY_FUNCTION__ << " error calling robotPose service!\n";
            continue;
         }
         if(!_positionReaderClient.call(victimReq))
         {
            std::cout << __PRETTY_FUNCTION__ << " error calling victimPose service!\n";
            continue;
         }
         double dist = std::sqrt(
               (robotReq.response.pose.position.x - victimReq.response.pose.position.x) * (robotReq.response.pose.position.x - victimReq.response.pose.position.x) +
               (robotReq.response.pose.position.y - victimReq.response.pose.position.y) * (robotReq.response.pose.position.y - victimReq.response.pose.position.y)
         );
         std::cout << "\nPosition reader got robot position:\nx = " << robotReq.response.pose.position.x
               << "\ny = " << robotReq.response.pose.position.y << "\nz = "
               << robotReq.response.pose.position.z << "\n";
         std::cout << "\nPosition reader got victim position:\nx = " << victimReq.response.pose.position.x
               << "\ny = " << victimReq.response.pose.position.y << "\nz = "
               << victimReq.response.pose.position.z << "\n";
         std::cout << __PRETTY_FUNCTION__ << " distance = " << dist << " FOUND + STEP = " << FOUND + STEP << "\n";
         sound_player::AudioType audioType;
         if((loopCtr++) % _emptyCycles == 0)
         {
            if(found)
                 break;
            if(dist > FOUND)
            {
               audioType.type = audioType.TYPE_APPROACH;
               if((dist < (FOUND + STEP * 1)))// && (dist < (FOUND + STEP * 2)))
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
            {
               audioType.type = audioType.TYPE_FOUND;
               found = true;
            }

            _audioTypePub.publish(audioType);
         }
         std::cout << __PRETTY_FUNCTION__ << " to be published: factor = " << audioType.factor << "\n";
   //   }

      _rate->sleep();
   }
   std::cout << __PRETTY_FUNCTION__ << " found a victim! Goodbye!\n";
}
