/*
 * RobotSwitch.cpp
 *
 *  Created on: 28.08.2013
 *      Author: phil
 */

#include "RobotSwitch.h"

#include <string>

#include <QDebug>

#define NO_ACTIVE 999

RobotSwitch::RobotSwitch(Gui* const gui) :
            _gui(*gui)
{
   std::string strVar;

   ros::NodeHandle nh("~");

   // Parameters for launch file
   nh.param("input_cmd_vel", strVar, std::string("vel/tele_vel"));
   _twistSub = _nh.subscribe(strVar, 10, &RobotSwitch::cmdVelCallback, this);
   _nbr = 0;
   _active = NO_ACTIVE;
}


void RobotSwitch::addRobot(const std::string& name)
{
   ros::Publisher pubVar;
   ros::Subscriber subVar;
   std::string strVar = name;
   strVar += "cmd_vel";
   pubVar = _nh.advertise<geometry_msgs::TwistStamped>(strVar, 1);
   strVar.clear();
   strVar = name;
   name += "camera/camera/image_raw";
   //robot/camera/camera/image_raw
   subVar = _nh.subscribe(strVar,10 ,&RobotSwitch::imgCallback, this);
   _twistPubs.push_back(pubVar);
   _imgSubs.push_back(subVar);
   ++_nbr;
   if(_active == NO_ACTIVE)
      _active = 0;
}

void RobotSwitch::start(void)
{
   if(_nbr)
   {
      qDebug() << __PRETTY_FUNCTION__ << " No robots found! Exit!\n";
      return;
   }
   else
      this->run();
}

