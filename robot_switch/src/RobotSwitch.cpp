/*
 * RobotSwitch.cpp
 *
 *  Created on: 28.08.2013
 *      Author: phil
 */

#include "RobotSwitch.h"

#include <string>

#include <QDebug>

#include <geometry_msgs/Twist.h>

#include "Gui.h"

#define NO_ACTIVE 999

RobotSwitch::RobotSwitch(Gui* const gui) :
            _gui(gui)
{
   std::string strVar;

   ros::NodeHandle nh("~");

   // Parameters for launch file
   nh.param("input_cmd_vel", strVar, std::string("vel/teleop"));
   _twistSub = _nh.subscribe(strVar, 10, &RobotSwitch::cmdVelCallback, this);
   _nbr = 0;
   _active = NO_ACTIVE;
}


void RobotSwitch::addRobot(const std::string& name)
{
   ros::Publisher pubVar;
   ros::Subscriber subVar;
   std::string strVar = name;
   strVar += "/cmd_vel";
   pubVar = _nh.advertise<geometry_msgs::Twist>(strVar, 1);
   strVar.clear();
   strVar = name;
   strVar += "/camera/camera/image_raw";
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

void RobotSwitch::cmdVelCallback(const geometry_msgs::TwistStamped& cmdVel)
{
  ros::Publisher pubVar = _twistPubs[_active];
  geometry_msgs::Twist msg = cmdVel.twist;
  pubVar.publish(msg);
}

void RobotSwitch::imgCallback(const sensor_msgs::ImageConstPtr& image)
{
  qDebug()  << __PRETTY_FUNCTION__ << " " << image->data.data()[0] << "\n";
  _gui->setImage(image->data.data(), image->height, image->width);
}

