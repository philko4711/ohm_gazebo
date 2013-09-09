/*
 * TwistSwitch.cpp
 *
 *  Created on: 09.09.2013
 *      Author: phil
 */

#include "TwistSwitch.h"

#include <geometry_msgs/Twist.h>

TwistSwitch::TwistSwitch()
{
   ros::NodeHandle private_nh("~");
   std::string twStSubsTopic;
   std::string twPubTopic;
   private_nh.param("stamped_subscriber", twStSubsTopic, std::string("vel/teleop"));
   private_nh.param("twist_publisher", twPubTopic, std::string("cmd_vel"));
   private_nh.param <double>("loop_rate", _loopRateData, 20.0);
   private_nh.param <double>("multiplicate", _multiplicate, 1.0);

   _twistStSubs = _nh.subscribe(twStSubsTopic, 10, &TwistSwitch::twistStCallback, this);
   _twistPub = _nh.advertise<geometry_msgs::Twist>(twPubTopic, 1);
   _loopRate = new ros::Rate(_loopRateData);
}

TwistSwitch::~TwistSwitch()
{
   delete _loopRate;
}

void TwistSwitch::run(void)
{
   while(ros::ok())
   {
      ros::spinOnce();
      _loopRate->sleep();
   }
   ros::shutdown();
}
void TwistSwitch::twistStCallback(const geometry_msgs::TwistStamped& twiststamped)
{
   geometry_msgs::Twist twist = twiststamped.twist;

   twist.angular.z *= _multiplicate;
   _twistPub.publish(twist);
}
