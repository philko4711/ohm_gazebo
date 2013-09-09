/*
 * TwistSwitch.h
 *
 *  Created on: 09.09.2013
 *      Author: phil
 */

#ifndef TWISTSWITCH_H_
#define TWISTSWITCH_H_

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

class TwistSwitch
{
public:
   TwistSwitch();
   virtual ~TwistSwitch();
   void twistStCallback(const geometry_msgs::TwistStamped& twiststamped);
   void run(void);
private:
   ros::NodeHandle _nh;
   ros::Subscriber _twistStSubs;
   ros::Publisher _twistPub;
   double _loopRateData;
   ros::Rate* _loopRate;
   double _multiplicate;
};

#endif /* TWISTSWITCH_H_ */
