/*
 * RobotSwitch.h
 *
 *  Created on: 28.08.2013
 *      Author: phil
 */

#ifndef ROBOTSWITCH_H_
#define ROBOTSWITCH_H_

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <vector>

class Gui;

class RobotSwitch
{
public:
   RobotSwitch(Gui* const gui = NULL);
   virtual ~RobotSwitch(void);
   void addRobot(const std::string& name);
   unsigned int setActive(unsigned int nbr){return(_nbr = nbr);}
   void start(void);
   unsigned int getNbr(void)const{return(_nbr);}
private:
   void run(void){ros::spin();}
   void cmdVelCallback(const geometry_msgs::TwistStamped& cmdVel);
   void imgCallback(const sensor_msgs::ImageConstPtr& image);
   ros::NodeHandle _nh;
   std::vector<ros::Publisher> _twistPubs;
   ros::Subscriber _twistSub;
   std::vector<ros::Subscriber> _imgSubs;
   unsigned int _nbr;
   unsigned int _active;
   Gui& _gui;

};

#endif /* ROBOTSWITCH_H_ */
