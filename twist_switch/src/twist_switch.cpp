/*
 * main.cpp
 *
 *  Created on: 09.09.2013
 *      Author: phil
 */

#include "TwistSwitch.h"

#include <ros/ros.h>

#include <string>




int main(int argc, char** argv)
{
   ros::init(argc, argv, "twist_switch");
   TwistSwitch twistSwitch;
   twistSwitch.run();
}

