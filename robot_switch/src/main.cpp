/*
 * main.cpp
 *
 *  Created on: 02.09.2013
 *      Author: phil
 */

#include "Gui.h"
#include "RobotSwitch.h"

#include <QApplication>
#include <QDebug>

#include <string>
#include <sstream>
#include <vector>

int main(int argc, char** argv)
{
   argc = 2;
   argv[1] = "_image_transport:=compressed";
   ros::init(argc, argv, "robot_switch");
   std::string strVar;
   std::stringstream ss;
   std::vector<std::string> robots;
   int robNbr = 0;
   ros::NodeHandle nh("~");
   nh.param("robot_nbr", robNbr, int(5));
   for(int i = 0; i < robNbr; i++)
   {
      ss << "robot";
      ss << i;
      nh.param(ss.str(), strVar, std::string("robot1"));
      robots.push_back(strVar);
      ss.str("");
   }
   QApplication app(argc, argv);
   RobotSwitch robSwitch;
   Gui gui(&robSwitch);
   robSwitch.setGui(&gui);
   for(std::vector<std::string>::iterator iter = robots.begin(); iter != robots.end(); iter++)
      robSwitch.addRobot(*iter);
   gui.resize(900,600);
   gui.show();
   return(app.exec());
}


