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
   ros::init(argc, argv, "robot_switch_node");
   std::string strVar;
   std::string robVar;
   std::stringstream ss;
   std::vector<std::string> robots;
   int robNbr = 0;
   ros::NodeHandle nh("~");
   nh.param<int>("robot_nbr", robNbr, 5);
   qDebug() << __PRETTY_FUNCTION__ << " number of robots = " << robNbr << "\n";
   for(int i = 0; i < robNbr; i++)
   {
      ss << "robot";
      ss << i;
      qDebug() << __PRETTY_FUNCTION__ << " LaunchFileParam: " << ss.str().c_str() << "\n";
      nh.param(ss.str(), strVar, std::string("robot1"));
      qDebug() << __PRETTY_FUNCTION__ << " ValueGainedFromLunch" << strVar.c_str() << "\n";
      robots.push_back(strVar);
      ss.str("");
   }
   QApplication app(argc, argv);
   RobotSwitch robSwitch;
   Gui gui(&robSwitch);
   robSwitch.setGui(&gui);
   for(std::vector<std::string>::iterator iter = robots.begin(); iter != robots.end(); iter++)
      robSwitch.addRobot(*iter);
 //  robSwitch.addRobot("robot2");
   gui.resize(600,600);
   gui.show();
   return(app.exec());
}


