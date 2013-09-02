/*
 * main.cpp
 *
 *  Created on: 02.09.2013
 *      Author: phil
 */

#include "Gui.h"
#include "RobotSwitch.h"

#include <QApplication>

int main(int argc, char** argv)
{
   argc = 2;
   argv[1] = "_image_transport:=compressed";
   ros::init(argc, argv, "robot_switch_node");
   QApplication app(argc, argv);
   RobotSwitch robSwitch;
   Gui gui(&robSwitch);
   robSwitch.setGui(&gui);
   robSwitch.addRobot("robot1");
   robSwitch.addRobot("robot2");
   gui.resize(600,600);
   gui.show();
   return(app.exec());
}


