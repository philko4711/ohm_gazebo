/*
 * pid_tester.cpp
 *
 *  Created on: 23.10.2013
 *      Author: phil
 */

#include "PidTesterGui.h"

#include <QApplication>

#include <ros/ros.h>

int main(int argc, char** argv)
{
	QApplication app(argc, argv);
	ros::init(argc, argv, "pid_test_node");
	PidTesterGui gui;
	gui.show();
	return(app.exec());
}


