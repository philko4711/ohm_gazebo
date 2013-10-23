/*
 * pid_tester.cpp
 *
 *  Created on: 23.10.2013
 *      Author: phil
 */

#include "PidTesterGui.h"

#include <QApplication>

int main(int argc, char** argv)
{
	QApplication app(argc, argv);
	PidTesterGui gui;
	gui.show();
	return(app.exec());
}


