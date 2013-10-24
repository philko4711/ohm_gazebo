/*
 * position_reader.cpp
 *
 *  Created on: 16.09.2013
 *      Author: phil
 */

#include "PositionReader.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_reader");
  PositionReader positionReader;
  positionReader.start();
}
