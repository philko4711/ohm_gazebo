/*
 * soundTest.cpp
 *
 *  Created on: 19.09.2013
 *      Author: phil
 */

#include <iostream>

#include <ros/ros.h>

#include "sound_player/AudioType.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "audio_test");
  ros::NodeHandle nh;
  ros::Publisher typePub = nh.advertise<sound_player::AudioType>("audio_type", 1);
  sound_player::AudioType audioType;
  audioType.type = audioType.TYPE_INIT;
  typePub.publish(audioType);
  sleep(2);
  audioType.type = audioType.TYPE_APPROACH;
  for(int i = 7; i >= 0; i--)
  {
    audioType.factor = i;
    typePub.publish(audioType);
    sleep(2);
  }
  audioType.type = audioType.TYPE_FOUND;
  typePub.publish(audioType);
  sleep(2);
}


