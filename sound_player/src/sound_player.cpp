/*
 * sound_player.cpp
 *
 *  Created on: 19.09.2013
 *      Author: phil
 */

#include "SoundPlayer.h"

#include <iostream>

int main(int argc, char** argv)
{
   ros::init(argc, argv, "sound_player");
   SoundPlayer soundPlayer;
   soundPlayer.start();
   /*ros::NodeHandle nh;
   ros::Publisher audioPub;
   sound_play::SoundRequest soundRqst;

   audioPub  = nh.advertise<sound_play::SoundRequest>("robotsound", 10, true);
   soundRqst.sound = soundRqst.PLAY_FILE;
   soundRqst.command = soundRqst.PLAY_ONCE;
   soundRqst.arg = "/home/phil/workspace/ros/ohm_gazebo/sound_player/sounds/help_full.wav";
   ros::Rate rate(1);
   while(ros::ok())
   {
      std::cout << __PRETTY_FUNCTION__ << "  Now publishing!\n";
      audioPub.publish(soundRqst);
      rate.sleep();
      sleep(2);
   }*/


}


