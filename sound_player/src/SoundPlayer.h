/*
 * SoundPlayer.h
 *
 *  Created on: 19.09.2013
 *      Author: phil
 */

#ifndef SOUNDPLAYER_H_
#define SOUNDPLAYER_H_

#include <ros/ros.h>

class SoundPlayer
{
public:
   SoundPlayer();
   virtual ~SoundPlayer();
private:
   ros::NodeHandle _nh;
   ros::Publisher _audioPub;
   ros::Subscriber _typeSubs;
};

#endif /* SOUNDPLAYER_H_ */
