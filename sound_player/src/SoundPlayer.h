/*
 * SoundPlayer.h
 *
 *  Created on: 19.09.2013
 *      Author: phil
 */

#ifndef SOUNDPLAYER_H_
#define SOUNDPLAYER_H_

#include <ros/ros.h>
#include "sound_player/AudioType.h"
#include "sound_play/SoundRequest.h"

#include <string>

class SoundPlayer
{
public:
   SoundPlayer(void);
   virtual ~SoundPlayer(void);
   void audioTypeCallback(const sound_player::AudioType& audioType);
   void start(void){this->run();}
private:
   void run(void){ros::spin();}
   ros::NodeHandle _nh;
   ros::Publisher _audioPub;
   ros::Subscriber _typeSubs;
   std::string _path;
   ros::Rate* _loopRate;
   sound_play::SoundRequest* _soundRqst;
};

#endif /* SOUNDPLAYER_H_ */
