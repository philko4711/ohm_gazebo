/*
 * SoundPlayer.cpp
 *
 *  Created on: 19.09.2013
 *      Author: phil
 */

#include "SoundPlayer.h"

SoundPlayer::SoundPlayer()
{
  ros::NodeHandle prvNh("~");
  std::string strVar;
  double dVar = 0.0;
  prvNh.param("audio_type_topic", strVar,  std::string("audio_type"));
  _typeSubs = _nh.subscribe(strVar, 10, &SoundPlayer::audioTypeCallback, this);
  prvNh.param("audio_play_topic", strVar,  std::string("robotsound"));
  _audioPub = _nh.advertise<sound_play::SoundRequest>(strVar, 1);
  prvNh.param("audio_path", _path,  std::string("/home/phil/workspace/ros/ohm_gazebo/sound_player/sounds/"));
  prvNh.param <double>("loop_rate",  dVar,  20.0);
  _loopRate = new ros::Rate(dVar);
  _soundRqst = new sound_play::SoundRequest;
  _soundRqst->sound   = _soundRqst->PLAY_FILE;
  _soundRqst->command = _soundRqst->PLAY_ONCE;
}

SoundPlayer::~SoundPlayer(void)
{
  delete _loopRate;
  delete _soundRqst;
}

void SoundPlayer::audioTypeCallback(const sound_player::AudioType& audioType)
{
  std::string audioPath = _path;
  if(audioType.type == audioType.TYPE_INIT)
    audioPath += "init.wav";
  else if(audioType.type == audioType.TYPE_APPROACH)
  {
    if(audioType.factor == 1)
      audioPath += "help_full.wav";
    else if(audioType.factor == 2)
      audioPath += "help_m2_5.wav";
    else if(audioType.factor == 3)
      audioPath += "help_m5.wav";
    else if(audioType.factor == 4)
      audioPath += "help_m10.wav";
    else if(audioType.factor == 5)
      audioPath += "help_m15.wav";
    else if(audioType.factor == 6)
      audioPath += "help_m20.wav";
    else if(audioType.factor == 7)
      audioPath += "help_m25.wav";
    else
    {
      std::cout << __PRETTY_FUNCTION__ << " ...this should not happen. Unknown factor!\n";
      return;
    }
  }
  else if(audioType.type == audioType.TYPE_FOUND)
    audioPath += "thanks_full.wav";
  else
  {
    std::cout << __PRETTY_FUNCTION__ << " ...this should not happen. Unknown type!\n";
    return;
  }
  std::cout << __PRETTY_FUNCTION__ << " path = "  << audioPath << "\n";
  _soundRqst->arg = audioPath;
  _audioPub.publish(*_soundRqst);
}
