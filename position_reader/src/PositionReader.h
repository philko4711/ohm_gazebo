/*
 * PositionReader.h
 *
 *  Created on: 19.10.2013
 *      Author: phil
 */

#ifndef POSITIONREADER_H_
#define POSITIONREADER_H_

#include <ros/ros.h>
#include <string>

class PositionReader
{
public:
   PositionReader();
   virtual ~PositionReader();
   void start(void){this->run();}
private:
   void run(void);
   ros::NodeHandle _nh;
   ros::ServiceClient _positionReaderClient;// = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
   ros::Publisher _audioTypePub;
   std::string _robotModel;
   std::string _victModel;
   ros::Rate* _rate;
   int _emptyCycles;
};

#endif /* POSITIONREADER_H_ */
