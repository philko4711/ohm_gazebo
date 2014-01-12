/*
 * LaserLevelPlugin.h
 *
 *  Created on: 08.11.2013
 *      Author: phil
 */

#ifndef LASERLEVELPLUGIN_H_
#define LASERLEVELPLUGIN_H_

#include <ros/ros.h>
#include <common/Plugin.hh>
#include <physics/PhysicsTypes.hh>
#include <sensors/SensorTypes.hh>
#include <transport/TransportTypes.hh>
#include <control_toolbox/pid.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>
#include "ohm_gaz_laser_lev/set_ctrl_param.h"

#include <string>

namespace gazebo
{

#define THRS 1.0
#define ROLL_VEL 0.1  //constant for testing of the roll joint
#define TILT_VEL 0.1  //constant for testing of the tilt joint
#define INTGR_MIN -50.0  //constants for testing of the pid controller (integral limits)
#define INTGR_MAX 50.0  //toDo:integrate into the PID Tester
#define TORQUE 20.0 //experimental magic number for the torque of the joints
#define P_VAL 14.8  //P value of the PID controller
#define I_VAL 17.8  //I value of the PID controller
#define D_VAL 0.0   //D value of the PID controller

class LaserLevelPlugin : public ModelPlugin
{
public:
  LaserLevelPlugin(void);
  virtual ~LaserLevelPlugin(void);
  virtual void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
  virtual void UpdateChild(void);
  bool setCtrlParamServCallBack(ohm_gaz_laser_lev::set_ctrl_param::Request& req, ohm_gaz_laser_lev::set_ctrl_param::Response& res);
private:
  void spin(void);
  ros::NodeHandle* _nh;
  ros::Publisher _jointStatePub;
  ros::ServiceServer _setPIDParamServ;
  ros::ServiceClient _positionReaderClient;
  std::string _nodeNamespace;
  physics::WorldPtr _world;
  physics::ModelPtr _model;
  physics::CollisionPtr _baseGeom;
  std::string _baseGeomName;
  event::ConnectionPtr _contactEvent;
  event::ConnectionPtr _updateConnection;
  //tf::TransformBroadcaster _transformBroadcaster;
  sensor_msgs::JointState _js;
  common::Time _prevUpdateTime;
  common::Time _lastCmdVelTime;

  std::string _rollJointName;
  std::string _tiltJointName;
  physics::JointPtr _rollJoint;
  physics::JointPtr _tiltJoint;
  bool _rollJointSet;
  bool _tiltJointSet;
  double _pVal;
  double _iVal;
  double _dVal;
  double _setRollAngle;
  double _setTiltAngle;
  float _torque;
  control_toolbox::Pid* _pidControler;
  ros::ServiceServer _setCtrlParamService;


  boost::thread* _spinnerThread;

};

}
#endif /* LASERLEVELPLUGIN_H_ */
