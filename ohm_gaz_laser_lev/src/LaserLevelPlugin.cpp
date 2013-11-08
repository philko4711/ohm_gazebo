/*
 * LaserLevelPlugin.cpp
 *
 *  Created on: 08.11.2013
 *      Author: phil
 */

#include <boost/thread.hpp>
#include "LaserLevelPlugin.h"
#include "physics/physics.h"
#include "physics/PhysicsTypes.hh"
#include "sensors/SensorTypes.hh"
#include "transport/TransportTypes.hh"
#include "common/Time.hh"
#include "common/Plugin.hh"
#include "common/Events.hh"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <control_toolbox/pid.h>
#include <boost/thread.hpp>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/tf.h>

using namespace gazebo;

LaserLevelPlugin::LaserLevelPlugin() :
                _pVal(9.9),
                _iVal(0.0),
                _dVal(0.0),
                _setRollAngle(0.0),
                _setTiltAngle(0.0),
                _torque(TORQUE),
                _pidControler(new control_toolbox::Pid)

{
  this->_spinnerThread = new boost::thread(boost::bind( &LaserLevelPlugin::spin, this));

  _rollJointSet = false;
  _tiltJointSet = false;
  _rollJoint.reset();
  _tiltJoint.reset();
  _pidControler->initPid(_pVal, _iVal, _dVal, INTGR_MAX, INTGR_MIN);
}

LaserLevelPlugin::~LaserLevelPlugin()
{
  _nh->shutdown();
  _spinnerThread->join();
  delete _spinnerThread;
  delete _nh;
}

void LaserLevelPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  _model = parent;
  _world = _model->GetWorld();
  _nodeNamespace = "";
  if (sdf->HasElement("robotNamespace"))
    _nodeNamespace = sdf->GetElement("robotNamespace")->GetValueString() + "/";

  //setup joint names
  _rollJointName = "base_link_to_roll_tilt_base";
  _tiltJointName = "roll_tilt_base_to_laser_platform";//"base_link_to_laser_platform";

  _baseGeomName = "base_link";
  _baseGeom     = _model->GetChildCollision(_baseGeomName);

  std::string modelName = sdf->GetParent()->GetValueString("name");

  _updateConnection = event::Events::ConnectWorldUpdateStart(boost::bind(&LaserLevelPlugin::UpdateChild, this));
  gzdbg << "plugin model name: " << modelName << "\n";

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_laser_level", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }
  _nh = new ros::NodeHandle(_nodeNamespace);
  _positionReaderClient = _nh->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  _jointStatePub = _nh->advertise<sensor_msgs::JointState>("joint_states", 1);
  _setCtrlParamService = _nh->advertiseService("/ohm_gaz_laser_lev/set_ctrl_param", &LaserLevelPlugin::setCtrlParamServCallBack, this);

  _js.name.push_back(_rollJointName);
  _js.position.push_back(0);
  _js.velocity.push_back(0);
  _js.effort.push_back(0);

  _js.name.push_back(_tiltJointName);
  _js.position.push_back(0);
  _js.velocity.push_back(0);
  _js.effort.push_back(0);

  _rollJoint = _model->GetJoint(_rollJointName);
  _tiltJoint = _model->GetJoint(_tiltJointName);

  if(_rollJoint)
  {
    _rollJointSet = true;
    _rollJoint->SetVelocity(0, 0.0);//ROLL_VEL);
  }
  if(_tiltJoint)
  {
    _tiltJointSet = true;
    _tiltJoint->SetVelocity(0, 0.0);//TILT_VEL);
  }

  _prevUpdateTime = 0;
  _lastCmdVelTime = 0;

  _prevUpdateTime = _world->GetSimTime();
  _lastCmdVelTime = _world->GetSimTime();
}

void LaserLevelPlugin::UpdateChild(void)
{
  common::Time timeNow = _world->GetSimTime();
  common::Time stepTime = timeNow - _prevUpdateTime;
  _prevUpdateTime = timeNow;
  ros::Duration timeStep(stepTime.Double());
  double levelVelocity = 0.0;    //output off the pid controller that levels the laser
  gazebo_msgs::GetModelState poseReq;
  poseReq.request.model_name = "rescue_robot";
  if(!_positionReaderClient.call(poseReq))
  {
    std::cout << __PRETTY_FUNCTION__ << " call of service failed!\n";
    _setRollAngle = _rollJoint->GetAngle(0).GetAsRadian();
    _setTiltAngle = _tiltJoint->GetAngle(0).GetAsRadian();
  }
  else
  {
    tf::Quaternion quat(poseReq.response.pose.orientation.x, poseReq.response.pose.orientation.y,
        poseReq.response.pose.orientation.z, poseReq.response.pose.orientation.w);
    tf::Matrix3x3 m(quat);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    m.getRPY(roll, pitch, yaw);
    _setRollAngle = (-1.0) * roll;
    _setTiltAngle = (-1.0) * pitch;
  }

  if(_rollJointSet)
  {
    levelVelocity = _pidControler->updatePid(_setRollAngle - _rollJoint->GetAngle(0).GetAsRadian(), timeStep);
    if(!isnan(levelVelocity))
    {
      if(levelVelocity > INTGR_MAX)
        levelVelocity = (-1.0) * INTGR_MAX;
      else if(levelVelocity < INTGR_MIN)
        levelVelocity = (-1.0) * INTGR_MIN;
      else
        levelVelocity *= (-1.0);
      _rollJoint->SetVelocity(0, levelVelocity);
      //std::cout << __PRETTY_FUNCTION__ << " toControler = " << levelVelocity << "\n";
    }
    else
    {
      _rollJoint->SetVelocity(0, 0.0);
    }
    _rollJoint->SetMaxForce(0, _torque);
  }
  else
    std::cout << __PRETTY_FUNCTION__ << " rolljoint not initialized!\n";
  if(_tiltJointSet)
  {
    levelVelocity = _pidControler->updatePid(_setTiltAngle - _tiltJoint->GetAngle(0).GetAsRadian(), timeStep);
    if(!isnan(levelVelocity))
    {
      if(levelVelocity > INTGR_MAX)
        levelVelocity = (-1.0) * INTGR_MAX;
      else if(levelVelocity < INTGR_MIN)
        levelVelocity = (-1.0) * INTGR_MIN;
      else
        levelVelocity *= (-1.0);
      _tiltJoint->SetVelocity(0, levelVelocity);
    }
    else
    {
      _tiltJoint->SetVelocity(0, 0.0);
    }
    _tiltJoint->SetMaxForce(0, _torque);
  }
  else
    std::cout << __PRETTY_FUNCTION__ << " tiltjoint not initialized!\n";


  _js.header.stamp.sec = timeNow.sec;
    _js.header.stamp.nsec = timeNow.nsec;
  if (_rollJointSet)
  {
    _js.position[4] = _rollJoint->GetAngle(0).GetAsRadian();
    _js.velocity[4] = _rollJoint->GetVelocity(0);
  }
  if (_tiltJointSet)
  {
    _js.position[5] = _tiltJoint->GetAngle(0).GetAsRadian();
    _js.velocity[5] = _tiltJoint->GetVelocity(0);
  }

  _jointStatePub.publish(_js);
}

void LaserLevelPlugin::spin()
{
  while(ros::ok())
    ros::spinOnce();
}

bool LaserLevelPlugin::setCtrlParamServCallBack(ohm_gaz_laser_lev::set_ctrl_param::Request& req, ohm_gaz_laser_lev::set_ctrl_param::Response& res)
{
  _pVal = req.pVal;
  _iVal = req.iVal;
  _dVal = req.dVal;
  _setRollAngle = req.rollAngle;
  _setTiltAngle = req.tiltAngle;
  _pidControler->setGains(_pVal, _iVal, _dVal, INTGR_MAX, INTGR_MIN);
  return(true);
}

GZ_REGISTER_MODEL_PLUGIN(LaserLevelPlugin);
