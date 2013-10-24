/*
 * Copyright (c) 2012, Clearpath Robotics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Desc: Gazebo 1.x plugin for a Clearpath Robotics Husky A200
 * Adapted from the TurtleBot plugin
 * Author: Ryan Gariepy
 */ 

#include <boost/thread.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

//new stuff
#include <control_toolbox/pid.h>
#include <gazebo_msgs/GetModelState.h>
#include <husky_plugin/husky_plugin_exp.h>
#include <tf/tf.h>
//new stuff

#include <ros/time.h>

#define THRS 1.0
#define ROLL_VEL 0.1  //constant for testing of the roll joint
#define TILT_VEL 0.1  //constant for testing of the tilt joint
#define INTGR_MIN -50.0  //constants for testing of the pid controller (integral limits)
#define INTGR_MAX 50.0  //toDo:integrate into the PID Tester
using namespace gazebo;

enum {BL= 0, BR=1, FL=2, FR=3};

HuskyPlugin::HuskyPlugin() :
												_pVal(9.9),
												_iVal(0.0),
												_dVal(0.0),
												_setRollAngle(0.0),
												_setTiltAngle(0.0),
												_pidControler(new control_toolbox::Pid)
{
	this->spinner_thread_ = new boost::thread( boost::bind( &HuskyPlugin::spin, this) );

	wheel_speed_ = new float[2];
	wheel_speed_[BL] = 0.0;
	wheel_speed_[BR] = 0.0;
	wheel_speed_[FL] = 0.0;
	wheel_speed_[FR] = 0.0;

	set_joints_[0] = false;
	set_joints_[1] = false;
	set_joints_[2] = false;
	set_joints_[3] = false;

	//new stuff
	_rollJointSet = false;
	_tiltJointSet = false;
	//new stuff

	joints_[0].reset();
	joints_[1].reset();
	joints_[2].reset();
	joints_[3].reset();

	//new stuff
	_rollJoint.reset();
	_tiltJoint.reset();
	_pidControler->initPid(_pVal, _iVal, _dVal, INTGR_MAX, INTGR_MIN);
	//new stuff

}

HuskyPlugin::~HuskyPlugin()
{
	delete _pidControler;
	rosnode_->shutdown();
	this->spinner_thread_->join();
	delete this->spinner_thread_;
	delete [] wheel_speed_;
	delete rosnode_;
}

void HuskyPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
	this->model_ = _parent;
	this->world_ = this->model_->GetWorld();

	this->node_namespace_ = "";
	if (_sdf->HasElement("robotNamespace"))
		this->node_namespace_ = _sdf->GetElement("robotNamespace")->GetValueString() + "/";


	bl_joint_name_ = "left_back_wheel_joint";
	br_joint_name_ = "right_back_wheel_joint";
	fl_joint_name_ = "left_front_wheel_joint";
	fr_joint_name_ = "right_front_wheel_joint";

	//new stuff
	_rollJointName = "base_link_to_roll_tilt_base";
	_tiltJointName = "roll_tilt_base_to_laser_platform";//"base_link_to_laser_platform";
	//new stuff

	wheel_sep_ = 0.415;
	wheel_diam_ = 0.260;
	torque_ = 24;   //realer Wert 6 da für alle vier Räder

	base_geom_name_ = "base_link";
	base_geom_ = model_->GetChildCollision(base_geom_name_);

	//base_geom_->SetContactsEnabled(true);

	// Get the name of the parent model
	std::string modelName = _sdf->GetParent()->GetValueString("name");

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection = event::Events::ConnectWorldUpdateStart(
			boost::bind(&HuskyPlugin::UpdateChild, this));
	gzdbg << "plugin model name: " << modelName << "\n";

	if (!ros::isInitialized())
	{
		int argc = 0;
		char** argv = NULL;
		ros::init(argc, argv, "gazebo_husky", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
	}

	rosnode_ = new ros::NodeHandle( node_namespace_ );

	cmd_vel_sub_ = rosnode_->subscribe("cmd_vel", 1, &HuskyPlugin::OnCmdVel, this );

	odom_pub_ = rosnode_->advertise<nav_msgs::Odometry>("odom", 1);

	joint_state_pub_ = rosnode_->advertise<sensor_msgs::JointState>("joint_states", 1);

	//new stuff
	_setCtrlParamService = rosnode_->advertiseService("/husky_plugin/set_ctrl_param", &HuskyPlugin::setCtrlParamServCallBack, this);
	_positionReaderClient = rosnode_->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	//new stuff

	js_.name.push_back( bl_joint_name_ );
	js_.position.push_back(0);
	js_.velocity.push_back(0);
	js_.effort.push_back(0);

	js_.name.push_back( br_joint_name_ );
	js_.position.push_back(0);
	js_.velocity.push_back(0);
	js_.effort.push_back(0);

	js_.name.push_back( fl_joint_name_ );
	js_.position.push_back(0);
	js_.velocity.push_back(0);
	js_.effort.push_back(0);

	js_.name.push_back( fr_joint_name_ );
	js_.position.push_back(0);
	js_.velocity.push_back(0);
	js_.effort.push_back(0);

	/*  new Stuff */
	js_.name.push_back(_rollJointName);
	js_.position.push_back(0);
	js_.velocity.push_back(0);
	js_.effort.push_back(0);

	js_.name.push_back(_tiltJointName);
	js_.position.push_back(0);
	js_.velocity.push_back(0);
	js_.effort.push_back(0);
	/*  new Stuff */



	prev_update_time_ = 0;
	last_cmd_vel_time_ = 0;


	//TODO: fix this

	joints_[BL] = model_->GetJoint(bl_joint_name_);
	joints_[BR] = model_->GetJoint(br_joint_name_);
	joints_[FL] = model_->GetJoint(fl_joint_name_);
	joints_[FR] = model_->GetJoint(fr_joint_name_);

	/*  new Stuff */
	_rollJoint = model_->GetJoint(_rollJointName);
	_tiltJoint = model_->GetJoint(_tiltJointName);
	/*  new Stuff */

	if (joints_[BL]) set_joints_[BL] = true;
	if (joints_[BR]) set_joints_[BR] = true;
	if (joints_[FL]) set_joints_[FL] = true;
	if (joints_[FR]) set_joints_[FR] = true;

	/*  new Stuff */
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
	//  else
	//    std::exit(1);
	/*  new Stuff */

	//initialize time and odometry position
	prev_update_time_ = last_cmd_vel_time_ = this->world_->GetSimTime();
	odom_pose_[0] = 0.0;
	odom_pose_[1] = 0.0;
	odom_pose_[2] = 0.0;
}


void HuskyPlugin::UpdateChild()
{
	common::Time time_now = this->world_->GetSimTime();
	common::Time step_time = time_now - prev_update_time_;
	prev_update_time_ = time_now;
	ros::Duration timeStep(step_time.Double());

	double wd, ws, wheel_gauge;
	double d_bl, d_br, d_fl, d_fr;
	double dr, da;
	double _slip_correction;

	//new stuff
	double levelVelocity = 0.0;    //output off the pid controller that levels the laser
	//new stuff

	wd = wheel_diam_;
	ws = wheel_sep_;
	wheel_gauge = 0.280; // bei Centervolksbot.urdf.xacro (Volksbot2013)   für Volksbot.urdf.xacro (Volksbot2012) Achsabstand 0.3

	_slip_correction     = cos(atan2(wheel_gauge, ws));
	_slip_correction     = _slip_correction * _slip_correction;

	d_bl = d_br = d_fl = d_fr = 0;
	dr = da = 0;

	// Distance travelled by front wheels
	if (set_joints_[BL])
		d_bl = step_time.Double() * (wd / 2) * joints_[FL]->GetVelocity(0);  // alt BL
	if (set_joints_[BR])
		d_br = step_time.Double() * (wd / 2) * joints_[FR]->GetVelocity(0);  // alt BR
	if (set_joints_[FL])
		d_fl = step_time.Double() * (wd / 2) * joints_[FL]->GetVelocity(0);
	if (set_joints_[FR])
		d_fr = step_time.Double() * (wd / 2) * joints_[FR]->GetVelocity(0);


	dr = (d_bl + d_br + d_fl + d_fr) / 4;
	da = ((d_br+d_fr)/2 - (d_bl+d_fl)/2) / ws * _slip_correction * 0.981;  // 0.981 entspricht Korrekturwert

	// Compute odometric pose
	odom_pose_[0] += dr * cos( odom_pose_[2] );
	odom_pose_[1] += dr * sin( odom_pose_[2] );
	odom_pose_[2] += da;

	// Compute odometric instantaneous velocity
	odom_vel_[0] = dr / step_time.Double();
	odom_vel_[1] = 0.0;
	odom_vel_[2] = da / step_time.Double();

	if (set_joints_[BL])
	{
		joints_[BL]->SetVelocity( 0, wheel_speed_[FL] / (wd/2.0) );  // alt BL    da Volksbot-Räder per Seite mit Kette verbunden
		joints_[BL]->SetMaxForce( 0, torque_ );
	}
	if (set_joints_[BR])
	{
		joints_[BR]->SetVelocity( 0, wheel_speed_[FR] / (wd / 2.0) );  // alt BR    da Volksbot-Räder per Seite mit Kette verbunden
		joints_[BR]->SetMaxForce( 0, torque_ );
	}
	if (set_joints_[FL])
	{
		joints_[FL]->SetVelocity( 0, wheel_speed_[FL] / (wd / 2.0) );
		joints_[FL]->SetMaxForce( 0, torque_ );
	}
	if (set_joints_[FR])
	{
		joints_[FR]->SetVelocity( 0, wheel_speed_[FR] / (wd / 2.0) );
		joints_[FR]->SetMaxForce( 0, torque_ );
	}
	// new stuff
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
		//double effort = pid.updatePid(currentPosition() - position_desi_, time - last_time);
		levelVelocity = _pidControler->updatePid(_setRollAngle - _rollJoint->GetAngle(0).GetAsRadian(), timeStep);
		std::cout << __PRETTY_FUNCTION__ << " controller Data:\n\tDes.Val: "  << _setRollAngle << "\n\tCur. Val: " << _rollJoint->GetAngle(0).GetAsRadian()
						  << "\n\tControlling error: " << _setRollAngle - _rollJoint->GetAngle(0).GetAsRadian() << "\n\tdt= " << timeStep.sec << "\n";
		std::cout << __PRETTY_FUNCTION__ << " rollVelocity = " << levelVelocity << "\n";
		if(!isnan(levelVelocity))
		{
			if(levelVelocity > INTGR_MAX)
				levelVelocity = (-1.0) * INTGR_MAX;
			else if(levelVelocity < INTGR_MIN)
				levelVelocity = (-1.0) * INTGR_MIN;
			else
				levelVelocity *= (-1.0);
			_rollJoint->SetVelocity(0, levelVelocity);
			std::cout << __PRETTY_FUNCTION__ << " toControler = " << levelVelocity << "\n";
		}
		else
			_rollJoint->SetVelocity(0, 0.0);
		_rollJoint->SetMaxForce(0, torque_);
		//		angleVar =_rollJoint->GetAngle(0).GetAsRadian();
		//		if((angleVar > (M_PI / 2)) && _rollJoint->GetVelocity(0) > 0.0)
		//			_rollJoint->SetVelocity(0, (-1.0) * ROLL_VEL);
		//		else if((angleVar < (-1.0) * (M_PI / 2)) && _rollJoint->GetVelocity(0) < 0.0)
		//			_rollJoint->SetVelocity(0, ROLL_VEL);
		//				_rollJoint->SetMaxForce(0, torque_);
	}
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
			_tiltJoint->SetVelocity(0, 0.0);
		_tiltJoint->SetMaxForce(0, torque_);
		//		angleVar = _tiltJoint->GetAngle(0).GetAsRadian();
		//		if((angleVar > (M_PI / 2)) && _tiltJoint->GetVelocity(0) > 0.0)
		//			_tiltJoint->SetVelocity(0, (-1.0) * ROLL_VEL);
		//		else if((angleVar < (-1.0) * (M_PI / 2)) && _tiltJoint->GetVelocity(0) < 0.0)
		//			_tiltJoint->SetVelocity(0, ROLL_VEL);
		//		_tiltJoint->SetMaxForce(0, torque_);
	}
	// new stuff

	/* vom Guardian_controller (Surface_correction)

  double v_left_mps, v_right_mps;

  // Calculate its own velocities for realize the motor control
  v_left_mps = ((joints_[FL]->GetVelocity(0) + joints_[BL]->GetVelocity(0)) / 2.0) * (wd / 2.0);
  v_right_mps =((joints_[FR]->GetVelocity(0) + joints_[BR]->GetVelocity(0)) / 2.0) * (wd / 2.0);

  linearSpeedMps_ = (v_right_mps + v_left_mps) / 2.0;                      // m/s
  angularSpeedRads_ = (v_right_mps - v_left_mps) / wheel_sep_;    //rad/s

  // Current AX3500 controllers close this loop allowing (v,w) references.
  double epv=0.0;
  double epw=0.0;
  static double epvant =0.0;
  static double epwant =0.0;

  // Adjusted for soft indoor office soil
  double kpv=100.0; double kdv=0.0;
  double kpw=200.0;  double kdw=0.0;

  // State feedback error
  epv = vr - linearSpeedMps_;
  epw = va - angularSpeedRads_;

  // Compute state control actions
  double uv= kpv * epv + kdv * (epv - epvant);
  double uw= kpw * epw + kdw * (epw - epwant);
  epvant = epv;
  epwant = epw;

  // Inverse kinematics
  double dUl = uv - 0.5 * wheel_sep_ * uw;
  double dUr = uv + 0.5 * wheel_sep_ * uw;

if (set_joints_[BL])
{    joints_[BL]->SetVelocity(0, (vr  - va * (wheel_sep_) / 2) );// -10.0 * (joints_[BL]->GetVelocity(0) - dUl)
    joints_[BL]->SetMaxForce( 0, torque_ );
}
if (set_joints_[FL])
{    joints_[FL]->SetVelocity(0, (vr  + va * (wheel_sep_) / 2) );
    joints_[FL]->SetMaxForce( 0, torque_ );
}
if (set_joints_[BR])
{    joints_[BR]->SetVelocity(0, (vr  - va * (wheel_sep_) / 2) );
    joints_[BR]->SetMaxForce( 0, torque_ );
}
if (set_joints_[FR])
{    joints_[FR]->SetVelocity(0, (vr  + va * (wheel_sep_) / 2) );
    joints_[FR]->SetMaxForce( 0, torque_ );
}

	 */



	nav_msgs::Odometry odom;
	odom.header.stamp.sec = time_now.sec;
	odom.header.stamp.nsec = time_now.nsec;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_footprint";
	odom.pose.pose.position.x = odom_pose_[0];
	odom.pose.pose.position.y = odom_pose_[1];
	odom.pose.pose.position.z = 0;   // = 0;


	btQuaternion qt;
	qt.setEuler(0,0,odom_pose_[2]);

	odom.pose.pose.orientation.x = qt.getX();
	odom.pose.pose.orientation.y = qt.getY();
	odom.pose.pose.orientation.z = qt.getZ();
	odom.pose.pose.orientation.w = qt.getW();

	double pose_cov[36] = { 1e-3, 0, 0, 0, 0, 0,
			0, 1e-3, 0, 0, 0, 0,
			0, 0, 1e6, 0, 0, 0,
			0, 0, 0, 1e6, 0, 0,
			0, 0, 0, 0, 1e6, 0,
			0, 0, 0, 0, 0, 1e3};

	memcpy( &odom.pose.covariance[0], pose_cov, sizeof(double)*36 );
	memcpy( &odom.twist.covariance[0], pose_cov, sizeof(double)*36 );

	odom.twist.twist.linear.x = 0;
	odom.twist.twist.linear.y = 0;
	odom.twist.twist.linear.z = 0;

	odom.twist.twist.angular.x = 0;
	odom.twist.twist.angular.y = 0;
	odom.twist.twist.angular.z = 0;

	odom_pub_.publish( odom );

	js_.header.stamp.sec = time_now.sec;
	js_.header.stamp.nsec = time_now.nsec;

	if (this->set_joints_[BL])
	{
		js_.position[0] = joints_[BL]->GetAngle(0).GetAsRadian();
		js_.velocity[0] = joints_[BL]->GetVelocity(0);
	}

	if (this->set_joints_[BR])
	{
		js_.position[1] = joints_[BR]->GetAngle(0).GetAsRadian();
		js_.velocity[1] = joints_[BR]->GetVelocity(0);
	}

	if (this->set_joints_[FL])
	{
		js_.position[2] = joints_[FL]->GetAngle(0).GetAsRadian();
		js_.velocity[2] = joints_[FL]->GetVelocity(0);
	}

	if (this->set_joints_[FR])
	{
		js_.position[3] = joints_[FR]->GetAngle(0).GetAsRadian();
		js_.velocity[3] = joints_[FR]->GetVelocity(0);
	}

	//new stuff
	if (_rollJointSet)
	{
		js_.position[4] = _rollJoint->GetAngle(0).GetAsRadian();
		js_.velocity[4] = _rollJoint->GetVelocity(0);
	}
	if (_tiltJointSet)
	{
		js_.position[5] = _tiltJoint->GetAngle(0).GetAsRadian();
		js_.velocity[5] = _tiltJoint->GetVelocity(0);
	}
	//new stuff

	joint_state_pub_.publish( js_ );

}

void HuskyPlugin::OnCmdVel( const geometry_msgs::TwistConstPtr &msg)
{
	last_cmd_vel_time_ = this->world_->GetSimTime();
	double vr, va;

	vr = msg->linear.x;
	va = msg->angular.z;

	/* Begrenzung der cmd_vel - Joystick ist zwar in der gazebo_joystick node auf +-1.39 limitiert aber die move_base-Node könnte schneller drehen lassen...  -> Odometriefehler */
	/* Your englisch is also not the yellow of the egg!*/
	if (msg->angular.z >= THRS)//0.35)
	{
		va = THRS;
	}
	else if (msg->angular.z <= (-1.0) * THRS)//-0.35)
	{
		va = (-1.0) * THRS;//-0.35;
	}
	else
	{
		va = msg->angular.z;
	}

	//  wheel_speed_[BL] = vr - va * (wheel_sep_) / 2;
	//  wheel_speed_[BR] = vr + va * (wheel_sep_) / 2;
	wheel_speed_[FL] = vr - va * (wheel_sep_) / 2;
	wheel_speed_[FR] = vr + va * (wheel_sep_) / 2;
}

void HuskyPlugin::spin()
{
	while(ros::ok()) ros::spinOnce();
}

//new stuff
bool HuskyPlugin::setCtrlParamServCallBack(husky_description_new::set_ctrl_param::Request& req, husky_description_new::set_ctrl_param::Response& res)
{
	_pVal = req.pVal;
	_iVal = req.iVal;
	_dVal = req.dVal;
	_setRollAngle = req.rollAngle;
	_setTiltAngle = req.tiltAngle;
	_pidControler->setGains(_pVal, _iVal, _dVal, INTGR_MAX, INTGR_MIN);
	return(true);
}

GZ_REGISTER_MODEL_PLUGIN(HuskyPlugin);

