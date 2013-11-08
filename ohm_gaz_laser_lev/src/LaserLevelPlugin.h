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

#include <string>

namespace gazebo
{

class LaserLevelPlugin : public ModelPlugin
{
public:
  LaserLevelPlugin();
  virtual ~LaserLevelPlugin();
  virtual void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
  virtual void UpdateChild();
private:
  ros::NodeHandle* _nh;
  ros::Publisher _jointStatePub;
  std::string _nodeNamespace;
};

}
#endif /* LASERLEVELPLUGIN_H_ */
