/*
 * PidTesterGui.cpp
 *
 *  Created on: 23.10.2013
 *      Author: phil
 */

#include "PidTesterGui.h"
#include "ohm_gaz_laser_lev/set_ctrl_param.h"

#include <string>

#include <QDebug>

PidTesterGui::PidTesterGui() :
_pidTesterGui(new Ui::PidTestUi)
{
  _pidTesterGui->setupUi(this);
  connect(_pidTesterGui->pSlider, SIGNAL(valueChanged(int)), this, SLOT(dataChanged()));
  connect(_pidTesterGui->iSlider, SIGNAL(valueChanged(int)), this, SLOT(dataChanged()));
  connect(_pidTesterGui->dSlider, SIGNAL(valueChanged(int)), this, SLOT(dataChanged()));
  connect(_pidTesterGui->rollAngleSlider, SIGNAL(valueChanged(int)), this, SLOT(dataChanged()));
  connect(_pidTesterGui->tiltAngleSlider, SIGNAL(valueChanged(int)), this, SLOT(dataChanged()));
  ros::NodeHandle prvNh("~");
  std::string strVar;
  prvNh.param("set_ctrl_pram_topic", strVar, std::string("/ohm_gaz_laser_lev/set_ctrl_param"));
  _setCtrlParamService = _nh.serviceClient<ohm_gaz_laser_lev::set_ctrl_param>(strVar);
}

PidTesterGui::~PidTesterGui()
{
  delete _pidTesterGui;
}

void PidTesterGui::dataChanged(void)
{
//  qDebug() << __PRETTY_FUNCTION__ << "\n\tP = " << _pidTesterGui->pSlider->value() << "\n\tI = "
//           << _pidTesterGui->iSlider->value() << "\n\tD = " << _pidTesterGui->dSlider->value()
//           << "\n\tangle = " << _pidTesterGui->angleSlider->value() << "\n";
  ohm_gaz_laser_lev::set_ctrl_param servCall;
  servCall.request.pVal = static_cast<double>(_pidTesterGui->pSlider->value()) / 10.0;
  servCall.request.iVal = static_cast<double>(_pidTesterGui->iSlider->value()) / 10.0;
  servCall.request.dVal = static_cast<double>(_pidTesterGui->dSlider->value()) / 10.0;
  servCall.request.rollAngle = static_cast<double>(_pidTesterGui->rollAngleSlider->value()) / 100.0;
  servCall.request.tiltAngle = static_cast<double>(_pidTesterGui->tiltAngleSlider->value()) / 100.0;
  if(!_setCtrlParamService.call(servCall))
	  qDebug() << __PRETTY_FUNCTION__ << " call of setPidparam service failed!\n";
}
