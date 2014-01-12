/*
 * PidTesterGui.h
 *
 *  Created on: 23.10.2013
 *      Author: phil
 */

#ifndef PIDTESTERGUI_H_
#define PIDTESTERGUI_H_

#include <QWidget>
#include <QObject>

#include <ros/ros.h>

#include "ui_pid_tester_gui.h"

class PidTesterGui: public QWidget
{
  Q_OBJECT
public:
  PidTesterGui();
  virtual ~PidTesterGui();
public slots:
  void dataChanged(void);
  void jumpPos(void);
  void jumpNeg(void);
private:
  Ui::PidTestUi* _pidTesterGui;
  ros::NodeHandle _nh;
  ros::ServiceClient _setCtrlParamService;
};

#endif /* PIDTESTERGUI_H_ */
