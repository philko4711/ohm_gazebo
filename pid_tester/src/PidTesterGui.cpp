/*
 * PidTesterGui.cpp
 *
 *  Created on: 23.10.2013
 *      Author: phil
 */

#include "PidTesterGui.h"

#include <QDebug>

PidTesterGui::PidTesterGui() :
_pidTesterGui(new Ui::Form)
{
  _pidTesterGui->setupUi(this);
  connect(_pidTesterGui->pSlider, SIGNAL(valueChanged(int)), this, SLOT(dataChanged()));
  connect(_pidTesterGui->iSlider, SIGNAL(valueChanged(int)), this, SLOT(dataChanged()));
  connect(_pidTesterGui->dSlider, SIGNAL(valueChanged(int)), this, SLOT(dataChanged()));
  connect(_pidTesterGui->angleSlider, SIGNAL(valueChanged(int)), this, SLOT(dataChanged()));
}

PidTesterGui::~PidTesterGui()
{
  delete _pidTesterGui;
}

void PidTesterGui::dataChanged(void)
{
  qDebug() << __PRETTY_FUNCTION__ << "\n\tP = " << _pidTesterGui->pSlider->value() << "\n\tI = "
           << _pidTesterGui->iSlider->value() << "\n\tD = " << _pidTesterGui->dSlider->value()
           << "\n\tangle = " << _pidTesterGui->angleSlider->value() << "\n";
}
