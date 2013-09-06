/*
 * Gui.cpp
 *
 *  Created on: 28.08.2013
 *      Author: phil
 */

#include "Gui.h"


#include <QHBoxLayout>
#include <QTimer>
#include <QDebug>

#include "ImageView.h"

#define QT_TIMER 100   //10 hz

Gui::Gui(RobotSwitch* robSwitch, QWidget* parent) :
         QWidget(parent),
         _imgView(new ImageView()),
         _robSelector(new QSpinBox(this)),
         _hLayout(new QHBoxLayout(this)),
         _timer(new QTimer(this)),
         _robSwitch(*robSwitch)
{
  _hLayout->addWidget(_imgView);
   _robSelector->setPrefix("Robot: ");
   _robSelector->setMinimum(1);
   _robSelector->setMaximum(1);
   _robSelector->resize(200,200);
   connect(_robSelector, SIGNAL(valueChanged(int)), this, SLOT(robotChanged(int)));
   _hLayout->addWidget(_robSelector);
   connect(_timer, SIGNAL(timeout()), this, SLOT(iterate()));
   _timer->start(QT_TIMER);
   this->setToolTip("Wer das liest ist doof!");
}

Gui::~Gui()
{
  delete _imgView;
  delete _robSelector;
  delete _timer;
  delete _hLayout;
}

void Gui::setImage(const unsigned char* image, const int height, const int width)
{
  if(!_imgView)
  {
    _imgView = new ImageView(image, height, width);
    this->update();
    return;
  }
  if(height != _imgView->height())
    _imgView->setHeight(height);
  if(width != _imgView->width())
    _imgView->setWidth(width);
  _imgView->setImage(image);
  this->update();
}
