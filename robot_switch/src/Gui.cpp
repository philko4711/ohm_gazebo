/*
 * Gui.cpp
 *
 *  Created on: 28.08.2013
 *      Author: phil
 */

#include "Gui.h"


#include <QVBoxLayout>
#include <QTimer>
#include <QDebug>

#include "ImageView.h"

#define QT_TIMER 100   //10 hz

Gui::Gui(RobotSwitch* robSwitch, QWidget* parent) :
         QWidget(parent),
         _imgView(new ImageView()),
         _robSelector(new QSpinBox(this)),
         _vLayout(new QVBoxLayout(this)),
         _timer(new QTimer(this)),
         _robSwitch(*robSwitch)
{
  _vLayout->addWidget(_imgView);
   _robSelector->setPrefix("Robot: ");
   _robSelector->setMinimum(1);
   _robSelector->setMaximum(_robSwitch.getNbr());
   connect(_robSelector, SIGNAL(valueChanged(int)), this, SLOT(robotChanged(int)));
   _vLayout->addWidget(_robSelector);
   connect(_timer, SIGNAL(timeout()), this, SLOT(iterate()));
   _timer->start(QT_TIMER);
}

Gui::~Gui()
{
  delete _imgView;
  delete _robSelector;
  delete _timer;
  delete _vLayout;
}

void Gui::robotChanged(int nbr)
{
   _robSwitch.setActive(nbr);
}

void Gui::iterate(void)
{
  _robSwitch.run();
}

void Gui::setImage(const unsigned char* image, const int height, const int width)
{
  qDebug() << __PRETTY_FUNCTION__ << "\n";
  if(!_imgView)
  {
    _imgView = new ImageView(image, height, width);
    this->update();
    return;
  }
  qDebug() << __PRETTY_FUNCTION__ << " width = " << width << " height = " << height << "\n";
  if(height != _imgView->height())
    _imgView->setHeight(height);
  if(width != _imgView->width())
    _imgView->setWidth(width);
  qDebug()  << __PRETTY_FUNCTION__ << " " << image[0] << "\n";
  _imgView->setImage(image);
  this->update();
}
