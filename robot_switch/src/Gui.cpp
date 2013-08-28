/*
 * Gui.cpp
 *
 *  Created on: 28.08.2013
 *      Author: phil
 */

#include "Gui.h"

#include <QImage>
#include <QSpinBox>
#include <QVBoxLayout>

#define QT_TIMER 100   //10 hz

Gui::Gui(const RobotSwitch* const robSwitch, QWidget* parent) :
         QWidget(parent),
         _image(NULL),
         _robSelector(new QSpinBox(this)),
         _vLayout(new QVBoxLayout(this)),
         _timer(new QTimer(this)),
         _robSwitch(robSwitch)
{
   _robSelector->setPrefix("X_dir: ");
   _robSelector->setMinimum(0);
   _robSelector->setMaximum(_robSwitch.getNbr());


}

Gui::~Gui() {
   // TODO Auto-generated destructor stub
}

void Gui::robotChanged(int nbr)
{
   _robSwitch.setActive(nbr);
}
