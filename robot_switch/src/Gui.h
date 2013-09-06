/*
 * Gui.h
 *
 *  Created on: 28.08.2013
 *      Author: phil
 */

#ifndef GUI_H_
#define GUI_H_

#include <QWidget>
#include <QSpinBox>

#include "RobotSwitch.h"

class ImageView;
class QSpinBox;
class QHBoxLayout;
class QTimer;
class RobotSwitch;

class Gui: public QWidget
{
  Q_OBJECT
public:
   Gui(RobotSwitch* robSwitch, QWidget* parent = NULL);
   virtual ~Gui();
public slots:
   void robotChanged(int nbr)const{_robSwitch.setActive(nbr);}
   void iterate(void)const{_robSwitch.run();}
   void setImage(const unsigned char* image, const int height, const int width);
   void updateNbr(void){_robSelector->setMaximum(_robSwitch.getNbr());}
private:
   ImageView* _imgView;
   QSpinBox* _robSelector;
   QHBoxLayout* _hLayout;
   QTimer* _timer;
   RobotSwitch& _robSwitch;
};

#endif /* GUI_H_ */
