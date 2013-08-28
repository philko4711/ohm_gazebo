/*
 * Gui.h
 *
 *  Created on: 28.08.2013
 *      Author: phil
 */

#ifndef GUI_H_
#define GUI_H_

#include <QWidget>

class QImage;
class QSpinBox;
class QVBoxLayout;
class QTimer;
class RobotSwitch;

class Gui: public QWidget
{
public:
   Gui(const RobotSwitch* const robSwitch, QWidget* parent = NULL);
   virtual ~Gui();
public slots:
   void iterateSwitch(void);
   void robotChanged(int nbr);
private:
   QImage* _image;
   QSpinBox* _robSelector;
   QVBoxLayout* _vLayout;
   QTimer* _timer;
   RobotSwitch& _robSwitch;
};

#endif /* GUI_H_ */
