/*
 * ImageView.h
 *
 *  Created on: 28.08.2013
 *      Author: phil
 */

#ifndef IMAGEVIEW_H_
#define IMAGEVIEW_H_

#include <QWidget>

class ImageView: public QWidget
{
public:
   ImageView(const unsigned char* image, const int height, const int width);
   ImageView(void);
   virtual ~ImageView(void){}
   void setHeight(const int height);
   void setWidth(const int width);
   void setImage(const unsigned char* image);
   int height(void)const{return _height;}
   int width(void)const{return _width;}
protected:
    void paintEvent(QPaintEvent* event);
private:
    const unsigned char* _image;
    int _height;
    int _width;
    unsigned char* _imgTmp;
    unsigned char* _imagee;
};

#endif /* IMAGEVIEW_H_ */
