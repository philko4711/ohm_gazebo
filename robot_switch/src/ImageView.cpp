/*
 * ImageView.cpp
 *
 *  Created on: 28.08.2013
 *      Author: phil
 */

#include "ImageView.h"

#include <QImage>
#include <QPainter>
#include <QDebug>

#include <cstring>

ImageView::ImageView(const unsigned char* image, const int height, const int width) :
                    _image(image),
                    _height(height),
                    _width(width)
{
  _imgTmp = new unsigned char[_width * _height * 3];
}

ImageView::ImageView(void) :
                    _image(NULL),
                    _height(480),
                    _width(640)
{
  _imgTmp = new unsigned char[_width * _height * 3];
  _imagee = new unsigned char[_width * _height * 3];
}

void ImageView::setHeight(const int height)
{
  _height = height;
  delete _imgTmp;
  _imgTmp = new unsigned char[_width * _height * 3];
  _imagee = new unsigned char[_width * _height * 3];
}
void ImageView::setWidth(const int width)
{
  _width = width;
  delete _imgTmp;
  _imgTmp = new unsigned char[_width * _height * 3];
  _imagee = new unsigned char[_width * _height * 3];
}

void ImageView::setImage(const unsigned char* image)
{
  std::memcpy(_imgTmp, image, _height * _width * 3 * sizeof(unsigned char));
}

void ImageView::paintEvent(QPaintEvent* event)
{
  if(0)//!_image)
  {
   qDebug() << __PRETTY_FUNCTION__ << " invalid image pointer!\n";
   return;
  }
  /*qDebug() << __PRETTY_FUNCTION__ << " predre!\n";
  unsigned char* imgVar = new unsigned char[_height * _width * 3];
  std::memcpy(imgVar, _image, _height * _width * 3 * sizeof(unsigned char));
  qDebug()  << __PRETTY_FUNCTION__ << " " << imgVar[0] << "\n";*/
  for(unsigned int i = 0; i < _width * _height * 3; i += 3)
  {
    //qDebug() << __PRETTY_FUNCTION__ << " i = " << i << "\n";
    _imagee[i]     = _imgTmp[i + 2];
    _imagee[i + 1] = _imgTmp[i + 1];
    _imagee[i + 2] = _imgTmp[i];
  }
  //qDebug() << __PRETTY_FUNCTION__ << " postdre!\n";
  QImage image(_imagee, _width, _height, QImage::Format_RGB888);
  QPainter painter(this);
  painter.drawImage(this->rect(),image, image.rect());
}

