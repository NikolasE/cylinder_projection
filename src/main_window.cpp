/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/cylinder_projection/main_window.hpp"

#include <opencv2/highgui/highgui.hpp>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace cylinder_projection {

 using namespace Qt;

 /*****************************************************************************
  ** Implementation [MainWindow]
  *****************************************************************************/

 MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
 : QMainWindow(parent)
 , qnode(argc,argv)
 {
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  setWindowIcon(QIcon(":/images/icon.png"));
  // QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  /*********************
   ** Logging
   **********************/
  //	ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
  QObject::connect(&qnode, SIGNAL(newKinectImage()), this, SLOT(updateKinectImage()));
  QObject::connect(&mouse_handler, SIGNAL(redraw_image()), this, SLOT(updateKinectImage()));
  ui.lb_kinectImage->installEventFilter(&mouse_handler);

 }

 MainWindow::~MainWindow() {}

 /*****************************************************************************
  ** Implementation [Slots]
  *****************************************************************************/






 /*****************************************************************************
  ** Implemenation [Slots][manually connected]
  *****************************************************************************/

 /**
  * This function is signalled by the underlying model. When the model changes,
  * this will drop the cursor down to the last line in the QListview to ensure
  * the user can always see the latest log message.
  */
 void MainWindow::updateLoggingView() {
  // ui.view_logging->scrollToBottom();
 }


 QImage Mat2QImage(const cv::Mat3b &src) {
  QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
  for (int y = 0; y < src.rows; ++y) {
   const cv::Vec3b *srcrow = src[y];
   QRgb *destrow = (QRgb*)dest.scanLine(y);
   for (int x = 0; x < src.cols; ++x) {
    destrow[x] = qRgba(srcrow[x][2], srcrow[x][1], srcrow[x][0], 255);
   }
  }
  return dest;
 }



 void MainWindow::updateKinectImage(){
  cv::Mat small_copy;
  cv::resize(qnode.current_kinect_image, small_copy, cv::Size(),image_scale,image_scale, CV_INTER_CUBIC);

  if (mouse_handler.area_marked())
   cv::rectangle(small_copy,  mouse_handler.down, mouse_handler.move, mouse_handler.area_marked()?CV_RGB(0,255,0):CV_RGB(255,0,0),2);

  QImage qimg = Mat2QImage(small_copy);
  QPixmap pixmap;
  ui.lb_kinectImage->setPixmap(pixmap.fromImage(qimg, 0));

 }


 void MainWindow::select_area(){

  if (!mouse_handler.area_marked()) return;
  if (qnode.current_cloud.size() == 0) return;

  // undo shrinking of image on GUI
  cv::Point l1(mouse_handler.down.x/image_scale,mouse_handler.down.y/image_scale);
  cv::Point l2(mouse_handler.up.x/image_scale,mouse_handler.up.y/image_scale);

//  ROS_INFO("l1: %i %i, l2: %i %i", l1.x, l1.y, l2.x,l2.y);

  cv::Mat mask(480,640,CV_8UC1);
  mask.setTo(0);
  cv::rectangle(mask, l1, l2,CV_RGB(255,255,255),-1);

  Cloud filtered;

  applyMaskOnCloud(mask, qnode.current_cloud, filtered);

  qnode.cylinder_processor.setNewInputCloud(qnode.current_cloud, &mask);

//  cv::namedWindow("mask");
//  cv::imshow("mask", mask);
//  cv::waitKey(0);
//  ROS_INFO("there are %zu points in the chosen area!", qnode.selected_cloud.size());

 }


 /*****************************************************************************
  ** Implementation [Menu]
  *****************************************************************************/





 void MainWindow::closeEvent(QCloseEvent *event)
 {
  QMainWindow::closeEvent(event);
 }

}  // namespace cylinder_projection

