/**
 * @file /include/cylinder_projection/main_window.hpp
 *
 * @brief Qt based gui for cylinder_projection.
 *
 * @date November 2010
 **/
#ifndef cylinder_projection_MAIN_WINDOW_H
#define cylinder_projection_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

#include <qevent.h>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace cylinder_projection {

 class MouseHandler : public QObject{
   Q_OBJECT
  public:


   MouseHandler( QObject *parent = 0 ) : QObject( parent ) {
    up = cv::Point2i(-1,-1);
   }

   cv::Point2i down;
   cv::Point2i move;
   cv::Point2i up;

   bool area_marked(){return move.x > 0;}

   void reset(){
    up = move = cv::Point2i(-1,-1);
   }

   //  bool active;
   Q_SIGNALS:
   void redraw_image();


  protected:
   bool eventFilter( QObject *dist, QEvent *event )
   {
    //   if (!active) return false;

    if( event->type() == QMouseEvent::MouseButtonPress)
     {
     QMouseEvent *mouseEvent = static_cast<QMouseEvent*>( event );
     //    ROS_INFO("down at %i %i", mouseEvent->x(), mouseEvent->y());

     down = cv::Point2i(mouseEvent->x(), mouseEvent->y());
     move = down;
     up = cv::Point2i(-1,-1);

     Q_EMIT redraw_image();

     }

    if( event->type() == QMouseEvent::MouseButtonRelease)
     {
     //    ROS_INFO("release");

     QMouseEvent *mouseEvent = static_cast<QMouseEvent*>( event );
     up = cv::Point2i(mouseEvent->x(), mouseEvent->y());
     move = up;
     Q_EMIT redraw_image();
     }

    if( event->type() == QMouseEvent::MouseMove){
     //    ROS_INFO("move");
     QMouseEvent *mouseEvent = static_cast<QMouseEvent*>( event );
     move = cv::Point2i(mouseEvent->x(), mouseEvent->y());
     Q_EMIT redraw_image();
    }


    return false;
   }
  };


/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

	// label to show projector image on
	QLabel lb_img;

public Q_SLOTS:


    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView();
    void updateKinectImage();
    void select_area();
    void load_calibration();
    void update_projector_image();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;

	MouseHandler mouse_handler;

	static const float image_scale = 0.5;

};

}  // namespace cylinder_projection

#endif // cylinder_projection_MAIN_WINDOW_H
