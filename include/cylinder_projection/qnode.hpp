/**
 * @file /include/cylinder_projection/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef cylinder_projection_QNODE_HPP_
#define cylinder_projection_QNODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>


#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <sensor_msgs/PointCloud2.h>
#include "projector_calibration/calibration_utils.h"
#include "cylinder_projection/cylinder_processing.h"
#include "cylinder_projection/cylinder_utils.h"

/*****************************************************************************
 ** Namespaces
 ****************************************************HI*************************/

namespace cylinder_projection {

 /*****************************************************************************
  ** Class
  *****************************************************************************/

 class QNode : public QThread {
  Q_OBJECT
 public:
  QNode(int argc, char** argv );
  virtual ~QNode();
  bool init();
  bool init(const std::string &master_url, const std::string &host_url);
  void run();


  cv::Mat current_kinect_image;
  Cloud current_cloud;
  Cloud selected_cloud;
  Cylinder_Processing cylinder_processor;





  void imgCloudCB(const sensor_msgs::ImageConstPtr& img_ptr, const sensor_msgs::PointCloud2ConstPtr& cloud_ptr);

  /*********************
   ** Logging
   **********************/
  enum LogLevel {
   Debug,
   Info,
   Warn,
   Error,
   Fatal
  };

  QStringListModel* loggingModel() { return &logging_model; }
  void log( const LogLevel &level, const std::string &msg);

// public  Q_SLOTS:


 Q_SIGNALS:
 void loggingUpdated();
 void rosShutdown();

 void newKinectImage();
 void update_proj_image();

 private:
 int init_argc;
 char** init_argv;


 //	ros::Publisher chatter_publisher;
 QStringListModel logging_model;
 };

}  // namespace cylinder_projection

#endif /* cylinder_projection_QNODE_HPP_ */
