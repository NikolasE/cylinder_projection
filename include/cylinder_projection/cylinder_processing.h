/*
 * cylinder_processsing.h
 *
 *  Created on: Jun 28, 2012
 *      Author: engelhan
 */

#ifndef CYLINDER_PROCESSSING_H_
#define CYLINDER_PROCESSSING_H_

#include "projector_calibration/calibration_utils.h"
#include "projector_calibration/projector_calibrator.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>


#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/integral_image_normal.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>


class Cylinder_Processing {

 Projector_Calibrator* calibrator;
 Cloud original;
 Cloud voxeled;



 ros::Publisher pub_input, pub_sampled, pub_inlier, pub_cylinder_marker;


 bool publishCylinderMarker(const pcl::ModelCoefficients::Ptr& coeffs);
 void sendCloud(ros::Publisher& pub, Cloud& cloud);


public:
 void init(ros::NodeHandle& nh);



 Cylinder_Processing();

 void setNewInputCloud(Cloud& cloud,cv::Mat* mask = NULL);








};


#endif /* CYLINDER_PROCESSSING_H_ */
