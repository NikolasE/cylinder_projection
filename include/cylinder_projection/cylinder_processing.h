/*
 * cylinder_processsing.h
 *
 *  Created on: Jun 28, 2012
 *      Author: engelhan
 */

#ifndef CYLINDER_PROCESSSING_H_
#define CYLINDER_PROCESSSING_H_

#include "rgbd_utils/type_definitions.h"
#include "rgbd_utils/calibration_utils.h"
#include "projector_calibration/projector_calibrator.h"



#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/common/transform.h>

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
#include "projector_calibration/stat_eval.h"
#include "cylinder_projection/cylinder_utils.h"



struct Cylindric_projection_area {

 pcl::ModelCoefficients::Ptr coefficients_cylinder;
 Cloud inlier_cloud;

 // range of inliers in y-direction
 float y_min, y_max;
 // range in yaw around cylinder. (negative z-axis is zero)
 float angle_min, angle_max;


 float getCylinderX(){ return coefficients_cylinder->values[0];}
 float getCylinderY(){ return coefficients_cylinder->values[1];}
 float getCylinderZ(){ return coefficients_cylinder->values[2];}
 float getCylinderRadius(){ return coefficients_cylinder->values[6];}

 // moves the cylinder so that its centered around y-axis
 Eigen::Affine3f cylinder_trafo;
 Cloud inlier_centered; // inliers with cylinder_trafo applied


 Cylindric_projection_area(){
  coefficients_cylinder = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
 }

 bool calculateProjectionArea();


};




class Cylinder_Processing {


 Cloud original;
 Cloud voxeled;
 // Projector_Calibrator* calibrator;

 ros::Publisher pub_input, pub_input_colored, pub_sampled, pub_inlier, pub_cylinder_marker, pub_inlier_moved;


 bool publishCylinderMarker(const pcl::ModelCoefficients::Ptr& coeffs);

 bool kinect_trafo_valid;
 Eigen::Affine3f kinect_trafo;

 bool proj_valid;

 std::string config_folder_path;



 void computeProjectorPosition();
 cv::Mat projector_position;



public:

 bool sendCloud(ros::Publisher& pub, Cloud& cloud);
 ros::Publisher pub_mean_cloud;

 void init(ros::NodeHandle& nh);

 cv::Mat proj_matrix;

 // TODO: read size from file
 cv::Mat proj_image;

 Cylinder_Processing();

 bool setNewInputCloud(Cloud& cloud, std::stringstream& msg, Cylindric_projection_area& cylinder,  cv::Mat* mask = NULL);
 bool readCalibration();



// bool visualizeAngles(const cv::Mat& proj_matrix, cv::Mat& img);

 void forward_projection(Cylindric_projection_area& cylinder);



};


#endif /* CYLINDER_PROCESSSING_H_ */
