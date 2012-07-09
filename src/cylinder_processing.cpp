/*
 * cylinder_processing.cpp
 *
 *  Created on: Jun 28, 2012
 *      Author: engelhan
 */

#include "cylinder_projection/cylinder_processing.h"


using namespace std;

void Cylinder_Processing::sendCloud(ros::Publisher& pub, Cloud& cloud){
 if (pub.getNumSubscribers() > 0){
  Cloud::Ptr msg = cloud.makeShared();

  msg->header.frame_id =  kinect_trafo_valid?"/world":"/openni_rgb_optical_frame";

  msg->header.stamp = ros::Time::now ();
  pub.publish(msg);
 }
}

bool Cylinder_Processing::publishCylinderMarker(const pcl::ModelCoefficients::Ptr& coeffs){
 if (pub_cylinder_marker.getNumSubscribers() == 0) {
  // ROS_INFO("no listener");
  return false;
 }

 visualization_msgs::Marker marker;

 marker.header.frame_id =  kinect_trafo_valid?"/world":"/openni_rgb_optical_frame";

 marker.header.stamp = ros::Time::now();
 marker.type = visualization_msgs::Marker::CYLINDER;

 marker.pose.position.x = coeffs->values[0];
 marker.pose.position.y = coeffs->values[1];
 marker.pose.position.z = coeffs->values[2];


 tf::Vector3 axis_vector(coeffs->values[3], coeffs->values[4], coeffs->values[5]);

 tf::Vector3 up_vector(0.0, 0.0, 1.0);
 tf::Vector3 right_vector = axis_vector.cross(up_vector);
 right_vector.normalized();
 tf::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
 q.normalize();
 geometry_msgs::Quaternion cylinder_orientation;
 tf::quaternionTFToMsg(q, cylinder_orientation);

 marker.pose.orientation = cylinder_orientation;
 // marker.pose.orientation.x = coeffs->values[3];
 // marker.pose.orientation.y = coeffs->values[4];
 // marker.pose.orientation.z = coeffs->values[5];
 // marker.pose.orientation.w = 1.0;


 marker.scale.x = 2*coeffs->values[6];
 marker.scale.y = 2*coeffs->values[6];
 marker.scale.z = 3.0;

 // Set the color -- be sure to set alpha to something non-zero!
 marker.color.r = 0.0f;
 marker.color.g = 1.0f;
 marker.color.b = 0.0f;
 marker.color.a = 1.0;

 marker.lifetime = ros::Duration();


 pub_cylinder_marker.publish(marker);
 // ROS_INFO("Publish marker");

 return true;

}



Cylinder_Processing::Cylinder_Processing(){
 proj_image = cv::Mat(768,1024, CV_8UC3);
 proj_image.setTo(0);
 coefficients_cylinder = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
}

void Cylinder_Processing::init(ros::NodeHandle& nh){
 pub_input   =  nh.advertise<Cloud>("/cylinder/input", 1);
 pub_sampled =  nh.advertise<Cloud>("/cylinder/input_sampled", 1);
 pub_inlier  =  nh.advertise<Cloud>("/cylinder/inlier", 1);
 pub_input_colored =  nh.advertise<Cloud>("/cylinder/inlier_colored", 1);
 pub_cylinder_marker =  nh.advertise<visualization_msgs::Marker>( "/cylinder/marker", 0 );


 // calibrator = new Projector_Calibrator();
}


bool Cylinder_Processing::readCalibration(){
 ros::param::param<string>("cylinder_projection/config_folder", config_folder_path,
   "/usr/gast/engelhan/ros/master_thesis/projector_calibration/data/");

 cout << "folder: " << config_folder_path << endl;

 char fn[100]; sprintf(fn, "%s/kinect_trafo.txt",config_folder_path.c_str());
 kinect_trafo_valid = loadAffineTrafo(kinect_trafo,fn);

 if (!kinect_trafo_valid)
  ROS_WARN("could not find kinect trafo");
 else
  ROS_WARN("Kinect trafo was loaded");


 proj_valid = loadMat(config_folder_path, "projection_matrix", proj_matrix);

 if (!proj_valid)
  ROS_WARN("Could not find projection matrix");

 return (proj_valid && kinect_trafo_valid);
}

bool Cylinder_Processing::setNewInputCloud(Cloud& cloud, std::stringstream& msg,  cv::Mat* mask){
 original = cloud;

 inlier_cloud.clear();

 if (cloud.size() == 0)
  return false;


 if (kinect_trafo_valid)
  pcl::getTransformedPointCloud(original, kinect_trafo, original);


 sendCloud(pub_input, original);

 pcl::PointCloud<pcl_Point>::Ptr cloud_filtered (new pcl::PointCloud<pcl_Point>);


 assert(original.isOrganized());
 pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

 pcl::IntegralImageNormalEstimation<pcl_Point, pcl::Normal> ne;
 ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
 ne.setMaxDepthChangeFactor(0.02f);
 ne.setNormalSmoothingSize(10.0f);
 ne.setInputCloud(original.makeShared());
 ne.compute(*normals);

 ROS_INFO("Computed %zu normals", normals->size());


 Cloud sampled;
 Cloud_n norm_sampled;

 sampleCloudWithNormals(original, *normals, sampled, norm_sampled, 2, mask);


 pcl::SACSegmentationFromNormals<pcl_Point, pcl::Normal> seg;

 seg.setOptimizeCoefficients (true);
 seg.setModelType (pcl::SACMODEL_CYLINDER);
 seg.setMethodType (pcl::SAC_LMEDS); // SAC_RANSAC
 seg.setNormalDistanceWeight (0.1);
 seg.setMaxIterations (1000);
 seg.setDistanceThreshold (0.05);
 seg.setRadiusLimits (0.1, 0.3);
 seg.setInputCloud (sampled.makeShared());
 seg.setInputNormals (norm_sampled.makeShared());


 pcl::PointIndices::Ptr  inliers_cylinder (new pcl::PointIndices);

 seg.segment (*inliers_cylinder, *coefficients_cylinder);
 std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;



 float angle = acos(abs(coefficients_cylinder->values[4]))/M_PI*180; // y-axis is gravitation!

 if (angle < 10)
  ROS_INFO("Cylinder has angle of %.1f deg to y-axis", angle);
 else
  ROS_WARN("Cylinder has angle of %.1f deg to y-axis", angle);

 if (angle > 20) return -1;

 if (inliers_cylinder->indices.size() == 0) return false;


 pcl::ExtractIndices<pcl_Point> extract;
 extract.setInputCloud (sampled.makeShared());
 extract.setIndices (inliers_cylinder);
 extract.setNegative (false);

 extract.filter (inlier_cloud);


 ROS_INFO("model has %zu inlier", inlier_cloud.size());


 if (inlier_cloud.size() < 0.5*sampled.size()){
  ROS_INFO("Cylinder fitting: Only %.1f%% inlier", inlier_cloud.size()*100.0/sampled.size());
 }


 msg << "Cylinder with " << inlier_cloud.size()*100.0/sampled.size() << "% inliers was found";


 publishCylinderMarker(coefficients_cylinder);

 sendCloud(pub_inlier,inlier_cloud);

 calculateProjectionArea();

 return true;
}

bool Cylinder_Processing::calculateProjectionArea(){

 if (inlier_cloud.size() == 0) return -1;

 // for now: cylinder is assumed to stand upright in y-direction

 y_min = angle_min = 1e5;
 y_max = angle_max = -1e5;

 for (uint i=0;i<inlier_cloud.size(); ++i){
  pcl_Point p= inlier_cloud[i];
  y_min = min(y_min, p.y);
  y_max = max(y_max, p.y);


  // point relative to axis!!

  float angle = atan2(-(p.z-getCylinderZ()),(p.x-getCylinderX()))/M_PI*180;

//  ROS_INFO("angle: %f", angle);

  cout << angle << endl;

  angle_min = min(angle_min, angle);
  angle_max = max(angle_max, angle);

 }

 // TODO: check if angles are in [250,10]...

 ROS_INFO("y: from %f to %f", y_min, y_max);
 ROS_INFO("yaw: from %f to %f", angle_min, angle_max);


 return true;

}


bool Cylinder_Processing::visualizeAngles(const cv::Mat& proj_matrix, cv::Mat& img){

 if (!proj_valid) return -1;

 cv::Mat p(4,1,CV_64FC1);
 cv::Mat px(3,1,CV_64FC1);
 img.setTo(0);

 // assumes min/max angles are set!
 for (uint i=0;i<inlier_cloud.size(); ++i){
  pcl_Point* p3 = &inlier_cloud[i];

  float angle = atan2(-(p3->z-getCylinderZ()),(p3->x-getCylinderX()))/M_PI*180;

  //  float c = (angle-angle_min)/(angle_max-angle_min)*180;
  //  cv::Scalar col(c,255,255);

  int c = floor((angle-angle_min)/(angle_max-angle_min)*5);
  cv::Scalar col;
  if (c % 2 )
   col =  cv::Scalar(255,255,255);
  else
   col =  cv::Scalar(0,0,0);

  if (angle < angle_min+1)
   col =  cv::Scalar(255,0,0);

  if (angle > angle_max-1)
     col =  cv::Scalar(0,0,255);




  p.at<double>(0) = p3->x;
  p.at<double>(1) = p3->y;
  p.at<double>(2) = p3->z;
  p.at<double>(3) = 1;

  px = proj_matrix*p;
  px /= px.at<double>(2);

  p3->r = col.val[0];
  p3->g = col.val[1];
  p3->b = col.val[2];

  cv::circle(img, cv::Point(px.at<double>(0),px.at<double>(1)), 2, col,-1);

 }

 sendCloud(pub_input_colored, inlier_cloud);


 //cv::cvtColor(img,img,CV_HSV2BGR);




 return true;

}



