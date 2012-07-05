/*
 * cylinder_processing.cpp
 *
 *  Created on: Jun 28, 2012
 *      Author: engelhan
 */

#include "cylinder_projection/cylinder_processing.h"


void Cylinder_Processing::sendCloud(ros::Publisher& pub, Cloud& cloud){
 if (pub.getNumSubscribers() > 0){
  Cloud::Ptr msg = cloud.makeShared();
  msg->header.frame_id = "/openni_rgb_optical_frame";
  msg->header.stamp = ros::Time::now ();
  pub.publish(msg);
 }
}

bool Cylinder_Processing::publishCylinderMarker(const pcl::ModelCoefficients::Ptr& coeffs){
 if (pub_cylinder_marker.getNumSubscribers() == 0) {

  ROS_INFO("no listener");
  return false;
 }

 visualization_msgs::Marker marker;
 marker.header.frame_id = "/openni_rgb_optical_frame";
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
 ROS_INFO("Publish marker");

 return true;

}



Cylinder_Processing::Cylinder_Processing(){

}

void Cylinder_Processing::init(ros::NodeHandle& nh){
 pub_input   =  nh.advertise<Cloud>("/cylinder/input", 1);
 pub_sampled =  nh.advertise<Cloud>("/cylinder/input_sampled", 1);
 pub_inlier  =  nh.advertise<Cloud>("/cylinder/inlier", 1);
 pub_cylinder_marker =  nh.advertise<visualization_msgs::Marker>( "/cylinder/marker", 0 );
}


void Cylinder_Processing::setNewInputCloud(Cloud& cloud, cv::Mat* mask){
 original = cloud;

 pcl::PointCloud<pcl_Point>::Ptr cloud_filtered (new pcl::PointCloud<pcl_Point>);

 // Cloud::ConstPtr ptr;
 //
 // pcl::VoxelGrid<pcl_Point> sor;
 // sor.setInputCloud(original.makeShared());
 // sor.setLeafSize (0.02f, 0.02f, 0.02f);
 // sor.filter(voxeled);

 // ROS_INFO("reduced from %zu to %zu pts", original.size(), voxeled.size());
 //
 // // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 // sendCloud(pub_input, cloud);
 // sendCloud(pub_sampled, voxeled);

 assert(cloud.isOrganized());
 pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

 pcl::IntegralImageNormalEstimation<pcl_Point, pcl::Normal> ne;
 ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
 ne.setMaxDepthChangeFactor(0.02f);
 ne.setNormalSmoothingSize(10.0f);
 ne.setInputCloud(cloud.makeShared());
 ne.compute(*normals);

 ROS_INFO("Computed %zu normals", normals->size());


 Cloud sampled;
 Cloud_n norm_sampled;

 sampleCloudWithNormals(cloud, *normals, sampled, norm_sampled, 3, mask);


 pcl::SACSegmentationFromNormals<pcl_Point, pcl::Normal> seg;

 seg.setOptimizeCoefficients (true);
 seg.setModelType (pcl::SACMODEL_CYLINDER);
 seg.setMethodType (pcl::SAC_LMEDS); // SAC_RANSAC
 seg.setNormalDistanceWeight (0.1);
 seg.setMaxIterations (1000);
 seg.setDistanceThreshold (0.03);
 seg.setRadiusLimits (0.1, 0.3);
 seg.setInputCloud (sampled.makeShared());
 seg.setInputNormals (norm_sampled.makeShared());






 ROS_INFO("looking for cylinders");
 // Obtain the cylinder inliers and coefficients
 pcl::PointIndices::Ptr  inliers_cylinder (new pcl::PointIndices);
 pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
 seg.segment (*inliers_cylinder, *coefficients_cylinder);
 std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
 ROS_INFO("found cylinders");

 Cloud inlier_cloud;
 pcl::ExtractIndices<pcl_Point> extract;
 extract.setInputCloud (sampled.makeShared());
 extract.setIndices (inliers_cylinder);
 extract.setNegative (false);

 extract.filter (inlier_cloud);


 ROS_INFO("model has %zu inlier", inlier_cloud.size());

 publishCylinderMarker(coefficients_cylinder);

 sendCloud(pub_inlier,inlier_cloud);
}
