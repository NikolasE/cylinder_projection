/*
 * cylinder_utils.h
 *
 *  Created on: Jul 11, 2012
 *      Author: lengelhan
 */

#ifndef CYLINDER_UTILS_H_
#define CYLINDER_UTILS_H_


#include "projector_calibration/calibration_utils.h"


// assumes cylinder parallel to y-axis
// m_x, m_z: position of cylinder in x-z-plane
// out_y, out_phi: intersection point in cylinder coordinates
// returns true iff ray intersects with cylinder
bool intersectWithCylinder(float radius, float m_x, float m_z, cv::Point2f pixel,const cv::Mat &P, const cv::Mat projector_position, cv::Point3f& S, float& out_y, float& out_phi,float &hit_angle);



bool intersect(cv::Point3f proj_center, cv::Point3f point_on_ray, float m_x, float m_z, float R, cv::Point3f& S);
bool intersect(cv::Point3f proj_center, cv::Point3f point_on_ray, float R, cv::Point3f& S);

void test_intersection();


void drawLineImage(cv::Mat& img, int cols, int rows, int w_cnt, int h_cnt, int line_width);



#endif /* CYLINDER_UTILS_H_ */
