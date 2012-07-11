/*
 * cylinder_utils.cpp
 *
 *  Created on: Jul 11, 2012
 *      Author: Nikolas Engelhard
 */


#include "cylinder_projection/cylinder_utils.h"
using namespace std;


bool similar(cv::Point3f a, cv::Point3f b){
 float thres = 1e-3;
 return (abs(a.x-b.x) < thres && abs(a.y-b.y) < thres && abs(a.z-b.z) < thres);
}


void test_intersection(){

 cv::Point3f center(10,0,0);
 cv::Point3f point_on_ray(9,0,0);
 float m_x = 0;
 float m_z = 0;
 float R = 3;
 cv::Point3f S;

 assert(intersect(center, point_on_ray, m_x, m_z, R, S));
 assert(similar(S,cv::Point3f(3,0,0)));

 point_on_ray = cv::Point3f(11,0,0);
 assert(intersect(center, point_on_ray, m_x, m_z, R, S));
 assert(similar(S,cv::Point3f(3,0,0)));

 point_on_ray = cv::Point3f(10,1,0);
 assert(!intersect(center, point_on_ray, m_x, m_z, R, S));

 m_x = -1;
 point_on_ray = cv::Point3f(9,0,0);
 assert(intersect(center, point_on_ray, m_x, m_z, R, S));
 //cout << "S " << S << endl;
 assert(similar(S,cv::Point3f(2,0,0)));

 m_x = 1;
 point_on_ray = cv::Point3f(9,0,0);
 assert(intersect(center, point_on_ray, m_x, m_z, R, S));
 assert(similar(S,cv::Point3f(4,0,0)));


 // ray just touches the cylinder
 m_x = 0; m_z = R;
 point_on_ray = cv::Point3f(9,0,0);
 assert(intersect(center, point_on_ray, m_x, m_z, R, S));
 //cout << "S " << S << endl;
 assert(similar(S,cv::Point3f(0,0,0)));

 //ray just touches the cylinder, small y-component
 point_on_ray = cv::Point3f(9,1,0);
 assert(intersect(center, point_on_ray, m_x, m_z, R, S));
 //cout << "S " << S << endl;
 assert(similar(S,cv::Point3f(0,10,0)));


 point_on_ray = cv::Point3f(0,0,R);
 center = cv::Point3f(R,0,0);
 m_x = 0; m_z = R;
 assert(intersect(center, point_on_ray, m_x, m_z, R, S));
 //cout << "S " << S << endl;
 //ROS_INFO("exp: %f %f",sin(45/180.0*M_PI)*R,R-cos(45/180.0*M_PI)*R);
 assert(similar(S,cv::Point3f(sin(45/180.0*M_PI)*R,0,R-cos(45/180.0*M_PI)*R)));


ROS_INFO("Checked intersection code!");


}


void drawLineImage(cv::Mat& img, int cols, int rows, int w_cnt, int h_cnt, int line_width){

 img = cv::Mat(rows, cols, CV_8UC3);
 img.setTo(0);


 int dh = rows/(h_cnt);
 int pos_y = dh;
 cv::line(img, cv::Point(0,line_width/2), cv::Point(cols,line_width/2), cv::Scalar(255,0,0),line_width);
 cv::line(img, cv::Point(0,rows-line_width/2), cv::Point(cols,rows-line_width/2), cv::Scalar(255,0,0),line_width);

 for (int i=1; i< h_cnt; ++i, pos_y+=dh){
  cv::line(img, cv::Point(0,pos_y), cv::Point(cols,pos_y), cv::Scalar(255,255,255),line_width);
 }


 int dw = cols/(w_cnt);
 int pos_x = dw;
 cv::line(img, cv::Point(line_width/2,0), cv::Point(line_width/2,cols), cv::Scalar(255,0,0),line_width);
 cv::line(img, cv::Point(cols-line_width/2-5,0), cv::Point(cols-line_width/2-5,rows), cv::Scalar(255,0,0),line_width);

 for (int i=1; i< w_cnt; ++i, pos_x+=dw){
  cv::line(img, cv::Point(pos_x,0), cv::Point(pos_x, rows), cv::Scalar(255,255,255),line_width);
 }


}




bool intersect(cv::Point3f proj_center, cv::Point3f point_on_ray, float m_x, float m_z, float R, cv::Point3f& S){

 float dx = point_on_ray.x - proj_center.x;
 float dy = point_on_ray.y - proj_center.y;
 float dz = point_on_ray.z - proj_center.z;

 if (abs(dx) < 1e-3 && abs(dz) < 1e-3){
  return false;
 }

 // position relative to center of cylinder
 float x = proj_center.x-m_x;
 // float y = proj_center.y;
 float z = proj_center.z-m_z;

 //ROS_INFO("x,y,z: %f %f %f, dx,dy,dz: %f %f %f", x,y,z,dx,dy,dz);

 float A = (2*x*dx+2*z*dz)/(dx*dx+dz*dz);
 float B = (x*x+z*z-R*R)/(dx*dx+dz*dz);

 float rad = (A*A/4-B);

 //ROS_INFO("A: %f, B: %f", A, B);

 if (rad < 0)
  return false;

 //ROS_INFO("rad: %f", rad);

 float pre = -A/2;


 float alpha_1 = pre+sqrt(rad);
 float alpha_2 = pre-sqrt(rad);

 //ROS_INFO("alphas: %f %f", alpha_1, alpha_2);


 // chose point that is closer to the projector
 float alpha = abs(alpha_1)<abs(alpha_2)?alpha_1:alpha_2;


 //ROS_INFO("d: %f %f %f, alpha: %f", dx,dy,dz, alpha);
 //ROS_INFO("center: %f %f %f", proj_center.x,proj_center.y,proj_center.z);


  S = cv::Point3f(proj_center.x+alpha*dx,
                  proj_center.y+alpha*dy,
                  proj_center.z+alpha*dz);

  return true;

}


bool intersectWithCylinder(float radius, float m_x, float m_z, cv::Point2f pixel,const cv::Mat &P, const cv::Mat projector_position, cv::Point3f& S, float& out_y, float& out_phi){


 // get one point that projects on the given pixel:
 cv::Point3f point_on_ray;

 project3D(pixel, P, 1, point_on_ray);

 cv::Point3f center(projector_position.at<double>(0),projector_position.at<double>(1),projector_position.at<double>(2));
 bool intersects_with_cylinder = intersect(center, point_on_ray, m_x,m_z, radius, S);

 if (!intersects_with_cylinder) return false;

 out_y = S.y;
 out_phi = atan2(-(S.z-m_z), S.x-m_x)/M_PI*180;

 return true;
}

