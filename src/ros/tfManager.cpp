#include <sstream>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <vector>

#include "boost/filesystem.hpp"  

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/static_transform_broadcaster.h>


//#include "ros_architecture_tf2_wrapper/ros_architecture_tf2_wrapper.h"
#include <math.h>
#include "angles/angles.h"

int encoder_tf_seq = 0;
int odom_tf_seq = 0;

//std::auto_ptr<tf::TransformBroadcaster> br;
//tf2_ros::TransformBroadcaster br;

void handleLaserEncoder(const sensor_msgs::JointState& encoder)
{
    static tf2_ros::TransformBroadcaster br;

    //    std::cout << "handleLaserEncoder " << std::endl;
    if (encoder.position.size()>0)
    {
        double angle_deg = encoder.position[0];
        double angle_rad = -angle_deg*3.1415926535/180.0;

        std::cout << "laser encoder angle: " << angle_deg << std::endl;

        tf2::Quaternion q;
        q.setRPY(0.0, angle_rad, 0.0);

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();//.fromNSec(encoder.header.stamp.toNSec());
        transformStamped.header.seq = encoder_tf_seq++;
        transformStamped.header.frame_id = "laserscanner_mounting_point";
        transformStamped.child_frame_id = "laserscanner_origin";
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br.sendTransform(transformStamped);

//        tf::Transform laserscannerOriginToMountingPoint;
//        laserscannerOriginToMountingPoint.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
//        tf::Quaternion orientation;
//        orientation.setEuler(0, angle_rad, 0);
//        laserscannerOriginToMountingPoint.setRotation(orientation);
//
//        ros::Time rosTime;
//        rosTime.fromNSec(encoder.header.stamp.toNSec());
//
//        tf::TransformBroadcaster br;
//        br.sendTransform(tf::StampedTransform(laserscannerOriginToMountingPoint.inverse(), rosTime, "laserscanner_origin", "laserscanner_mounting_point"));
//
//        tf2_ros::StaticTransformBroadcaster br2;
//        br2.sendTransform(transformStamped);
    }
}

void handleLaserEncoder(const sensor_msgs::JointState::ConstPtr& msg)
{
    //    std::cout << "handleLaserEncoder ptr " << std::endl;
    if (msg)
    {
        handleLaserEncoder(*msg);
    }
}

void handleOdometry(const nav_msgs::Odometry& msg)
{
    static tf2_ros::TransformBroadcaster br_odom;

    //    std::cout << "handleOdometry " << std::endl;
    if (true)
    {
        double x = msg.pose.pose.position.x/100.0;
        double y = msg.pose.pose.position.y/100.0;
        double z = msg.pose.pose.position.z/100.0;
        double w = msg.pose.pose.orientation.w;
        double wx = msg.pose.pose.orientation.x;
        double wy = msg.pose.pose.orientation.y;
        double wz = msg.pose.pose.orientation.z;

        std::cout << "odometry: " << x << " " << y << " " << z << " " << w << " " << wx << " " << wy << " " << wz << std::endl;

        tf2::Quaternion q;
        q.setValue(wx,wy,wz,w);

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();//.fromNSec(msg.header.stamp.toNSec());
        transformStamped.header.seq = odom_tf_seq++;
        transformStamped.header.frame_id = "odometry_origin";
        transformStamped.child_frame_id = "laserscanner_mounting_point";
        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = z;
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br_odom.sendTransform(transformStamped);

        //        tf::Transform laserscannerOriginToMountingPoint;
        //        laserscannerOriginToMountingPoint.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        //        tf::Quaternion orientation;
        //        orientation.setEuler(0, angle_rad, 0);
        //        laserscannerOriginToMountingPoint.setRotation(orientation);
        //
        //        ros::Time rosTime;
        //        rosTime.fromNSec(encoder.header.stamp.toNSec());
        //
        //        tf::TransformBroadcaster br;
        //        br.sendTransform(tf::StampedTransform(laserscannerOriginToMountingPoint.inverse(), rosTime, "laserscanner_origin", "laserscanner_mounting_point"));
        //
        //        tf2_ros::StaticTransformBroadcaster br2;
        //        br2.sendTransform(transformStamped);
    }
}

void handleOdometry(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(msg)
    {
        handleOdometry(*msg);
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tfManager");
  ros::NodeHandle nh;

  //  static tf2_ros::TransformBroadcaster br;

  const std::string nodename(ros::this_node::getName());

  ros::Subscriber laser_encoder_sub;

  ros::Subscriber odometry_sub;

  laser_encoder_sub = nh.subscribe<sensor_msgs::JointState>("/sensor/encoder", 1, handleLaserEncoder);

  odometry_sub = nh.subscribe<nav_msgs::Odometry>("/sensor/odometry", 1, handleOdometry);

//  std::vector<std::string> frame_para_vec;
//  nh.param(nodename+"/frames", frame_para_vec, {""});
//
//  //  for (uint i = 0; i < frame_para_vec.size(); i++ )
//  //    std::cout << frame_para_vec[i] << std::endl;
//
//  double test;
//  nh.getParam(nodename+"/test", test);
//  std::cout << test << std::endl;


//    tf2_ros::StaticTransformBroadcaster br_static;
//
//    geometry_msgs::TransformStamped transformStamped;
//    tf2::Quaternion q;
//
//    transformStamped.header.frame_id = "/laserscanner_origin";
//    transformStamped.child_frame_id = "/laserscanner_mounting_point";
//    transformStamped.transform.translation.x = 0.0;
//    transformStamped.transform.translation.y = 0.0;
//    transformStamped.transform.translation.z = 5.0;
//    q.setRPY(1.0, 0.0, 0.0);
//    transformStamped.transform.rotation.x = q.x();
//    transformStamped.transform.rotation.y = q.y();
//    transformStamped.transform.rotation.z = q.z();
//    transformStamped.transform.rotation.w = q.w();
//    br_static.sendTransform(transformStamped);


  ////  tf2_ros::TransformBroadcaster br;
  //
  //  geometry_msgs::TransformStamped transformStamped;
  //  tf2::Quaternion q;
  //
  //  for (uint i = 0; i < frame_para_vec.size(); i++)
  //  {
  //    std::string frame_id, child_frame_id;
  //    float x, y, z, roll, pitch, yaw;
  //    std::stringstream istrstr;
  //    istrstr.str(frame_para_vec[i].c_str());
  //    istrstr >> frame_id >> child_frame_id >> x >> y >> z >> roll >> pitch >> yaw;
  //    std::cout << "double check: " << frame_id << " " <<  child_frame_id << " " << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << std::endl;
  //
  //    transformStamped.header.frame_id = frame_id;
  //    transformStamped.child_frame_id = child_frame_id;
  //    transformStamped.transform.translation.x = x;
  //    transformStamped.transform.translation.y = y;
  //    transformStamped.transform.translation.z = z;
  //    q.setRPY(roll, pitch, yaw);
  //    transformStamped.transform.rotation.x = q.x();
  //    transformStamped.transform.rotation.y = q.y();
  //    transformStamped.transform.rotation.z = q.z();
  //    transformStamped.transform.rotation.w = q.w();
  //    br_static.sendTransform(transformStamped);
  //  }

//  ros::NodeHandle node;
//  br.reset(new tf::TransformBroadcaster());
 // rt_sub = node.subscribe<ros_messages::sensor_rt3000>("/sensorik/rt3k/rt3000", 1, handlePositionHypothesis);
//  ros::Subscriber fix_sub = node.subscribe("sensorik/rt3k/rt3000", 10, handlePositionHypothesis);


  ros::Rate rate(10.0);
  while (nh.ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
;
