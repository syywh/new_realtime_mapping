#ifndef QSMAPPER_H
#define QSMAPPER_H

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>
#include <pcl/registration/ndt.h>

class QSMapper
{

    typedef pcl::PointXYZINormal xyzin;
    typedef pcl::PointXYZI xyzi;
    typedef pcl::PointXYZ xyz;

public:

    QSMapper(): a(0){}
    ~QSMapper() {}

    QSMapper(double _a): a(_a){}

    void addPointCloud(const sensor_msgs::PointCloud2& ros_pc, const std::vector<double>& ros_pc_pose) {

        pcl::fromROSMsg<xyzi>(ros_pc, cloud_in);
        std::cout << "pcl point cloud2 size " << cloud_in.points.size() << std::endl;

        Eigen::Quaternionf init_rotation (ros_pc_pose[6], ros_pc_pose[3], ros_pc_pose[4], ros_pc_pose[5]);
        Eigen::Translation3f init_translation (ros_pc_pose[0], ros_pc_pose[1], ros_pc_pose[2]);
        Eigen::Matrix4f pc_pose = (init_translation * init_rotation).matrix ();


        if (!initialized)
        {
            prev_cloud = cloud_in;
            prev_pose = pc_pose;
            prev_aligned_pose = pc_pose;
            initialized = true;
        }

        pcl::PointCloud<xyz>::Ptr input_cloud_ptr (new pcl::PointCloud<xyz>);
        pcl::PointCloud<xyz>::Ptr target_cloud_ptr (new pcl::PointCloud<xyz>);
        pcl::PointCloud<xyz>::Ptr output_cloud_ptr (new pcl::PointCloud<xyz>);

        pcl::copyPointCloud(cloud_in/**tmp_no_ground_ptr*//*+*tmp_ground_ptr*/, *input_cloud_ptr);
        pcl::copyPointCloud(prev_cloud/**pre_no_ground_ptr*//*+*pre_ground_ptr*/, *target_cloud_ptr);
        std::cout << "input point cloud2 size " << input_cloud_ptr->points.size() << std::endl;
        std::cout << "target point cloud2 size " << target_cloud_ptr->points.size() << std::endl;

        pcl::IterativeClosestPoint<xyz,xyz> matcher;
        matcher.setMaximumIterations (10); // 50
        matcher.setMaxCorrespondenceDistance (0.3); //3.0, 5.0
        matcher.setTransformationEpsilon(0.1);
        matcher.setEuclideanFitnessEpsilon(0.1);
        matcher.setInputTarget (target_cloud_ptr);
        matcher.setInputSource (input_cloud_ptr);

        Eigen::Matrix4f initial_relative_pose = prev_pose.inverse() * pc_pose;
        matcher.align (*output_cloud_ptr, initial_relative_pose/*.cast<float>()*//*(_initial_poses.empty())?Eigen::Matrix4f::Identity():(Eigen::Matrix4f)(_initial_poses[idx-_file_id_step].inverse()*_initial_poses[idx]).cast<float>()*/);

        std::cout << "- - - - - converged??? - - - - - " << matcher.hasConverged () << " - - - - score: " << matcher.getFitnessScore () << std::endl;

        Eigen::Matrix4f aligned_relative_pose = matcher.getFinalTransformation ();
        Eigen::Matrix4f aligned_pc_pose = prev_aligned_pose * aligned_relative_pose;

        pcl::transformPointCloud (cloud_in/*tmp_no_ground_ptr*/, cloud_out, aligned_pc_pose);

//        pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2;
//        pcl::PCLPointCloud2ConstPtr cloud2Ptr(cloud2);
////        pcl::PCLPointCloud2 cloud_filtered;
//
//        // Convert to PCL data type
//        pcl_conversions::toPCL(ros_pc, *cloud2);
//
//
//        pcl::PCLPointCloud2 pcl_pc2;
//        pcl_conversions::toPCL(ros_pc, pcl_pc2);
//
////        std::cout << "pcl point cloud2 size " << pcl_pc2.points.size() << std::endl;
//
//        pcl::MsgFieldMap field_map;
//        pcl::createMapping<pcl::PointXYZI> (ros_pc.fields, field_map);
//
//        pcl::PointCloud<pcl::PointXYZI> new_cloud;
////        pcl::fromPCLPointCloud2(cloud2, new_cloud);

        prev_cloud = cloud_in;
        prev_pose = pc_pose;
        prev_aligned_pose = aligned_pc_pose;

    }

    void getPointCloud(sensor_msgs::PointCloud2& ros_pc) {

        pcl::toROSMsg(cloud_out, ros_pc);

    }


private:

    pcl::PointCloud<xyzi> cloud_in;
    pcl::PointCloud<xyzi> cloud_out;

    bool initialized = false;
    pcl::PointCloud<xyzi> prev_cloud;
    Eigen::Matrix4f prev_pose;
    Eigen::Matrix4f prev_aligned_pose;

    double a;
};

#endif
