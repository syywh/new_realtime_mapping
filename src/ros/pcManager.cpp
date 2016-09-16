#include <sstream>
#include <iostream>
#include <fstream>
#include <dirent.h>

#include <QSMapper/include/QSMapper.h>

#include "boost/filesystem.hpp"  
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/LaserScan.h"

ros::Publisher points_pub;
ros::Publisher points2_pub;
bool reset;
tf::Transform frame_pose;
boost::shared_ptr<sensor_msgs::PointCloud> msg_pc_whole;
QSMapper qsmapper;

void writePCDascii(const std::string &file_name, const boost::shared_ptr<sensor_msgs::PointCloud> pc_msg_ptr, const int precision=8)
{

//  std::cout << file_name << std::endl;
  std::cout << "writePCDascii: step 1 " << std::endl;
  if (pc_msg_ptr->points.empty ())
  {
    ROS_WARN_THROTTLE(1, "Got empty point cloud");
    return;
  }

  std::cout << "writePCDascii: step 2 " << std::endl;
  if (pc_msg_ptr->channels.size()==0)
  {
    ROS_WARN_THROTTLE(1, "pc_msg_ptr->channels.size()==0");
    pc_msg_ptr->channels[0].values[0];
    return;
  }
  else if (pc_msg_ptr->channels[0].values.size()==0)
  {
    ROS_WARN_THROTTLE(1, "pc_msg_ptr->channels[0].values.size()==0");
    pc_msg_ptr->channels[0].values[0];
    return;
  }

  if (pc_msg_ptr->points.empty ())
  {
    ROS_WARN_THROTTLE(1, "Got empty point cloud");
    return;
  }

  std::ofstream fs;
  fs.open (file_name.c_str ());      // Open file

  if (!fs.is_open () || fs.fail ())
  {
    ROS_WARN_THROTTLE(1, "Could not open file for writing!");
    return;
  }

  auto pointCount = pc_msg_ptr->points.size();

  // Mandatory lock file
  // boost::interprocess::file_lock file_lock;
  // setLockingPermissions (file_name, file_lock);

  fs.precision (precision);
  fs.imbue (std::locale::classic ());

  // Write the header information
  fs << "# .PCD v0.7 - Point Cloud Data file format"
      << "\nVERSION 0.7"
      << "\nFIELDS x y z intensity"
      << "\nSIZE 4 4 4 4"
      << "\nTYPE F F F F"
      << "\nCOUNT 1 1 1 1"
      << "\nWIDTH " << pointCount
      << "\nHEIGHT " << 1
      << "\nPOINTS " << pointCount
      << "\nDATA ascii\n";

  std::ostringstream stream;
  stream.precision (precision);
  stream.imbue (std::locale::classic ());
  // Iterate through the points
  for (auto i = 0u; i < pointCount; ++i) {
        const auto& point = pc_msg_ptr->points[i];
        stream << boost::numeric_cast<float>(point.x)
            << " " << boost::numeric_cast<float>(point.y)
            << " " << boost::numeric_cast<float>(point.z)
            << " " << boost::numeric_cast<float>(pc_msg_ptr->channels[0].values[i]);
        // Copy the stream, trim it, and write it to disk
        std::string result = stream.str ();
        boost::trim (result);
        stream.str ("");
        fs << result << "\n";
  }

  fs.close ();              // Close file
  // resetLockingPermissions (file_name, file_lock);

  std::cout << "pcd file generated." << std::endl;

}

bool
  loadLaserLog(const std::string& file_name)
{

  using boost::filesystem::path;
  auto tempPath = path(file_name);//path(dir) / path(tile_description_filename);
  std::ifstream infile(tempPath.native(), std::ofstream::binary);

  std::cout << "Loading " << file_name.c_str() << std::endl;

  if (!infile.is_open()){
    std::cout << "cannot open LaserLog file!" << std::endl;
    return false;
  }

  int line_count = 0;
  std::string str;
  while (std::getline(infile, str))
  {
    std::cout << "LaserLog: text line length: " << str.length() << std::endl;
    if (!str.length()) continue;

    std::stringstream istrstr;
    istrstr.str(str.c_str());

    double pitch = 10.0 - 0.03*3.1415926535*((double)line_count)/180.0;

    tf::Transform B;
    B.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion rotation;
    rotation.setEuler(0.0, pitch, 0.0);
    B.setRotation(rotation);

    auto msg_pc = boost::make_shared<sensor_msgs::PointCloud>();
//    msg_out->points.reserve(pointCount);
//    msg_pc->header.stamp.fromNSec(uint64_t(std::chrono::nanoseconds{std::chrono::microseconds{meanTime}}.count()));
    msg_pc->header.stamp = ros::Time::now();
    msg_pc->header.frame_id = "laserscanner_mounting_point";
//    msg_out->channels.emplace_back();
//    auto& intensity = msg_out->channels.back();

    double timestamp;
    istrstr >> timestamp;
//    msg_pc->header.stamp.fromNSec(uint64_t(std::chrono::nanoseconds{std::chrono::microseconds{timestamp}}.count()));
    
    line_count++;
    int num_val = 0;
    for (int i = 0; i<361; i++)
    {
    	double value;
	istrstr >> value;
	std::cout << value << " ";
	num_val++;

	double angle_deg = 180.0 - 0.5 * ((double)i);
	double angle_rad = angle_deg*3.1415926535/180.0;
        double x = value*cos(angle_rad);
	double y = value*sin(angle_rad);

        tf::Vector3 tmp;
	tmp.setValue(x, y, 0.0);
        auto v = B(tmp);

        msg_pc->points.emplace_back(); // remove argument when compiling with mountingPositions
        auto& p_out = msg_pc->points.back();
        p_out.x = float(v.x()); p_out.y = float(v.y()); p_out.z = float(v.z());
    } 
    std::cout << std::endl;
    std::cout << num_val << " values received! " << std::endl;

    points_pub.publish(msg_pc);

    // Eigen::AngleAxisf init_rotation (clamp_angle((90-Heading+_imu_offset_yaw)*0.017453293), Eigen::Vector3f::UnitZ ());
    // Eigen::Translation3f init_translation (easting-offset_x, northing-offset_y, 0);
    // Eigen::Matrix4f gps_pose_mat = (init_translation * init_rotation).matrix ();
    // _gps_poses.emplace_back(gps_pose_mat.cast<double>());
    
    sleep(0.1);
  }

//  if (_gps_poses.empty())
//    return false;
//  else
    return true;
}


void
  handleLaserScan(const sensor_msgs::LaserScan& scan)
{

    std::cout << "received laser scan with " << scan.ranges.size() << " points" << std::endl;

    if (!msg_pc_whole)
    {
        msg_pc_whole = boost::make_shared<sensor_msgs::PointCloud>();
        msg_pc_whole->header.stamp.fromNSec(scan.header.stamp.toNSec());
        //        msg_pc->header.stamp = ros::Time::now();
        msg_pc_whole->header.frame_id = "laserscanner_mounting_point";
        msg_pc_whole->channels.emplace_back();
        //        auto& intensity_pc_whole = msg_pc_whole->channels.back();
        msg_pc_whole->channels.back().name = "intensity";

        reset = true;
    }

    typedef decltype(scan.header.stamp.toNSec()) timestamp_t;
    static timestamp_t last_timestamp_nsec;
    if (!last_timestamp_nsec)
    {
        last_timestamp_nsec = scan.header.stamp.toNSec();
    }

    bool isInconsistent = std::fabs(scan.header.stamp.toNSec()-last_timestamp_nsec) > 5e9;

    auto msg_pc = boost::make_shared<sensor_msgs::PointCloud>();
    msg_pc->points.reserve(scan.ranges.size());
    msg_pc->header.stamp = ros::Time::now();//.fromNSec(scan.header.stamp.toNSec());
    //    msg_pc->header.stamp = ros::Time::now();
    msg_pc->header.frame_id = "laserscanner_origin";


//        msg_pc->points.reserve(scan.ranges.size());
//        msg_pc->header.stamp.fromNSec(scan.header.stamp.toNSec());
//        //    msg_pc->header.stamp = ros::Time::now();
//        msg_pc->header.frame_id = "laserscanner_origin";

    ros::Time rosTime;
    rosTime.fromNSec(scan.header.stamp.toNSec());
    geometry_msgs::TransformStamped transformStamped;
    static tf2_ros::Buffer buffer;
    static tf2_ros::TransformListener tfListener(buffer);
    try
    {
//        buffer.waitForTransform("laserscanner_origin", "laserscanner_mounting_point", rosTime+ros::Duration(3.0), ros::Duration(3.0));
        transformStamped = buffer.lookupTransform("laserscanner_mounting_point", "laserscanner_origin", rosTime, ros::Duration(0.1));
    }
    catch (tf2::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    //    std::cout << transformStamped.transform << std::endl;

    tf::Transform B;
    B.setOrigin(tf::Vector3(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z));
    tf::Quaternion rotation;
    rotation.setValue(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
    B.setRotation(rotation);


    geometry_msgs::TransformStamped transformStamped2;
    static tf2_ros::Buffer buffer2;
    static tf2_ros::TransformListener tfListener2(buffer2);
    try
    {
        //        buffer.waitForTransform("laserscanner_origin", "laserscanner_mounting_point", rosTime+ros::Duration(3.0), ros::Duration(3.0));
        transformStamped2 = buffer2.lookupTransform("odometry_origin", "laserscanner_mounting_point", rosTime, ros::Duration(0.5));
    }
    catch (tf2::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    //    std::cout << transformStamped2.transform << std::endl;

    tf::Transform C;
    C.setOrigin(tf::Vector3(transformStamped2.transform.translation.x, transformStamped2.transform.translation.y, transformStamped2.transform.translation.z));
    tf::Quaternion rotation2;
    rotation2.setValue(transformStamped2.transform.rotation.x, transformStamped2.transform.rotation.y, transformStamped2.transform.rotation.z, transformStamped2.transform.rotation.w);
    C.setRotation(rotation2);

    if (reset)
    {
        frame_pose = C;
        reset = false;
    }

    tf::Transform D = C * B;
    D = frame_pose.inverseTimes(D);

    bool isValid=false;
    for (int i = 0; i<scan.ranges.size(); i++)
    {

        if (isnan(scan.ranges[i]) || scan.ranges[i]<=0)
            isValid = false;
        else
            isValid = true;

        //        double angle_deg = 180.0 - 0.5 * ((double)i);
        //        double angle_rad = angle_deg*3.1415926535/180.0;
        double angle_rad = scan.angle_min + (double(i))*scan.angle_increment;

        double x = scan.ranges[i]*cos(angle_rad);
        double y = scan.ranges[i]*sin(angle_rad);

        tf::Vector3 tmp;
        tmp.setValue(x, y, 0.0);
        auto v = D(tmp);

        msg_pc->points.emplace_back(); // remove argument when compiling with mountingPositions
        auto& p_out = msg_pc->points.back();
        p_out.x = x; p_out.y = y; p_out.z = 0.0;
        //        p_out.x = float(v.x()); p_out.y = float(v.y()); p_out.z = float(v.z());

//        if (isnan(v.x())||isnan(v.y())||isnan(v.z()))
//            isValid = false;
//        else
//            isValid = true;

        if (isValid)
        {
            msg_pc_whole->points.emplace_back(); // remove argument when compiling with mountingPositions
            auto& p_whole_out = msg_pc_whole->points.back();
            p_whole_out.x = float(v.x()); p_whole_out.y = float(v.y()); p_whole_out.z = float(v.z());
        }
//        std::cout << "received point: " << v.x() << " " << v.y() << " " << v.z() << std::endl;
    }

    bool hasIntensity = scan.ranges.size() == scan.intensities.size();
    if (hasIntensity)
    {
        msg_pc->channels.emplace_back();
        auto& intensity = msg_pc->channels.back();
        intensity.name = "intensity";

        for (int i = 0; i<scan.intensities.size(); i++)
        {
            tf::Vector3 tmp;
            tmp.setValue(1.0, 2.0, 0.0);
            auto v = B(tmp);
            if (isnan(v.x())||isnan(v.y())||isnan(v.z()))
                isValid = false;
            else
                isValid = true;

            intensity.values.emplace_back();
            auto& intensity_of_this_point = intensity.values.back();
            intensity_of_this_point = scan.intensities[i];

            if (isValid)
            {
                msg_pc_whole->channels.back().values.emplace_back();
                //            auto& intensity_of_this_point = intensity.values.back();
                msg_pc_whole->channels.back().values.back() = scan.intensities[i];
            }
        }
    }

    if (isInconsistent)
    {
        std::cout << "\n ===== \n ===== \n ===== \n ===== \n ===== \n ===== \n ===== \n ===== \n ===== \n ===== \n ===== \n ===== " << std::endl;
        std::cout << "total points: " << msg_pc_whole->points.size() << std::endl;
        //        msg_pc->points.reserve(scan.ranges.size());
        //        msg_pc_whole->header.stamp.fromNSec(last_timestamp_nsec);
        //    msg_pc->header.stamp = ros::Time::now();
                msg_pc_whole->header.frame_id = "odometry_origin";

//        writePCDascii("pcd.pcd", msg_pc_whole);
        auto msg_pc2 = boost::make_shared<sensor_msgs::PointCloud2>();
        sensor_msgs::convertPointCloudToPointCloud2(*msg_pc_whole, *msg_pc2);

        std::vector<double> pc_pose = {frame_pose.getOrigin()[0],
                                       frame_pose.getOrigin()[1],
                                       frame_pose.getOrigin()[2],
                                       frame_pose.getRotation().x(),
                                       frame_pose.getRotation().y(),
                                       frame_pose.getRotation().z(),
                                       frame_pose.getRotation().w()};

        qsmapper.addPointCloud(*msg_pc2, pc_pose);
        auto msg_pc2_new = boost::make_shared<sensor_msgs::PointCloud2>();
        qsmapper.getPointCloud(*msg_pc2_new);

        points2_pub.publish(msg_pc2_new);

        msg_pc_whole.reset();
        msg_pc_whole = boost::make_shared<sensor_msgs::PointCloud>();
        msg_pc_whole->header.stamp.fromNSec(scan.header.stamp.toNSec());
        //        msg_pc->header.stamp = ros::Time::now();
        msg_pc_whole->header.frame_id = "laserscanner_mounting_point";
        msg_pc_whole->channels.emplace_back();
        //        auto& intensity_pc_whole = msg_pc_whole->channels.back();
        msg_pc_whole->channels.back().name = "intensity";

        last_timestamp_nsec = scan.header.stamp.toNSec();

        reset = true;
        ros::Rate rate(1.0);
//        rate.sleep();
    }


    points_pub.publish(msg_pc);


}


void
  handleLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (msg)
    {
        handleLaserScan(*msg);
    }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcManager");

  ros::NodeHandle node;
  tf2_ros::Buffer buffer;

  ros::Subscriber laser_scan_sub;

  tf2_ros::TransformListener tfListener(buffer);

  points_pub = node.advertise<sensor_msgs::PointCloud>("points_out", 100);
  points2_pub = node.advertise<sensor_msgs::PointCloud2>("points2_out", 100);

//  loadLaserLog("/home/haf/catkin_ws/src/ros_learning/realtime_mapping/data/laserData.txt");
  laser_scan_sub = node.subscribe<sensor_msgs::LaserScan>("/sensor/LaserScan", 100, handleLaserScan);


  ros::Rate rate(10.0);
  while (node.ok())
  {
//      geometry_msgs::TransformStamped transformStamped;
//      try
//      {
//          transformStamped = buffer.lookupTransform("/laserscanner_origin", "/laserscanner_mounting_point", ros::Time(0));
//      }
//      catch (tf2::TransformException ex)
//      {
//          ROS_ERROR("%s", ex.what());
//      }
//
//      std::cout << transformStamped.transform << std::endl;

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
;
