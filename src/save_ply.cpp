#include "ros/init.h"
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <string>

#include <nlohmann/json.hpp>

using json = nlohmann::json;
//json j;
//std::ofstream o("/home/nate/Datasets/SemanticKittiPLY/08/poses.json");

std::atomic<int> idx(0);
int id = 0;

void odom_cb(const boost::shared_ptr<const nav_msgs::Odometry> &odom) {

  // {
  //   poses: [ {
  //     scan_id:
  //     x:
  //     y:
  //     z:
  //   }, ...]
  // }
  
 // json pose;
 // pose["scan_id"] = id;
 // pose["x"] = odom->pose.pose.position.x;
 // pose["y"] = odom->pose.pose.position.y;
 // pose["z"] = odom->pose.pose.position.z;


 // j["poses"].push_back(pose);

 // id++;
}

void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input) {
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

  // Now we want to save temp_cloud to a ply file
  std::string file_path =
      "/home/nate/Datasets/SemanticKittiPLY/08/" + std::to_string(idx) + ".ply";
  pcl::io::savePLYFile(file_path, *temp_cloud);
  std::cout << idx << std::endl;
  idx++;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/velodyne_points", 1000, cloud_cb);
  ros::spin();
  //ros::Subscriber sub_odom = n.subscribe("/odom_pose", 1000, odom_cb);
  //ros::Rate r(10);
  //while(id < 4071)
  //{
  //  ros::spinOnce();
  //  r.sleep();
  //}

  //o << std::setw(4) << j << std::endl;

  return 1;
}
