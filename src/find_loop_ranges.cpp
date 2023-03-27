#include "vtkIOStream.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/Point.h"
#include "ros/init.h"
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using json = nlohmann::json;

struct Loop {
  std::pair<int, int> q_range;
  std::pair<int, int> r_range;
};

double distance(double x1, double y1, double z1, double x2, double y2,
                double z2) {
  double d = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));
  return d;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "find_loop_ranges_marker_pub");
  ros::NodeHandle n;
  ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>(
      "visualization_marker_array", 1);

  std::string sequence = "08";

  std::ifstream f("/home/nate/Development/semnatickitteval/results/" +
                  sequence + "/poses.json");
  json data = json::parse(f);

  // {
  //   poses: [ {
  //     scan_id:
  //     x:
  //     y:
  //     z:
  //   }, ...]
  // }

  // loop through all the poses
  std::vector<json> poses = data["poses"];

  int query_idx = 0;
  int last_query_idx = 0;
  int idx = 0;

  // std::unordered_map<std::string, ReferenceMatch> result_map;
  // std::vector<std::pair<int, int>> loops;
  visualization_msgs::MarkerArray marker_array;

  std::vector<Loop> loop_vector;
  int last_idx = -1;
  while (query_idx < poses.size()) {
    int ref_idx = 0;

    double q_x = poses[query_idx]["x"];
    double q_y = poses[query_idx]["y"];
    double q_z = poses[query_idx]["z"];

    while (ref_idx < query_idx) {
      double r_x = poses[ref_idx]["x"];
      double r_y = poses[ref_idx]["y"];
      double r_z = poses[ref_idx]["z"];

      double dist = distance(q_x, q_y, q_z, r_x, r_y, r_z);
      if (dist < 5.0 && abs(ref_idx - query_idx) > 100) {
        if (query_idx == last_idx || query_idx == last_idx + 20 || query_idx == last_idx + 10) {
        } else {
          std::cout << "----------------------" << std::endl;
        }

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = idx;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = q_x;
        marker.pose.position.y = q_y;
        marker.pose.position.z = q_z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 6.0;
        marker.scale.y = 6.0;
        marker.scale.z = 6.0;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.8;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration(100);
        marker_array.markers.push_back(marker);
        idx++;

        std::cout << "query_id: " << query_idx << std::endl;
        std::cout << "ref_id: " << ref_idx << std::endl;
        last_idx = query_idx;
      }
      ref_idx += 10;
    }

    query_idx += 10;
  }

  ros::Rate r(1);

  while (ros::ok()) {
    while (vis_pub.getNumSubscribers() < 1) {
      if (!ros::ok()) {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    // vis_pub.publish(gt_marker_array);
    vis_pub.publish(marker_array);

    r.sleep();
  }

  return 1;
}
