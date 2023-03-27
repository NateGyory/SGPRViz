#include <cmath>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/Point.h"
#include "ros/init.h"
#include "ros/ros.h"
#include "vtkIOStream.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/program_options.hpp>

using json = nlohmann::json;

struct GTReferenceMatch {
  visualization_msgs::Marker marker;
  std::string ref_id;
};

struct PredReferenceMatch {
  std::string ref_id;
  double probability;
};

std::string sequence, result_file, sample;
int ref_start, ref_end, query_start, query_end, method, inc;

double distance(double x1, double y1, double z1, double x2, double y2,
                double z2) {
  double d = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));
  return d;
}

void GetGroundTruthMatches(std::map<std::string, GTReferenceMatch> &result_map,
                           visualization_msgs::MarkerArray &marker_array) {
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

  int idx = 0;

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
        int q_scan_id = poses[query_idx]["scan_id"];
        int r_scan_id = poses[ref_idx]["scan_id"];
        std::cout << "q_scan_id: " << q_scan_id << std::endl;
        std::cout << "r_scan_id: " << r_scan_id << std::endl;
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
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.lifetime = ros::Duration(100);
        marker_array.markers.push_back(marker);
        idx++;

        GTReferenceMatch rm;
        rm.ref_id = std::to_string(r_scan_id);
        rm.marker = marker;
        std::string key = std::to_string(q_scan_id) + ".ply";
        result_map[key] = rm;
      }
      ref_idx++;
    }

    query_idx++;
  }
}

void GetPredictionMap(
    double threshold, std::map<std::string, GTReferenceMatch> &gt_result_map,
    std::unordered_map<std::string, PredReferenceMatch> &pred_map,
    visualization_msgs::MarkerArray &marker_array) {

  std::ifstream res_f("/home/nate/Development/catkin_ws/src/sgpr_ros/results/"
                      "SemanticKitti/" +
                      sequence + "/" + sample + "/" + result_file);
  json res_data = json::parse(res_f);

  int tp = 0;
  int count = 0;
  for (auto &comparison : res_data["comparisons"]) {
    if (comparison["probability"] > threshold) {
      std::string key = comparison["query_id"];
      std::string ref_id = comparison["ref_id"];
      double prob = comparison["probability"];

      PredReferenceMatch match;
      match.ref_id = ref_id;
      match.probability = prob;
      pred_map[key] = match;
      std::cout << "query key: " << comparison["query_id"] << std::endl;
      std::cout << "SGPR prediction ref: " << comparison["ref_id"] << std::endl;
      // std::cout << "Ground truth ref: " << result_map[key].ref_id <<
      // std::endl;
      std::cout << "---------------------" << std::endl;

      if (gt_result_map.find(key) != gt_result_map.end()) {
        std::cout << "Ground truth ref: " << gt_result_map[key].ref_id
                  << std::endl;

        // Add the marker to the pred_marker array
        visualization_msgs::Marker marker_copy = gt_result_map[key].marker;

        marker_copy.color.r = 0.0;
        marker_copy.color.b = 0.0;
        marker_copy.color.g = 1.0;
        marker_copy.scale.x = 7.0;
        marker_copy.scale.y = 7.0;
        marker_copy.scale.z = 7.0;
        marker_array.markers.push_back(marker_copy);

        std::string ref_string = comparison["ref_id"];
        std::string extension = ".ply";

        ref_string.erase(ref_string.size() - extension.size());
        int ref_int = std::stoi(ref_string);
        int gt_int = std::stoi(gt_result_map[key].ref_id);
        int diff = abs(ref_int - gt_int);
        if (diff < 60) {
          std::cout << "Success: " << ref_int << std::endl;
          tp++;
        }
        // Save this marker to a marker array
      }
      count++;
    }
  }

  std::cout << "TP: " << tp << std::endl;
  std::cout << "Total Objs: " << count << std::endl;
}

void SavePR() {
  // Generate the pred map

  // {
  //    query_id : {
  //      ref_id : probability
  //    }
  // }
  //
  std::unordered_map<int, std::unordered_map<int, double>> q_r_prob_map;
  // {
  //    comparisons: [ {
  //      probability:
  //      query_id:
  //      ref_id:
  //    }]
  // }
  std::ifstream pred_f("/home/nate/Development/catkin_ws/src/sgpr_ros/results/"
                       "SemanticKitti/" +
                       sequence + "/" + sample + "/" + result_file);
  json pred_data = json::parse(pred_f);

  std::vector<json> comparisons = pred_data["comparisons"];
  for (auto &comp : comparisons) {
    std::string r_string = comp["ref_id"];
    std::string q_string = comp["query_id"];
    double prob = comp["probability"];
    std::string extension = ".ply";

    r_string.erase(r_string.size() - extension.size());
    q_string.erase(q_string.size() - extension.size());
    int r_int = std::stoi(r_string);
    int q_int = std::stoi(q_string);

    q_r_prob_map[q_int][r_int] = prob;
  }

  // Loop through the GT values
  std::ifstream f("/home/nate/Development/semnatickitteval/results/" +
                  sequence + "/poses.json");
  json poses_j = json::parse(f);

  // {
  //   poses: [ {
  //     scan_id:
  //     x:
  //     y:
  //     z:
  //   }, ...]
  // }

  // loop through all the poses
  std::vector<json> poses = poses_j["poses"];
  std::vector<json> truth_list;
  std::vector<json> pred_list;
  std::vector<json> pose_prob_list;

  int query_idx = query_start;
  int tp_count = 0;

  while (query_idx <= query_end) {
    int ref_idx = ref_start;

    double q_x = poses[query_idx]["x"];
    double q_y = poses[query_idx]["y"];
    double q_z = poses[query_idx]["z"];

    double true_prob = 0;
    double false_prob = 0;
    int q_has_lc = 0;

    while (ref_idx <= ref_end) {
      double r_x = poses[ref_idx]["x"];
      double r_y = poses[ref_idx]["y"];
      double r_z = poses[ref_idx]["z"];

      double dist = distance(q_x, q_y, q_z, r_x, r_y, r_z);
      int q_scan_id = poses[query_idx]["scan_id"];
      int r_scan_id = poses[ref_idx]["scan_id"];
      // TODO need to check if the q_scan_id and r_scan_id exist in the
      // q_r_prob_map
      bool key_exists = q_r_prob_map.find(q_scan_id) != q_r_prob_map.end() &&
                        q_r_prob_map[q_scan_id].find(r_scan_id) !=
                            q_r_prob_map[q_scan_id].end();
      if (key_exists) {

        //std::cout << "Dist: " << dist << std::endl;
        if (dist < 5.0 && abs(ref_idx - query_idx) > 100) {
          if (method) {
            q_has_lc = 1;
            true_prob = q_r_prob_map[q_scan_id][r_scan_id] > true_prob
                            ? q_r_prob_map[q_scan_id][r_scan_id]
                            : true_prob;
          } else {
            truth_list.push_back(1);
            pred_list.push_back(q_r_prob_map[q_scan_id][r_scan_id]);
          }
          

          // tp_count++;
        } else {
          if(method) {
            false_prob = q_r_prob_map[q_scan_id][r_scan_id] > false_prob
                             ? q_r_prob_map[q_scan_id][r_scan_id]
                             : false_prob;
          } else {
            truth_list.push_back(0);
            pred_list.push_back(q_r_prob_map[q_scan_id][r_scan_id]);
          }
          
        }
      }
      ref_idx+=inc;
    }
    if(method) {
      std::cout << "Truth: " << q_has_lc << std::endl;
      truth_list.push_back(q_has_lc);
      if (q_has_lc) {
        json pose_prob;
        pose_prob["query_id"] = query_idx;
        pose_prob["probability"] = true_prob;
        pose_prob_list.push_back(pose_prob);
        pred_list.push_back(true_prob);
        std::cout << "Highest Prob: " << true_prob << std::endl;
      } else {
        json pose_prob;
        pose_prob["query_id"] = query_idx;
        pose_prob["probability"] = false_prob;
        pose_prob_list.push_back(pose_prob);
        pred_list.push_back(false_prob);
        std::cout << "Highest Prob: " << false_prob << std::endl;
      }
    }
    query_idx+=inc;
  }

  std::cout << "PRED LIST: " << pred_list.size() << std::endl;
  std::cout << "GT LIST: " << truth_list.size() << std::endl;
  // Save the pred and gt values
  // {
  //   pred: []
  //   truth: []
  // }

  std::string r_s = std::to_string(ref_start);
  std::string q_s = std::to_string(query_start);
  std::string dir = std::to_string(method);
  json j;
  std::ofstream o("/home/nate/Development/semnatickitteval/results/" +
                  sequence +  "/" + sample + "/" + dir + "/pr" + r_s + "_" + q_s + ".json");
  j["pred"] = pred_list;
  j["truth"] = truth_list;
  j["poses"] = pose_prob_list;
  o << std::setw(4) << j << std::endl;
}

int main(int argc, char **argv) {
  // Define the program options
  boost::program_options::options_description desc("Allowed options");
  desc.add_options()(
      "s", boost::program_options::value<std::string>(&sequence)->required(),
      "The input sequence")(
      "samp", boost::program_options::value<std::string>(&sample)->required(),
      "The sample size")(
      "f", boost::program_options::value<std::string>(&result_file)->required(),
      "The results file name")(
      "r_s", boost::program_options::value<int>(&ref_start)->required(),
      "Ref start")(
      "r_e", boost::program_options::value<int>(&ref_end)->required(),
      "Ref end")(
      "q_s", boost::program_options::value<int>(&query_start)->required(),
      "Query start")(
      "q_e", boost::program_options::value<int>(&query_end)->required(),
      "Query end")(
      "inc", boost::program_options::value<int>(&inc)->required(),
      "Key frame inc")(
      "m", boost::program_options::value<int>(&method)->required(),
        "Method: 0 all 1 max prob"
      );

  // Parse the command line arguments
  boost::program_options::variables_map vm;
  try {
    boost::program_options::store(
        boost::program_options::parse_command_line(argc, argv, desc), vm);

    // Handle the help option
    if (vm.count("help")) {
      std::cout << desc << std::endl;
      return 0;
    }

    boost::program_options::notify(vm);
  } catch (boost::program_options::required_option &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  } catch (boost::program_options::error &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  // Use the parsed arguments
  std::cout << "Input sequence: " << sequence << std::endl;
  std::cout << "Output file: " << result_file << std::endl;

  ros::init(argc, argv, "marker_pub");
  ros::NodeHandle n;
  ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>(
      "visualization_marker_array", 1);

  // Get ground truth map and gt_markers
  std::map<std::string, GTReferenceMatch> gt_map;
  visualization_msgs::MarkerArray gt_marker_array;
  //GetGroundTruthMatches(gt_map, gt_marker_array);

  //// Get prediction results for pred_map and pred_marker_array
  std::unordered_map<std::string, PredReferenceMatch> pred_map;
  double threshold = .1;
  visualization_msgs::MarkerArray pred_marker_array;
  //GetPredictionMap(threshold, gt_map, pred_map, pred_marker_array);

  SavePR();

  //// Publish Markers
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
    vis_pub.publish(pred_marker_array);

    r.sleep();
  }

  return 1;
}
