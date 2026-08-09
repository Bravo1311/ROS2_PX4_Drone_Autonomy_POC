#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <cmath>
#include <map>
#include <vector>
#include <string>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>

using json = nlohmann::json; 

class ArucoMapAnnotator : public rclcpp::Node {
public:
    ArucoMapAnnotator(): Node("aruco_map_annotator_cpp") {
        auto map_qos = rclcpp::QoS(1)
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::TransientLocal);
  
        detections_sub_ = create_subscription<std_msgs::msg::String>(
            "/aruco_detections_json",
            10,
            std::bind(&ArucoMapAnnotator::cb_detections, this, std::placeholders::_1)
        );
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            std::bind(&ArucoMapAnnotator::cb_odom, this, std::placeholders::_1)
        );
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            map_qos,
            std::bind(&ArucoMapAnnotator::cb_map, this, std::placeholders::_1)
        );

        save_srv_ = create_service<std_srvs::srv::Trigger>(
            "/save_marker_map",
            std::bind(&ArucoMapAnnotator::cb_save, this, 
                std::placeholders::_1, std::placeholders::_2)
        );

        // At the end of the constructor
        RCLCPP_INFO(get_logger(), "ArucoMapAnnotator ready. Call /save_marker_map when done.");
    }

private:
    //  Odometry Message
    double x_pos_ = 0.0;
    double y_pos_ = 0.0;
    double z_pos_ = 0.0;
    double yaw_ = 0.0;
    bool odom_ready_ = false;

    double min_score_ = 0.3;

    // Map Message
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    bool map_ready_ = false;

    // marker msgs
    std::map<int, std::vector<std::pair<double, double>>> detections_;

    std::string output_dir_ = "/home/bravo1311/px4_ros2_ws/src/"
                          "ROS2-PX4_Drone_Teleoperation_Using_Joystick/maps";

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr detections_sub_;    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;  

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;

    void cb_save(
        const std_srvs::srv::Trigger::Request::SharedPtr req,
        std_srvs::srv::Trigger::Response::SharedPtr res){
            if(!map_ready_){
                res->success = false;
                res->message = "No map received yet";
                return;
            } 

            if(detections_.empty()){
                res->success = false;
                res->message = "No markers detected yet";
                return;
            }

            std::map<int, std::pair<double, double>> markers;

            for( auto & [id, pts]: detections_){
                double sum_x = 0.0;
                double sum_y = 0.0;
                for(auto & pt: pts){
                    sum_x += pt.first;
                    sum_y += pt.second;
                }
                double avg_x = sum_x/pts.size();
                double avg_y = sum_y/pts.size();
                markers[id] = {avg_x, avg_y};

                RCLCPP_INFO(get_logger(),
                    "Marker %d: (%.3f, %.3f) from %zu for detections",
                    id, avg_x, avg_y, pts.size());
            }
            save_yaml(markers);
            annotate_map(markers);
            res->success = true;
            res->message = "Saved" + std::to_string(markers.size()) + "markers.";
        }

    void save_yaml(const std::map<int, std::pair<double, double>> & markers){
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "markers" << YAML::Value;
        out << YAML::BeginMap;

        for (auto & [id, pos]: markers){
            out << YAML::Key << id;
            out << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "x" << YAML::Value << pos.first;
            out << YAML::Key << "y" << YAML::Value << pos.second;
            out << YAML::Key << "frame" << YAML::Value << "map";
            out << YAML::EndMap;
        }

        out << YAML::EndMap;
        out << YAML::EndMap;

        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);

        std::ostringstream oss;
        oss << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S");

        std::string path = output_dir_ + "/markers_" + oss.str() + ".yaml";
        std::ofstream f(path);
        f << out.c_str();

        RCLCPP_INFO(get_logger(), "Saved markers -> %s", path.c_str());
    }

    void annotate_map(const std::map<int, std::pair<double, double>> & markers){
        auto & info = map_->info;
        double res = info.resolution;
        double origin_x = info.origin.position.x;
        double origin_y = info.origin.position.y;
        uint32_t width = info.width;
        uint32_t height = info.height;

        cv::Mat img(height, width, CV_8UC1);
        for (uint32_t r = 0; r < height; r++){
            for (uint32_t c = 0; c < width; c++){
                int8_t val = map_->data[(height - 1 - r)*width + c];
                uint8_t pixel;
                if (val == -1) pixel = 128; // unknown -> grey
                else if (val == 0) pixel = 255; //free -> white
                else pixel = 0; // occupied -> black
                img.at<uint8_t>(r,c) = pixel;
            }
        }

        cv:: Mat img_bgr;
        cv::cvtColor(img, img_bgr, cv::COLOR_GRAY2BGR);

        for (auto & [id, pos]: markers){
            double mx = pos.first;
            double my = pos.second;

            int px = static_cast<int>((mx - origin_x) / res);
            int py = static_cast<int>(height - (my - origin_y) / res);

            px = std::clamp(px, 5, (int)width -5);
            py = std::clamp(py, 5, (int)height -5);

            cv::circle(img_bgr, {px, py}, 14, {0,0,255}, -1);
            cv::circle(img_bgr, {px, py}, 14, {255,255,255}, 2);

            std::string label = "ID:" + std::to_string(id);
            cv::putText(img_bgr, label, {px + 18, py}, cv::FONT_HERSHEY_SIMPLEX,
                0.6, {0, 0, 255}, 2, cv::LINE_AA);
        }
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);

        std::ostringstream oss;
        oss << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S");

        std::string path = output_dir_ + "/markers_" + oss.str() + ".yaml";
        std::string out_path = output_dir_ + "/walls_map_annotated" + oss.str() + ".png";
        cv::imwrite(out_path, img_bgr);
        RCLCPP_INFO(get_logger(), "Saved annotated map -> %s", out_path.c_str());
    }

    void cb_detections(const std_msgs::msg::String::SharedPtr msg){
        if(!odom_ready_) return;

        json data;
        try
        {
            data = json::parse(msg->data);
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(get_logger(), "JSON parse error: %s", e.what());
            return;
        }
        
        int n_avg_ = 50;

        for(auto & det: data["detections"]){
            int id = det["id"].get<int>();
            auto & tvec = det["tvec"];
            double score = det.value("score", 0.0);
            // if(score < min_score_) continue;

            // if(abs(tvec[2].get<double>()-z_pos_) > 3.0) continue;

            // Frame Conversion
            double body_dx = -tvec[1].get<double>();
            double body_dy = -tvec[0].get<double>();

            // Body -> Map frame;
            double map_x = x_pos_ + body_dx*cos(yaw_) -body_dy*sin(yaw_);
            double map_y = y_pos_ + body_dx*sin(yaw_) +body_dy*cos(yaw_);

            detections_[id].push_back({map_x, map_y});
            if ((int)detections_[id].size() > n_avg_){
                detections_[id].erase(detections_[id].begin());
            }
            // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            //     "Odom %d: %zu detections |   =(%.3f, %.3f, %.3f)",
            //     id, detections_[id].size(), x_pos_, y_pos_, yaw_);
            // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            //     "Marker %d: %zu detections | latest=(%.3f, %.3f)",
            //     id, detections_[id].size(), map_x, map_y);
            RCLCPP_INFO(get_logger(),
                "DEBUG | drone=(%.3f, %.3f) yaw=%.3f | "
                "tvec=(%.3f, %.3f, %.3f) | "
                "body_dx=%.3f body_dy=%.3f | "
                "map=(%.3f, %.3f)",
                x_pos_, y_pos_, yaw_,
                tvec[0].get<double>(), tvec[1].get<double>(), tvec[2].get<double>(),
                body_dx, body_dy,
                map_x, map_y);
        }
    }

    void cb_odom(const nav_msgs::msg::Odometry::SharedPtr msg){
        x_pos_ = msg->pose.pose.position.x;
        y_pos_ = msg->pose.pose.position.y;
        z_pos_ = msg->pose.pose.position.z;
        auto & q = msg->pose.pose.orientation;

        yaw_ = std::atan2(2.0 * (q.w*q.z + q.x*q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z));
        odom_ready_ = true;
    }
    
    void cb_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        map_ = msg;
        map_ready_ = true;
    }
};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoMapAnnotator>());
    rclcpp::shutdown();
    return 0;
}