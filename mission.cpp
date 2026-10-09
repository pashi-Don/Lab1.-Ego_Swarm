#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <fstream>
#include <vector>
#include <string>

struct Point3D {
    double x, y, z;
};

Point3D current_pos = {0.0, 0.0, 0.0};
bool odom_received = false;
double total_distance = 0.0;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (odom_received) {
        double dx = msg->pose.pose.position.x - current_pos.x;
        double dy = msg->pose.pose.position.y - current_pos.y;
        double dz = msg->pose.pose.position.z - current_pos.z;
        total_distance += std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    current_pos.x = msg->pose.pose.position.x;
    current_pos.y = msg->pose.pose.position.y;
    current_pos.z = msg->pose.pose.position.z;
    odom_received = true;
}

double getDistance(Point3D p1, Point3D p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::Subscriber odom_sub = nh.subscribe("/drone_0_visual_slam/odom", 10, odomCallback);
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    ros::Rate loop_rate(10); 

    ROS_INFO("Mission Started! Waiting for odometry to set Point A (spawn location)...");
    while (ros::ok() && !odom_received) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::vector<Point3D> targets(5);
    targets[0] = current_pos; 
    ROS_INFO("Point A (Spawn) set at: %.2f, %.2f, %.2f", targets[0].x, targets[0].y, targets[0].z);

   
    bool params_ok = true;
    params_ok &= nh_private.getParam("B_x", targets[1].x);
    params_ok &= nh_private.getParam("B_y", targets[1].y);
    params_ok &= nh_private.getParam("B_z", targets[1].z);
    params_ok &= nh_private.getParam("C_x", targets[2].x);
    params_ok &= nh_private.getParam("C_y", targets[2].y);
    params_ok &= nh_private.getParam("C_z", targets[2].z);
    params_ok &= nh_private.getParam("D_x", targets[3].x);
    params_ok &= nh_private.getParam("D_y", targets[3].y);
    params_ok &= nh_private.getParam("D_z", targets[3].z);

    if (!params_ok) {
        ROS_ERROR("CRITICAL ERROR: Coordinates not found in rosparam!");
        ROS_ERROR("The YAML file was not loaded correctly.");
        ROS_ERROR("Please run the node using: roslaunch lab_dontsov mission.launch");
        ros::shutdown();
        return 1;
    }
    
    targets[4] = targets[0]; 
    std::vector<std::string> target_names = {"A", "B", "C", "D", "A (Return)"};

    std::ofstream csv_file("flight_log.csv");
    csv_file << "Time,X,Y,Z,Target_Index,Distance_to_Target\n";

    int current_target_idx = 1; 
    bool target_published = false;
    bool in_arrival_zone = false;
    ros::Time arrival_start_time;
    
    ros::Time mission_start_time = ros::Time::now();
    std::vector<double> segment_times;
    ros::Time segment_start_time = mission_start_time;

    while (ros::ok() && current_target_idx < 5) {
        ros::spinOnce();

        Point3D current_target = targets[current_target_idx];
        double dist_to_target = getDistance(current_pos, current_target);

        if (!target_published) {
            geometry_msgs::PoseStamped goal_msg;
            goal_msg.header.stamp = ros::Time::now();
            goal_msg.header.frame_id = "world";
            goal_msg.pose.position.x = current_target.x;
            goal_msg.pose.position.y = current_target.y;
            goal_msg.pose.position.z = current_target.z;
            goal_msg.pose.orientation.w = 1.0;
            
            goal_pub.publish(goal_msg);
            target_published = true;
            in_arrival_zone = false;
            ROS_INFO("Flying to target %s (%.2f, %.2f, %.2f)", 
                     target_names[current_target_idx].c_str(), current_target.x, current_target.y, current_target.z);
        }

        ROS_INFO_THROTTLE(1.0, "Distance to %s: %.2f meters", target_names[current_target_idx].c_str(), dist_to_target);

        if (dist_to_target < 1.5) {
            if (!in_arrival_zone) {
                in_arrival_zone = true;
                arrival_start_time = ros::Time::now();
            } else {
                if ((ros::Time::now() - arrival_start_time).toSec() >= 1.0) {
                    double seg_time = (ros::Time::now() - segment_start_time).toSec();
                    segment_times.push_back(seg_time);
                    ROS_INFO("Target %s reached in %.2f seconds!", target_names[current_target_idx].c_str(), seg_time);
                    
                    current_target_idx++;
                    target_published = false; 
                    segment_start_time = ros::Time::now();
                }
            }
        } else {
            in_arrival_zone = false; 
        }

        double time_from_start = (ros::Time::now() - mission_start_time).toSec();
        csv_file << time_from_start << "," 
                 << current_pos.x << "," << current_pos.y << "," << current_pos.z << "," 
                 << current_target_idx << "," << dist_to_target << "\n";

        loop_rate.sleep();
    }

    csv_file.close();

    double total_time = (ros::Time::now() - mission_start_time).toSec();
    ROS_INFO("=== MISSION COMPLETE ===");
    ROS_INFO("Total Mission Time: %.2f s", total_time);
    ROS_INFO("Total Path Length: %.2f m", total_distance);
    for (size_t i = 0; i < segment_times.size(); ++i) {
        ROS_INFO("Segment %s -> %s time: %.2f s", 
                 target_names[i].c_str(), target_names[i+1].c_str(), segment_times[i]);
    }

    return 0;
}