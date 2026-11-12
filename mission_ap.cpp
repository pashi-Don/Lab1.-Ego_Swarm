#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <fstream>
#include <cmath>
#include <vector>
#include <chrono>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

struct Waypoint {
    double x, y, z;
    std::string name;
};

double getDistance(double x1, double y1, double z1, double x2, double y2, double z2) {
    return std::hypot(std::hypot(x2 - x1, y2 - y1), z2 - z1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_ap_node");
    ros::NodeHandle nh;             
    ros::NodeHandle pnh("~");       
    Waypoint ptA, ptB, ptC, ptD;
    ptA.name = "A"; ptB.name = "B"; ptC.name = "C"; ptD.name = "D";
    if (!pnh.getParam("A_x", ptA.x) || !pnh.getParam("A_y", ptA.y) || !pnh.getParam("A_z", ptA.z)) {
        ROS_ERROR("Failed to load point A from YAML! Shutting down.");
        return 1;
    }
    if (!pnh.getParam("B_x", ptB.x) || !pnh.getParam("B_y", ptB.y) || !pnh.getParam("B_z", ptB.z)) {
        ROS_ERROR("Failed to load point B from YAML! Shutting down.");
        return 1;
    if (!pnh.getParam("C_x", ptC.x) || !pnh.getParam("C_y", ptC.y) || !pnh.getParam("C_z", ptC.z)) {
        ROS_ERROR("Failed to load point C from YAML! Shutting down.");
        return 1;
    }
    if (!pnh.getParam("D_x", ptD.x) || !pnh.getParam("D_y", ptD.y) || !pnh.getParam("D_z", ptD.z)) {
        ROS_ERROR("Failed to load point D from YAML! Shutting down.");
        return 1;
    }
    double h = ptA.z;
    
    std::vector<Waypoint> mission = {ptA, ptB, ptC, ptD, ptA};

    ROS_INFO("Successfully loaded coordinates from YAML.");
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

    ros::Rate rate(20.0);

    ROS_INFO("Waiting for FCU connection...");
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected!");

    std::ofstream log_file("flight_log.csv");
    log_file << "time,x,y,z,target\n";
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";

    while(ros::ok() && current_state.mode != "GUIDED"){
        if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
            ROS_INFO("GUIDED enabled");
        }
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    while(ros::ok() && !current_state.armed){
        if(arming_client.call(arm_cmd) && arm_cmd.response.success){
            ROS_INFO("Vehicle armed");
        }
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    mavros_msgs::CommandTOL takeoff_cmd;
    takeoff_cmd.request.altitude = h;
    if(takeoff_client.call(takeoff_cmd) && takeoff_cmd.response.success){
        ROS_INFO("Takeoff command sent. Target height: %.1f m", h);
    }
    
    while(ros::ok() && current_pose.pose.position.z < 0.95 * h){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Takeoff complete. Starting mission...");

    auto start_time = std::chrono::steady_clock::now();
    double total_path_length = 0.0;
    geometry_msgs::PoseStamped last_pose = current_pose;
    for (size_t i = 0; i < mission.size(); ++i) {
        Waypoint target = mission[i];
        ROS_INFO("Moving to Waypoint %s: [%.1f, %.1f, %.1f]", target.name.c_str(), target.x, target.y, target.z);

        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id = "map";
        target_pose.pose.position.x = target.x;
        target_pose.pose.position.y = target.y;
        target_pose.pose.position.z = target.z;
        target_pose.pose.orientation.w = 1.0;

        bool point_reached = false;
        ros::Time entered_radius_time;

        while(ros::ok() && !point_reached){
            local_pos_pub.publish(target_pose);
            double current_time = ros::Time::now().toSec();
            log_file << current_time << "," 
                     << current_pose.pose.position.x << "," 
                     << current_pose.pose.position.y << "," 
                     << current_pose.pose.position.z << "," 
                     << target.name << "\n";
            log_file.flush();
            double dist_step = getDistance(last_pose.pose.position.x, last_pose.pose.position.y, last_pose.pose.position.z,
                                           current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
            total_path_length += dist_step;
            last_pose = current_pose;
            double dist_to_target = getDistance(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                                                target.x, target.y, target.z);

            if (dist_to_target < 0.5) {
                if (entered_radius_time.isZero()) {
                    entered_radius_time = ros::Time::now();
                } else if ((ros::Time::now() - entered_radius_time).toSec() >= 1.0) {
                    ROS_INFO("Waypoint %s reached!", target.name.c_str());
                    point_reached = true;
                }
            } else {
                entered_radius_time = ros::Time(0);
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

    auto end_time = std::chrono::steady_clock::now();
    double total_time = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
    offb_set_mode.request.custom_mode = "LAND";
    if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
        ROS_INFO("LAND command sent.");
    }

    ROS_INFO("Waiting for disarm...");
    while(ros::ok() && current_state.armed){
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    
    ROS_INFO("--- MISSION COMPLETE ---");
    ROS_INFO("Total Time: %.1f seconds", total_time);
    ROS_INFO("Total Path Length: %.2f meters", total_path_length);
    log_file.close();

    return 0;
}
