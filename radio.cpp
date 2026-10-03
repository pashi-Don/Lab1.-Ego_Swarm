#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "radio");

  
  ros::NodeHandle n;


  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 500);

  ros::Rate loop_rate(2);

  int count = 0;
  while (ros::ok())
  {
    
    std_msgs::String msg;

    std::stringstream ss;
    ss << "Listener";
    ss << "\n Do you hear me?"<< count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

   
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
