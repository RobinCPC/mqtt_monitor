// Author: Robin Chen
#ifndef  MQTT_UR_MONITOR_H
#define  MQTT_UR_MONITOR_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
//#include <robot_gateway/ur_msg.h>

#include <vector>
#include <string>

#define DEG2RAD 0.017453292519943

class MqttUrMonitor
{
public:
  MqttUrMonitor(ros::NodeHandle nh);  // Constructor
  ~MqttUrMonitor();                   // Destructor

private:
  ros::NodeHandle nh_;

  //ros::Subscriber ur_mod_sub;
  ros::Publisher ur_jnt_pub;
  //ros::Publisher ur_mqtt_pub;
  ros::Subscriber ur_mqin_sub;

  //ros::Publisher time_pub;
  ros::Time start_time;
  ros::Time update_time;

  std::vector<std::string> jointNames = {"shoulder_pan_joint",
                                         "shoulder_lift_joint",
                                         "elbow_joint",
                                         "wrist_1_joint",
                                         "wrist_2_joint",
                                         "wrist_3_joint"};
  const double r2d = 57.295779513;
  const double d2r = 0.017453292519943;
  bool in_cloud = false;

  //void ur_msg_callback(const robot_gateway::ur_msg& ur_mod_msg);
  void ur_mqin_callback(const std_msgs::String& ur_mqin_msg);
  void pub_joint_state(const std::vector<double>& jntVals);
  std::vector<std::string> split(std::string cmd, std::string delimiter);

};
#endif // MQTT_UR_MONITOR_H

