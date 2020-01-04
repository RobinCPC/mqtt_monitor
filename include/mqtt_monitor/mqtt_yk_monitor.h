// Author: Robin Chen
#ifndef  MQTT_YK_MONITOR_H
#define  MQTT_YK_MONITOR_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <sensor_msgs/JointState.h>

#include <vector>
#include <string>

#define DEG2RAD 0.017453292519943

class MqttYkMonitor
{
public:
  MqttYkMonitor(ros::NodeHandle nh);  // Constructor
  ~MqttYkMonitor();                   // Destructor

private:
  ros::NodeHandle nh_;

  ros::Publisher yk_jnt_pub;
  ros::Subscriber yk_mqin_sub;
  ros::Publisher target_jnt_pub;
  std_msgs::Float64MultiArray target_jnt_msg;

  ros::Time start_time;
  ros::Time update_time;

  // TODO: use config yaml to load jointNames
  std::vector<std::string> jointNames = {"joint_1_s",
                                         "joint_2_l",
                                         "joint_3_u",
                                         "joint_4_r",
                                         "joint_5_b",
                                         "joint_6_t"};

  const double r2d = 57.295779513;
  const double d2r = 0.017453292519943;
  const double mm2meter = 0.001;
  bool in_cloud = false;  // check if run in the cloud service
  bool is_sim = false;    // check if use Gazebo for simulation
  double sampling_time = 0.1;  // time period for message publish to mqtt or control interface

  void yk_mqin_callback(const std_msgs::String& yk_mqin_msg);
  void pub_joint_state(const std::vector<double>& jntVals);
  std::vector<std::string> split(std::string cmd, std::string delimiter);

};
#endif // MQTT_YK_MONITOR_H

