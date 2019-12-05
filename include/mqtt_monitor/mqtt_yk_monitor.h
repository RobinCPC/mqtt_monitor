// Author: Robin Chen
#ifndef  MQTT_YK_MONITOR_H
#define  MQTT_YK_MONITOR_H

#include <ros/ros.h>
#include <std_msgs/String.h>
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

  ros::Time start_time;
  ros::Time update_time;

  std::vector<std::string> jointNames = {"joint_1_s",
                                         "joint_2_l",
                                         "joint_3_u",
                                         "joint_4_r",
                                         "joint_5_b",
                                         "joint_6_t"};

  const double r2d = 57.295779513;
  const double d2r = 0.017453292519943;
  const double mm2meter = 0.001;
  bool in_cloud = false;

  void yk_mqin_callback(const std_msgs::String& yk_mqin_msg);
  void pub_joint_state(const std::vector<double>& jntVals);
  std::vector<std::string> split(std::string cmd, std::string delimiter);

};
#endif // MQTT_YK_MONITOR_H

