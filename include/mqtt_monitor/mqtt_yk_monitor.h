// Author: Robin Chen
#ifndef  MQTT_YK_MONITOR_H
#define  MQTT_YK_MONITOR_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <robot_gateway/ur_msg.h>

#include <vector>
#include <string>
#include <unordered_map>

#define DEG2RAD 0.017453292519943
#define MM2METER 0.001

typedef std::unordered_map<std::string, double> jointUnitMap;


class MqttYkMonitor
{
public:
  MqttYkMonitor(ros::NodeHandle nh);  // Constructor
  ~MqttYkMonitor();                   // Destructor

private:
  ros::NodeHandle nh_;

  ros::Subscriber ur_mod_sub;
  ros::Publisher yk_jnt_pub;
  ros::Publisher yk_mqtt_pub;
  ros::Subscriber yk_mqin_sub;

  //ros::Publisher time_pub;
  ros::Time start_time;
  ros::Time update_time;

  // TODO: use config yaml to update jointNames
  std::vector<std::string> jointNames = {"joint_1_s",
                                         "joint_2_l",
                                         "joint_3_u",
                                         "joint_3_u-tool0"};

  const double r2d = 57.295779513;
  const double d2r = 0.017453292519943;
  const double mm2meter = 0.001;  // convert millimeter to meter unit
  jointUnitMap unitConvet  {{"revolute", DEG2RAD}, {"prismatic", MM2METER}};
  bool in_cloud = false;

  void ur_msg_callback(const robot_gateway::ur_msg& ur_mod_msg);
  void ur_mqin_callback(const std_msgs::String& ur_mqin_msg);
  void pub_joint_state(const std::vector<double>& jntVals);
  std::vector<std::string> split(std::string cmd, std::string delimiter);

};
#endif // MQTT_YK_MONITOR_H

