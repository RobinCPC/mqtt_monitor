#include <mqtt_monitor/mqtt_yk_monitor.h>
#include "../include/mqtt_monitor/mqtt_yk_monitor.h"

MqttYkMonitor::MqttYkMonitor(ros::NodeHandle nh)
  : nh_(nh)
{
  start_time = ros::Time::now();
  update_time = start_time;

  nh_.param<bool>("is_sim", in_cloud, false);
  ROS_INFO_STREAM("Is This Node in AWS Cloud? " << in_cloud);

  // Publish Yaskawa joints value to joint_states for rviz.
  yk_jnt_pub = nh_.advertise<sensor_msgs::JointState>("/yk/joint_states", 1);
  // Subscribe YasKawa data from AWS IoT MQTT Bridge
  yk_mqin_sub = nh_.subscribe("/mqin_ur_data", 1, &MqttYkMonitor::yk_mqin_callback, this);

}

MqttYkMonitor::~MqttYkMonitor(){}

void MqttYkMonitor::yk_mqin_callback(const std_msgs::String& yk_mqin_msg)
{
  ROS_INFO_STREAM("MQTT in message: " << yk_mqin_msg.data);
  std::vector<std::string> jnt_str{};
  jnt_str = split(yk_mqin_msg.data, ",");

  std::vector<double> jointVals = {};
  //for(size_t i=1; i < 5; ++i) 	// TODO: update joint number by rosparam, not fixed.
  //{
  //  jointVals.push_back( std::stod(jnt_str[i]) * d2r);
  //}
  jointVals.push_back( std::stod(jnt_str[1]) * d2r);
  jointVals.push_back( std::stod(jnt_str[2]) * d2r);
  jointVals.push_back( std::stod(jnt_str[3]) * d2r);
  jointVals.push_back( std::stod(jnt_str[4]) * d2r);
  jointVals.push_back( std::stod(jnt_str[5]) * d2r);
  jointVals.push_back( std::stod(jnt_str[6]) * d2r);
  pub_joint_state(jointVals);

  std::string tmp_str ="";
  for (auto& s : jnt_str)
  {
    tmp_str += s + " ";
  }
  ROS_DEBUG_STREAM("MQTT in message: " << tmp_str);

  return;
}

void MqttYkMonitor::pub_joint_state(const std::vector<double>& jntVals)
{
  sensor_msgs::JointState jointStateMsg;
  jointStateMsg.header.stamp = ros::Time::now();
  jointStateMsg.header.frame_id = "base_link";
  jointStateMsg.name = jointNames; 	// TODO: update jointNames by config yaml file.
  jointStateMsg.position = jntVals;
  yk_jnt_pub.publish(jointStateMsg);

  return;
}

std::vector<std::string> MqttYkMonitor::split(std::string cmd, std::string delimiter)
{
  std::vector<std::string> list;
  std::size_t start = 0;
  std::size_t end = 0;
  std::string token;
  while ((end = cmd.find(delimiter, start)) != std::string::npos)
  {
    std::size_t len = end - start;
    token = cmd.substr(start, len);
    list.emplace_back(token);
    start += len + delimiter.length();
  }
  // catch last one
  token = cmd.substr(start, end - start);
  list.emplace_back(token);
  return list;
}

int main( int argc, char **argv )
{
  ros::init(argc, argv, "mqtt_yk_monitor");
  ros::NodeHandle nh;

  MqttYkMonitor yk_monitor(nh);

  ros::spin();
  return 0;
}

