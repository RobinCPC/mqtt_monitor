#include <mqtt_monitor/mqtt_yk_monitor.h>
#include "../include/mqtt_monitor/mqtt_yk_monitor.h"

MqttYkMonitor::MqttYkMonitor(ros::NodeHandle nh)
  : nh_(nh)
{
  start_time = ros::Time::now();
  update_time = start_time;

  nh_.param<bool>("in_cloud", in_cloud, true);
  ROS_INFO_STREAM("Is mqtt_monitor Node in AWS Cloud? " << in_cloud);
  nh_.param<bool>("is_sim", is_sim, false);
  ROS_INFO_STREAM("Is mqtt_monitor Node do simulation with Gazebo? " << is_sim);
  nh_.param<double>("sampling_time", sampling_time, 0.1);
  ROS_INFO_STREAM("Sampling time (sec) for sending out message: " << sampling_time);

  if(is_sim)
  {
    target_jnt_pub = nh_.advertise<std_msgs::Float64MultiArray>("/yk/target_jnt_data", 1);

    this->target_jnt_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    this->target_jnt_msg.layout.dim[0].size = 6;  // TODO: use rosparam to get correct value.
    this->target_jnt_msg.layout.dim[0].stride = 1;
    this->target_jnt_msg.layout.dim[0].label = "Joint"; 
  }else
  {
    // Publish Yaskawa joints value to joint_states for rviz.
    yk_jnt_pub = nh_.advertise<sensor_msgs::JointState>("/yk/joint_states", 1);
  }
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
  // TODO: update joint number by rosparam, not fixed.
  for(size_t i=1; i < 7; ++i)
  {
    jointVals.push_back( std::stod(jnt_str[i]) * d2r);
  }
  if(!is_sim)
    pub_joint_state(jointVals);

  std::string tmp_str ="";
  for (auto& s : jnt_str)
  {
    tmp_str += s + " ";
  }
  ROS_DEBUG_STREAM("MQTT in message: " << tmp_str);
  
  // Publish target joint to the control interface
  ros::Duration time_diff = (ros::Time::now() - update_time);
  if (is_sim && time_diff.toSec() >= sampling_time)
  {
    update_time = ros::Time::now();
    //ros::Duration power_on_time = update_time - start_time;

    // publish target joint value
    this->target_jnt_msg.data.clear();
    this->target_jnt_msg.data.insert(this->target_jnt_msg.data.end(),
        jointVals.begin(), jointVals.end());
    this->target_jnt_pub.publish(this->target_jnt_msg);
  } 

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

