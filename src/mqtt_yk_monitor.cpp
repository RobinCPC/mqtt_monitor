#include <mqtt_monitor/mqtt_yk_monitor.h>
#include "../include/mqtt_monitor/mqtt_yk_monitor.h"

MqttYkMonitor::MqttYkMonitor(ros::NodeHandle nh)
  : nh_(nh)
{
  start_time = ros::Time::now();
  update_time = start_time;

  nh_.param<bool>("in_cloud", in_cloud, false);
  ROS_INFO_STREAM("Is mqtt_monitor node in AWS Cloud? " << in_cloud);
  nh_.param<bool>("is_sim", is_sim, false);
  ROS_INFO_STREAM("Is mqtt_monitor node do simulation with Gazebo? " << is_sim);
  nh_.param<double>("sampling_time", sampling_time, 0.1);
  ROS_INFO_STREAM("Sampling time (sec) for sending out message: " << sampling_time);

  // Subscribe UR modbus data from SRB robot_gateway
  if(in_cloud)
  {
    yk_mqin_sub = nh_.subscribe("/mqin_yk_data", 1, &MqttYkMonitor::ur_mqin_callback, this);
  }else
  {
    ur_mod_sub = nh_.subscribe("/pub_I0", 1, &MqttYkMonitor::ur_msg_callback, this);
    yk_mqtt_pub = nh_.advertise<std_msgs::String>("/mqtt_yk_data", 1);
  }

  if(is_sim)
  {
    target_jnt_pub = nh_.advertise<std_msgs::Float64MultiArray>("/yk/target_jnt_data", 1);

    this->target_jnt_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    this->target_jnt_msg.layout.dim[0].size = 6;  // TODO: use rosparam to get correct value.
    this->target_jnt_msg.layout.dim[0].stride = 1;
    this->target_jnt_msg.layout.dim[0].label = "Joint";
  }else
  {
    yk_jnt_pub = nh_.advertise<sensor_msgs::JointState>("/yk/joint_states", 1);
  }

  //// Publish total connecting time
  //time_pub = nh_.advertise<std_msgs::String>("/mqtt_time", 1);
}

MqttYkMonitor::~MqttYkMonitor(){}

void MqttYkMonitor::ur_msg_callback(const robot_gateway::ur_msg& ur_mod_msg)
{
  //publish Yaskawa joint state for local RViz
  std::vector<double> jointVals = {ur_mod_msg.Addr_270 * d2r,
                                   ur_mod_msg.Addr_271 * d2r,
                                   ur_mod_msg.Addr_272 * d2r,
                                   ur_mod_msg.Addr_273 * d2r,
                                   ur_mod_msg.Addr_274 * d2r,
                                   ur_mod_msg.Addr_275 * d2r
                                  };
  if(!is_sim)
    pub_joint_state(jointVals);

  // publish UR robot data to cloud
  ros::Duration time_diff = (ros::Time::now() - update_time);
  if (time_diff.toSec() >= sampling_time)
  {
    // Compute Power on time  (TODO: need to get real one from UR_modbus)
    update_time = ros::Time::now();
    ros::Duration power_on_time = update_time - start_time;
    double cur_time = power_on_time.toSec();
    int hr = (int) cur_time/3600;
    int mn = (int) cur_time/60;
    int sc = (int) cur_time % 60;
    std::string hms = std::to_string(hr) + ":" + std::to_string(mn) + ":"
      + std::to_string(sc);

    std_msgs::String ur_data_msg;
    std::string ur_data_str = "";
    ur_data_str += hms + ","; // add time string
    ur_data_str += std::to_string(ur_mod_msg.Addr_270) + "," +
      std::to_string(ur_mod_msg.Addr_271) + "," +
      std::to_string(ur_mod_msg.Addr_272) + "," +
      std::to_string(ur_mod_msg.Addr_273) + "," +
      std::to_string(ur_mod_msg.Addr_274) + "," +
      std::to_string(ur_mod_msg.Addr_275) + ",";
    ur_data_str += std::to_string(ur_mod_msg.Addr_400) + "," +
      std::to_string(ur_mod_msg.Addr_401) + "," +
      std::to_string(ur_mod_msg.Addr_402) + "," +
      std::to_string(ur_mod_msg.Addr_403) + "," +
      std::to_string(ur_mod_msg.Addr_404) + "," +
      std::to_string(ur_mod_msg.Addr_405);

    ur_data_msg.data = ur_data_str;
    yk_mqtt_pub.publish(ur_data_msg);

    // publish target joint val per sec
    if(is_sim)
    {
      this->target_jnt_msg.data.clear();
      this->target_jnt_msg.data.insert(this->target_jnt_msg.data.end(),
          jointVals.begin(), jointVals.end());
      this->target_jnt_pub.publish(this->target_jnt_msg);
    }

  }

  return;
}

void MqttYkMonitor::ur_mqin_callback(const std_msgs::String& ur_mqin_msg)
{
  ROS_INFO_STREAM("MQTT in message: " << ur_mqin_msg.data);
  std::vector<std::string> jnt_str{};
  jnt_str = split(ur_mqin_msg.data, ",");

  std::vector<double> jointVals = {};
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

  return;
}

void MqttYkMonitor::pub_joint_state(const std::vector<double>& jntVals)
{
  sensor_msgs::JointState jointStateMsg;
  jointStateMsg.header.stamp = ros::Time::now();
  jointStateMsg.header.frame_id = "base_link";
  jointStateMsg.name = jointNames;
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
  ros::init(argc, argv, "mqtt_ur_monitor");
  ros::NodeHandle nh;

  MqttYkMonitor ur_monitor(nh);

  ros::spin();
  return 0;
}

