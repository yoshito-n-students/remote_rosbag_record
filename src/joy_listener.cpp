#include <remote_rosbag_record/call.hpp>
#include <ros/init.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Joy.h>

#include <boost/regex.hpp>

// params
int start_button, stop_button;
boost::regex start_regex, stop_regex;
bool verbose;

// callback
void onJoyRecieved(const sensor_msgs::JoyConstPtr &joy) {
  if (joy->buttons.size() > start_button && joy->buttons[start_button] > 0) {
    remote_rosbag_record::call(start_regex, verbose);
  }

  if (joy->buttons.size() > stop_button && joy->buttons[stop_button] > 0) {
    remote_rosbag_record::call(stop_regex, verbose);
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "joy_listener");

  // load params
  ros::NodeHandle nh, pnh("~");
  start_button = pnh.param("start_button", 12 /* PS4's R3 button */);
  stop_button = pnh.param("stop_button", 11 /* PS4's L3 button */);
  start_regex =
      pnh.param< std::string >("start_regex", ros::names::append(nh.getNamespace(), "start"));
  stop_regex =
      pnh.param< std::string >("stop_regex", ros::names::append(nh.getNamespace(), "stop"));
  verbose = pnh.param("verbose", true);

  // start subscribing
  ros::Subscriber sub(nh.subscribe("joy", 1, onJoyRecieved));

  ros::spin();

  return 0;
}