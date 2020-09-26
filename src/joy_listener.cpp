#include <string>

#include <remote_rosbag_record/call.hpp>
#include <ros/init.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Joy.h>

#include <boost/regex.hpp>

// params
int start_button_id, stop_button_id;
boost::regex start_regex, stop_regex;
bool verbose;

// memo
bool start_was_pressed, stop_was_pressed;

// callback
void onJoyRecieved(const sensor_msgs::JoyConstPtr &joy) {
  // handle the start button
  const bool start_is_pressed(joy->buttons.size() > start_button_id &&
                              joy->buttons[start_button_id] > 0);
  if (!start_was_pressed && start_is_pressed) {
    remote_rosbag_record::call(start_regex, verbose);
  }
  start_was_pressed = start_is_pressed;

  // handle the stop button
  const bool stop_is_pressed(joy->buttons.size() > stop_button_id &&
                             joy->buttons[stop_button_id] > 0);
  if (!stop_was_pressed && stop_is_pressed) {
    remote_rosbag_record::call(stop_regex, verbose);
  }
  stop_was_pressed = stop_is_pressed;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "joy_listener");

  // load params
  ros::NodeHandle nh, pnh("~");
  start_button_id = pnh.param("start_button", 12 /* PS4's R3 button */);
  stop_button_id = pnh.param("stop_button", 11 /* PS4's L3 button */);
  start_regex =
      pnh.param< std::string >("start_regex", ros::names::append(nh.getNamespace(), "start"));
  stop_regex =
      pnh.param< std::string >("stop_regex", ros::names::append(nh.getNamespace(), "stop"));
  verbose = pnh.param("verbose", true);

  // init memos
  start_was_pressed = false;
  stop_was_pressed = false;

  // start subscribing
  const ros::Subscriber sub(nh.subscribe("joy", 1, onJoyRecieved));

  ros::spin();

  return 0;
}