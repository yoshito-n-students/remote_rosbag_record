#include <string>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>

#include <remote_rosbag_record/call.hpp>

#include <boost/regex.hpp>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "remote_rosbag_trigger");
  ros::NodeHandle nh;

  // load regular expression services should match
  boost::regex regex;
  {
    std::string regex_str;
    if (!ros::param::get("~regex", regex_str)) {
      return 0;
    }
    regex = regex_str;
  }

  // call all services which match the regex
  remote_rosbag_record::call(regex);

  return 0;
}