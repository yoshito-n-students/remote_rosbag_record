#ifndef REMOTE_ROSBAG_RECORD_CALL
#define REMOTE_ROSBAG_RECORD_CALL

#include <string>
#include <vector>

#include <ros/console.h>
#include <ros/master.h>
#include <ros/service.h>
#include <ros/this_node.h>
#include <std_srvs/Empty.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <boost/regex.hpp>

namespace remote_rosbag_record {

inline static void call(const boost::regex &expression, const bool verbose = true) {
  try {
    // call ros master api to get services info
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();
    if (!ros::master::execute("getSystemState", args, result, payload, false)) {
      throw XmlRpc::XmlRpcException("getSystemState");
    }

    // call services which match the given expression
    XmlRpc::XmlRpcValue services(payload[2]);
    for (int i = 0; i < services.size(); ++i) {
      const std::string name(services[i][0]);
      if (!boost::regex_match(name, expression)) {
        continue;
      }
      std_srvs::Empty srv;
      if (ros::service::call(name, srv)) {
        if (verbose) {
          ROS_INFO_STREAM("Called " << name);
        }
      } else {
        ROS_ERROR_STREAM("Failed to call " << name);
      }
    }
  } catch (const XmlRpc::XmlRpcException &error) {
    ROS_ERROR_STREAM("Faild to get service names: " << error.getMessage());
  }
}

} // namespace remote_rosbag_record

#endif