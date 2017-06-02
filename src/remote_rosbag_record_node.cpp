
#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/service_server.h>
#include <rosbag/recorder.h>
#include <std_srvs/Empty.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/thread.hpp>

boost::scoped_ptr< rosbag::Recorder > recorder;
boost::thread run_thread;

bool start(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
  // do nothing if the recorder already started
  if (recorder) {
    return false;
  }

  // read record options
  rosbag::RecorderOptions options;
  if (!ros::param::get("~topics", options.topics)) {
    ROS_ERROR("No topics to record");
    return false;
  }
  // TODO: read more options

  // launch rosbag-record. this thread will continue unless ros::shutdown has been called
  recorder.reset(new rosbag::Recorder(options));
  run_thread = boost::thread(&rosbag::Recorder::run, recorder.get());

  return true;
}

bool stop(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
  // TODO: delayed shutdown
  // (because direct call of ros::shutdown() may disconnect the current client)
  ros::shutdown();

  return true;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "remote_rosbag_record");
  ros::NodeHandle nh;

  nh.advertiseService("start", start);
  nh.advertiseService("stop", stop);

  // run services. this serving will continue unless ros::shutdown has been called
  ros::spin();

  // finalize the rosbag-record thread
  if (run_thread.joinable()) {
    run_thread.join();
  }

  return 0;
}