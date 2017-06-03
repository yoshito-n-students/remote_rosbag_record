#include <ros/callback_queue.h>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/service_server.h>
#include <ros/spinner.h>
#include <rosbag/recorder.h>
#include <std_srvs/Empty.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/thread.hpp>

boost::scoped_ptr< rosbag::Recorder > recorder;
boost::thread run_thread;

bool start(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
  namespace rp = ros::param;

  // do nothing if the recorder already started
  if (recorder) {
    ROS_ERROR("Already started");
    return false;
  }

  // read recorder options
  rosbag::RecorderOptions options;
  rp::get("~record_all", options.record_all);
  rp::get("~append_date", options.append_date);
  rp::get("~name", options.name);
  rp::get("~topics", options.topics);
  // TODO: read more options

  // launch rosbag-record. this thread will continue unless ros::shutdown has been called
  ROS_INFO("Start recording");
  recorder.reset(new rosbag::Recorder(options));
  run_thread = boost::thread(&rosbag::Recorder::run, recorder.get());

  return true;
}

bool stop(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
  // TODO: delayed shutdown
  // (because direct call of ros::shutdown() may disconnect the current client)
  ROS_INFO("Stop recording");
  ros::shutdown();

  return true;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "remote_rosbag_record");
  ros::NodeHandle nh;

  // rosbag will use the global queue.
  // to avoid confriction, use local queue for following serveces
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);

  ros::ServiceServer start_server(nh.advertiseService("start", start));
  ros::ServiceServer stop_server(nh.advertiseService("stop", stop));

  // run services. this serving will continue unless ros::shutdown has been called
  ros::SingleThreadedSpinner spinner;
  spinner.spin(&queue);

  // finalize the rosbag-record thread
  if (run_thread.joinable()) {
    run_thread.join();
  }

  return 0;
}