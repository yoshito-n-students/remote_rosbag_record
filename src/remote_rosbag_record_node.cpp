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
boost::thread shutdown_thread;

bool start(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
  namespace rp = ros::param;

  // do nothing if the recorder already started
  if (recorder || run_thread.joinable()) {
    ROS_ERROR("Already started");
    return false;
  }

  // read recorder options
  rosbag::RecorderOptions options;
  rp::get("~record_all", options.record_all);
  rp::get("~regex", options.regex);
  rp::get("~quiet", options.quiet);
  // note: as of kinetic, the quiet option looks ignored in rosbag::Recorder :(
  rp::get("~append_date", options.append_date);
  rp::get("~verbose", options.verbose);
  {
    std::string compression;
    if (rp::get("~compression", compression)) {
      if (compression == "uncompressed") {
        options.compression = rosbag::compression::Uncompressed;
      } else if (compression == "bz2") {
        options.compression = rosbag::compression::BZ2;
      } else if (compression == "lz4") {
        options.compression = rosbag::compression::LZ4;
      } else {
        ROS_WARN_STREAM("Unknown compression type: " << compression);
      }
    }
  }
  rp::get("~prefix", options.prefix);
  rp::get("~name", options.name);
  rp::get("~topics", options.topics);
  {
    std::string exclude_regex;
    if (rp::get("~exclude_regex", exclude_regex)) {
      options.do_exclude = true;
      options.exclude_regex = exclude_regex;
    }
  }
  {
    int buffer_size;
    if (rp::get("~buffer_size", buffer_size)) {
      options.buffer_size = buffer_size;
    }
  }
  rp::get("~node", options.node);
  // note: this node aims to enable service-triggered logging.
  //       so does not support the following options that may autonomously stop logging
  //   trigger / snapshot / chunk_size / limit / split
  //   / max_size / max_split / max_duration / min_space

  // launch rosbag-record. this thread will continue unless ros::shutdown() has been called
  ROS_INFO("Start recording");
  recorder.reset(new rosbag::Recorder(options));
  run_thread = boost::thread(&rosbag::Recorder::run, recorder.get());

  return true;
}

void sleepAndShutdown() {
  ros::Duration(0.5).sleep();
  ros::shutdown();
}

bool stop(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
  // do nothing if the shutdown already scheduled
  if (shutdown_thread.joinable()) {
    ROS_ERROR("Already stopped");
    return false;
  }

  // schedule a future call of ros::shutdown()
  // (because a direct call disconnects the current client)
  ROS_INFO("Stop recording");
  shutdown_thread = boost::thread(&sleepAndShutdown);

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

  // run services. this serving will continue unless ros::shutdown() has been called
  //ros::SingleThreadedSpinner spinner;
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin(&queue);

  // finalize the rosbag-record threads
  if (run_thread.joinable()) {
    run_thread.join();
  }
  if (shutdown_thread.joinable()) {
    shutdown_thread.join();
  }

  return 0;
}