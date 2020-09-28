# remote_rosbag_record
ROS nodes that start/stop rosbag-record by a remote trigger

## Usage
1. Launch recorder nodes with your favourite nodes. Use respawn options to record multiple times.
```
<launch>

  <group ns="console">
    <include file="console.machine"/>

    ... other console nodes ...
    
    <node name="console-record" pkg="remote_rosbag_record" type="record" respawn="true">
      <rosparam file="console_topics.yml"/>
      <param name="prefix" value="console"/>
      <param name="apped_date" value="true"/>
    </node>
  </group>

  <group ns="robot">
    <include file="robot.machine"/>

    ... other robot nodes ....

    <node name="robot-record" pkg="remote_rosbag_record" type="record" respawn="true">
      <rosparam file="robot_topics.yml"/>
      <param name="prefix" value="robot"/>
      <param name="apped_date" value="true"/>
    </node>
  </group>
  
</launch>
```

```
# xxx_topics.yml may look like this
topics:
- /joint_states
- /imu
- ...
```

2. Start rosbag-record at your own timing

* method 1: **use remote_rosbag_record/trigger node**
```
rosrun remote_rosbag_record trigger _regex:="/.*/start"
```

* method 2: **use remote_rosbag_record/joy_listener node**
```
rosrun remote_rosbag_record joy_listener _start_regex:="/.*/start" _start_button:=10 ...
```

* method 3: **call trigger services from your teleoperation node**
```
#include<remote_rosbag_record/call.hpp>

#include<boost/regex.hpp>

...

if (start_button_pressed) {
  remote_rosbag_record::call(boost::regex("/.*/start"));
}
```

3. Stop rosbag-record (similar to the previous step)

## Nodes

### record
starts/stops rosbag-record when a coressponding service is called

##### Services
* start (std_srvs/Empty)
* stop (std_srvs/Empty)

##### Parameters
All parameters are optional and function like the original rosbag-record.
See http://wiki.ros.org/rosbag/Commandline#record

* ~record_all (bool)
* ~regex (bool)
* ~quiet (bool)
* ~append_date (bool)
* ~verbose (bool)
* ~compression (string)
  * "uncompressed", "bz2", or "lz4"
* ~prefix (string)
* ~name (string)
* ~topics (string array)
* ~buffer_size (int)
* ~exclude_regex (string)
* ~node (string)

### trigger
calls multiple start/stop services at once

##### Parameters
* ~regex (string, required)
  * regular expression which matches services you want to call
* ~verbose (bool, default: true)
  * verbose console output when calling services

### joy_listener
subscribes joystick messages and calls start/stop services when a specified button is pressed

#### Subscribed topics
* joy (sensor_msgs/Joy)

#### Parameters
* ~start_button (int, default: 12)
  * button id to trigger start services
* ~start_regex (string, default: "<node's namespace>/start")
  * regular expression which matches start services
* ~stop_button (int, default: 11)
  * button id to trigger stop services
* ~stop_regex (string, default: "<node's namespace>/stop")
  * regular expression which matches stop services
* ~verbose (bool, default: true)
  * verbose console output when calling services