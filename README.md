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

* method 2: **call trigger services from your teleoperation node**
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
All parameters function like original rosbag-record.
See http://wiki.ros.org/rosbag/Commandline#record

* ~record_all
* ~regex
* ~quiet
* ~append_date
* ~verbose
* ~compression (string)
  * "uncompressed", "bz2", or "lz4"
* ~prefix
* ~name
* ~topics
* ~buffer_size
* ~exclude_regex
* ~node

### trigger
calls multiple start/stop services at once

##### Parameters
* ~regex (string)
  * regular expression which matches services you want to call
