# TKO Charger ROS

## Published Topics

- battery_state [(sensor_msgs/BatteryState)](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/BatteryState.html)
Publishes the status about the battery.

- charger_detected [(std_msgs/Bool)](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/Bool.html)
Publish whether the robot is charging or not.

## Run 

    rosrun tko_charger_ros tko_charger_node