# TKO Charger ROS

## Published Topics

- battery_state [(sensor_msgs/BatteryState)](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/BatteryState.html)
    - Publishes the status about the battery.
    - voltage[mV] - measured Voltage of Battery
    - temperature - not measured yet 
    - current[mA] - measured current of load
    - charge[mA] - measured current of charge
    - percentage - percentage of charge ranged from 1.0 to 0.0 with 1.0 = 100%
    - power_supply_status -
        * CHARGER_STEP_OFF         0 
        * CHARGER_STEP_READY       1 
        * CHARGER_STEP_DETECTED    2 
        * CHARGER_STEP_WAIT        3 
        * CHARGER_STEP_CHARGING    4 
        * CHARGER_STEP_FULL        5 
        * CHARGER_STEP_STOP        6 
        * CHARGER_STEP_ERROR       9
    - power_supply_health - 
        * Uvw: under voltage warning    4
        * Ovw: over voltage warning     2
        * Otw: over temperature warning 1
        * (uint8_t)((battery_uvw<<2)|(battery_ovw<<1)|(battery_otw))


- charger_detected [(std_msgs/Bool)](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/Bool.html)
Publish whether the robot is charging or not.

## Run 

    rosrun tko_charger_ros tko_charger_node

## TODO
- [ ] set up parameters for reconfiguration
- [ ] custom msg for all state information
- [ ] 
