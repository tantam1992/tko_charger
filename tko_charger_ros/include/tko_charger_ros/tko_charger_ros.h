#ifndef TKO_CHARGER_ROS_H
#define TKO_CHARGER_ROS_H

#include "ros/ros.h"
#include "sensor_msgs/BatteryState.h"
// #include "tko_charger_hw/tko_charger_hw.h"

class TKOChargerRos
{
public:
    TKOChargerRos();
    void init_charger_hw();
    void init_battery_state();

private:
    ros::NodeHandle nh_;

    void Timer1HzCallbackCallback(const ros::TimerEvent& event);
    ros::Timer timer_1hz_cb_timer;
    bool timer1HzTimeOut;

    sensor_msgs::BatteryState battery_msg;
    ros::Publisher battery_state_pub;
};


#endif