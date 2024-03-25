#ifndef TKO_CHARGER_ROS_H
#define TKO_CHARGER_ROS_H

#include "ros/ros.h"
#include "sensor_msgs/BatteryState.h"
#include "std_msgs/Bool.h"
#include "tko_charger_hw/tko_charger_hw.h"

TKO_CHARGER charger_hw;

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
    void Timer10HzCallbackCallback(const ros::TimerEvent& event);
    ros::Timer timer_10hz_cb_timer;

    sensor_msgs::BatteryState battery_msg;
    std_msgs::Bool charger_detected;

    ros::Publisher battery_state_pub;
    ros::Publisher charger_detected_pub;

    std::string serial_port;
};


#endif