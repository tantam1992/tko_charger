#include "ros/ros.h"
#include "tko_charger_ros/tko_charger_ros.h"

TKOChargerRos::TKOChargerRos() : nh_("~")
{

  init_charger_hw();
  init_battery_state();

  timer_1hz_cb_timer = nh_.createTimer(ros::Duration(1), &TKOChargerRos::Timer1HzCallbackCallback, this, false);
}

void TKOChargerRos::init_battery_state()
{
  battery_state_pub = nh_.advertise<sensor_msgs::BatteryState>("battery", 1);
  battery_msg.header.frame_id = "tko_battery";
}

void TKOChargerRos::init_charger_hw()
{
  // TKOChargerRos::begin("/dev/charger", 115200, 0x50);
}

void TKOChargerRos::Timer1HzCallbackCallback(const ros::TimerEvent &event)
{
  battery_state_pub.publish(battery_msg);
}