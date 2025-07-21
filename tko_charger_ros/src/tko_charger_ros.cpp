#include "ros/ros.h"
#include "tko_charger_ros/tko_charger_ros.h"

TKOChargerRos::TKOChargerRos() : nh_("~")
{
  init_charger_hw();
  init_battery_state();

  timer_2hz_cb_timer = nh_.createTimer(ros::Duration(0.5), &TKOChargerRos::Timer2HzCallbackCallback, this, false);
  timer_10hz_cb_timer = nh_.createTimer(ros::Duration(0.1), &TKOChargerRos::Timer10HzCallbackCallback, this, false);
}

void TKOChargerRos::init_battery_state()
{
  battery_state_pub = nh_.advertise<sensor_msgs::BatteryState>("battery", 1);
  battery_msg.header.frame_id = "tko_battery";

  charger_state_pub = nh_.advertise<tko_charger_interfaces::TkoCharger>("state", 1);
  charger_state.header.frame_id = "tko_charger";
  charger_state.version = charger_hw.get_version();

  charger_detected_pub = nh_.advertise<std_msgs::Bool>("charger_detected", 1);
}

void TKOChargerRos::init_charger_hw()
{
  serial_port = "/dev/charger";
  try {
    charger_hw.begin(serial_port, 115200, 0x50);
    serial_stat = SER_STAT_CONNECTED ;
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception occurred while opening tko_charger hardware serial port: " << e.what());
    // Handle the exception here, such as logging the error or taking appropriate action
    serial_stat = SER_STAT_DISCONNECT ;
  }
  ROS_INFO_STREAM("Charger board Serial Port open success, com_port_name= " << serial_port);
}

void TKOChargerRos::Timer2HzCallbackCallback(const ros::TimerEvent &event)
{
  int retry_counter = 0;
  int noData_counter = 0;

  if (serial_stat == SER_STAT_DISCONNECT) {
    init_charger_hw();
    return;
  }

  while (retry_counter < MAX_ERR_RETRY) {
    int return_code = charger_hw.read(0x00, 20);
    switch (return_code) {
      case 0:
      {
        // Battery topic
        battery_msg.header.stamp = ros::Time::now();
        battery_msg.voltage = charger_hw.get_charger_voltage();
        battery_msg.temperature = charger_hw.get_temperature();
        battery_msg.current = charger_hw.get_load_current();
        battery_msg.charge = charger_hw.get_charging_current();
        battery_msg.percentage = charger_hw.get_battery_percentage();
        battery_msg.power_supply_status = charger_hw.get_charge_detected();
        battery_msg.power_supply_health = charger_hw.get_charger_error();
        battery_msg.present = charger_hw.get_charge_detected();
        battery_state_pub.publish(battery_msg);

        // State topic
        charger_state.header.stamp = ros::Time::now();
        charger_state.battery_soc = charger_hw.get_battery_percentage();
        charger_state.battery_warning = charger_hw.get_battery_warning_string();
        charger_state.charger_detected = charger_hw.get_charge_detected();
        charger_state.charger_step = charger_hw.get_charge_step();
        charger_state.emergency_button = charger_hw.get_emergency_button();
        charger_state.manual_break = charger_hw.get_manual_break_button();
        charger_state.charger_error = charger_hw.get_charger_error();

        float battery_v = charger_hw.get_battery_voltage();
        float charger_v = charger_hw.get_charger_voltage();

        charger_state.battery_voltage = battery_v;
        charger_state.charger_voltage = charger_v;
        charger_state.load_voltage = charger_hw.get_load_voltage();
        charger_state.charging_current = charger_hw.get_charging_current();
        charger_state.load_current = charger_hw.get_load_current();
        charger_state.temperature = charger_hw.get_temperature();

        // Charging detection
        charger_state.charging = (fabs(charger_v - battery_v) <= 10.0);

        charger_state_pub.publish(charger_state);
        break;
      }
      
      case 1:
        ROS_DEBUG_STREAM("CRC check error");
        break;

      case 2:
        ROS_DEBUG_STREAM("Message length error");
        break;

      case 3:
        ROS_DEBUG_STREAM("Error code detected");
        break;

      case 4:
        ROS_DEBUG_STREAM("Address code error");
        break;

      case SERIAL_ERR_NOT_OPEN:
        ROS_DEBUG_STREAM("Serial port not open");
        charger_hw.closeConnect();
        serial_stat = SER_STAT_DISCONNECT;
        return;
        break;

      case SERIAL_ERR_NO_DATA:
        ROS_DEBUG_STREAM("Serial no data");
        break;

      case SERIAL_ERR_WRITE:
        ROS_DEBUG_STREAM("Serial write error");
        break;

      case SERIAL_ERR_READ:
        ROS_DEBUG_STREAM("Serial read error");
        break;

      default:
        break;
    }

    if (return_code == 0 ){
      return ;
    }else{
      // handle serial error 
      if (return_code >= SERIAL_ERR_NO_DATA){
        if (noData_counter < MAX_TIMEOUT_RETRY){
        noData_counter++;
        }else{
        charger_hw.closeConnect();
          serial_stat = SER_STAT_DISCONNECT ; 
          return ; 
        }
      }
      retry_counter++ ;
      charger_hw.sleep(100); 
    }
  }
}

void TKOChargerRos::Timer10HzCallbackCallback(const ros::TimerEvent &event)
{
  charger_detected.data = charger_hw.get_charge_detected();
  charger_detected_pub.publish(charger_detected);
}
