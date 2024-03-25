# TKO charger pkgs
ROS interface with the charger board on TKO robot for getting battery status.
It consist of two packages. 
- [tko_charger_hw](https://github.com/JosefGst/tko_charger/tree/master/tko_charger_hw) is a library package which establishes the serial communication with the charger board.
- [tko_charger_ros](https://github.com/JosefGst/tko_charger/tree/master/tko_charger_ros) is the ros wrapper around the charger hardware package and publishes the ros topics about the battery status. 