#include "tko_charger_ros/tko_charger_ros.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "tko_charger");

    TKOChargerRos charger;

    ros::spin();
    return 0;
}