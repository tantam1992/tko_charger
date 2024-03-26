#include "tko_charger_hw/tko_charger_hw.h"

int main(void)
{

    TKO_CHARGER charger;
    // // signal(SIGINT, signal_callback_handler);
    charger.begin("/dev/charger", 115200, 0x50);

    // charger.read(0x00,3);

    //full read
    while(1){
        charger.sleep(1000);

        charger.read(0x00,20);
        printf("version: %d\n", charger.get_version());
        printf("battery_soc: %d\n", charger.get_battery_soc());
        printf("battery_warning: %d, %d, %d \n", charger.get_battery_warning()<<2,charger.get_battery_warning()<<1,charger.get_battery_warning());
        printf("charge detected: %d\n", charger.get_charge_detected());
        printf("charge step: %d\n", charger.get_charge_step());
        printf("emergency button: %d\n", charger.get_emergency_button());
        printf("manual break button: %d\n", charger.get_manual_break_button());
        printf("charger error: %d\n", charger.get_charger_error());
        printf("battery voltage [V]: %f\n", charger.get_battery_voltage());
        printf("charger voltage [V]: %f\n", charger.get_charger_voltage());
        printf("load voltage [V]: %f\n", charger.get_load_voltage());
        printf("charging current [A]: %f\n", charger.get_charging_current());
        printf("load current [A]: %f\n", charger.get_load_current());
        printf("Temperature [NA]: %f\n", charger.get_temperature());
    }
    

    return 0;
}