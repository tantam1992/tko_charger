#include "tko_charger_hw/tko_charger_hw.h"

void TKO_CHARGER::sleep(unsigned long milliseconds)
{
#ifdef _WIN32
    Sleep(milliseconds); // 100 ms
#else
    usleep(milliseconds * 1000); // 100 ms
#endif
}

void TKO_CHARGER::begin(std::string port, int baudrate, uint8_t _ID)
{
    ID = _ID;
    _serial.setPort(port);
    _serial.setBaudrate(baudrate);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(200);
    _serial.setTimeout(timeout);

    _serial.open();
    _serial.flushInput();
    std::cout << "SERIAL OK!" << std::endl;
}

uint8_t TKO_CHARGER::read(uint8_t register_addr, uint8_t data_length)
{
    uint8_t rx_serial_buff[26] = {0}; // tom edit

    // memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = ID;
    hex_cmd[1] = READ;
    hex_cmd[2] = register_addr;
    hex_cmd[3] = data_length;

    calculate_crc();

    _serial.write(hex_cmd, COMMAND_LENGTH);
    // print_hex_cmd(COMMAND_LENGTH);

    uint8_t return_byte_length = COMMAND_LENGTH+data_length;
    std::string line = _serial.read(return_byte_length);

    /****************** Tom Edit ******************/ 
    if (uint8_t(line[0]) == 0x50) {
        rx_serial_buff[0] = uint8_t(line[0]) ;
    }else{
        // address error
        _serial.flushInput();
        printf("address error read_charger\n");
        return 4; 
    }

    if (uint8_t(line[1]) < 0x80) {
        rx_serial_buff[1] = uint8_t(line[1]) ;
    }else{
        // cmd error
        _serial.flushInput();
        printf("cmd error read_charger: %02x\n", uint8_t(line[2]));
        return 3; 
    }

    uint8_t rx_reg_address = uint8_t(line[2]);
    uint8_t rx_data_length = uint8_t(line[3]);
    if ( (rx_reg_address + rx_data_length) < 21 ){
        return_byte_length = rx_data_length + COMMAND_LENGTH ;
    }else{
        // lenght error
         _serial.flushInput();       
        printf("lenght error read_charger: %d\n", uint8_t(line[3]));
        return 2;        
    }

    for (uint8_t i = 2; i < return_byte_length; i++)
    {
        rx_serial_buff[i] = uint8_t(line[i]);
    }

    if (crc16(rx_serial_buff, return_byte_length) != 0){
        // crc error 
        _serial.flushInput();
        printf("crc check error read_charger\n");
        return 1;
    }

    receive_hex[0] = rx_serial_buff[0];     // device address
    receive_hex[1] = rx_serial_buff[1];     // cmd code
    receive_hex[2] = rx_serial_buff[2];     // data register address
    receive_hex[3] = rx_serial_buff[3];     // data lenght

    for (uint8_t i = 4; i < return_byte_length; i++)
    {
        receive_hex[i + rx_reg_address] = rx_serial_buff[i];
    }

    _serial.flushOutput();
    _serial.flushInput();

    /****************** End Tom Edit ******************/ 
    /*
    // convert string to hex
    for (uint8_t i = 0; i < return_byte_length; i++)
    {
        receive_hex[register_addr + i] = uint8_t(line[i]);
        printf("rec %d, %02x\n", i, receive_hex[register_addr + i]);
    }

    // crc check of received data
    if (crc16(receive_hex + register_addr, return_byte_length) != 0)
    {
        // _serial.flush();
        _serial.flushInput();
        printf("crc check error read_charger\n");
        return 1;
    }
    // print_rec_hex(register_addr, return_byte_length);
    */

    return 0;
}

void TKO_CHARGER::calculate_crc()
{
    // calculate crc and append to hex cmd
    unsigned short result = crc16(hex_cmd, sizeof(hex_cmd) - 2);
    hex_cmd[4] = result & 0xFF;
    hex_cmd[5] = (result >> 8) & 0xFF;
}

// *****************************************************
// ------------------ GETTER FUNCTIONS -----------------
// *****************************************************

uint8_t TKO_CHARGER::get_version()
{
    return receive_hex[4];
}

uint8_t TKO_CHARGER::get_battery_soc()
{
    return receive_hex[5];
}

float TKO_CHARGER::get_battery_percentage()
{
    uint8_t battery_soc = get_battery_soc();
    switch (battery_soc) {
        case 0:
            return 0.75;
        case 1:
            return 0.5;
        case 2:
            return 0.25;
        case 3:
            return 0.0;
        default:
            return 0.0;
    }

}

uint8_t TKO_CHARGER::get_battery_warning()
{
    return receive_hex[6];
}

std::string TKO_CHARGER::get_battery_warning_string()
{
    uint8_t error_code = get_battery_warning();
    if (error_code == 0)
    {
        return "No Warning";
    }
    else if (error_code == 1)
    {
        return "over temperature warning";
    }
    else if (error_code == 2){
        return "over voltage warning";
    }
    else if (error_code == 4){
        return "under voltage warning";
    }
    else {
        return "Unknown Warning";
    }
}

uint8_t TKO_CHARGER::get_charge_detected()
{
    return receive_hex[7];
}

uint8_t TKO_CHARGER::get_charge_step()
{
    return receive_hex[8];
}

uint8_t TKO_CHARGER::get_emergency_button()
{
    return receive_hex[9];
}

uint8_t TKO_CHARGER::get_manual_break_button()
{
    return receive_hex[10];
}

uint8_t TKO_CHARGER::get_charger_error()
{
    return receive_hex[11];
}

float TKO_CHARGER::get_battery_voltage()
{
    uint16_t battery_voltage = receive_hex[13] + (receive_hex[12] << 8);
    // return (float)battery_voltage/1000.f;
    return (float)battery_voltage*0.004883f; //Tom edit
}

float TKO_CHARGER::get_charger_voltage()
{
    uint16_t charger_voltage = receive_hex[15] + (receive_hex[14] << 8);
    return float(charger_voltage)/1000.f;
}

float TKO_CHARGER::get_load_voltage()
{
    uint16_t load_voltage = receive_hex[17] + (receive_hex[16] << 8);
    return (float)load_voltage/1000.f;
}

float TKO_CHARGER::get_charging_current()
{
    uint16_t charging_current = receive_hex[19] + (receive_hex[18] << 8);
    return ((float)charging_current - 8190.f) * 0.016;
}

float TKO_CHARGER::get_load_current()
{
    uint16_t load_current = receive_hex[21] + (receive_hex[20] << 8);
    // return ((float)load_current - 1638.f) * 0.016;
    return ((float)load_current - 32768) * 0.01119;  //Tom edit
}


float TKO_CHARGER::get_temperature()
{
    uint16_t temperature = receive_hex[23] + (receive_hex[22] << 8);
    return (float)temperature;
}

// *****************************************************
// ------------------ HELPER FUNCTIONS -----------------
// *****************************************************
void TKO_CHARGER::print_hex_cmd(uint8_t num_bytes) const
{
    // print
    for (int i = 0; i < num_bytes; i++)
    {
        printf("send: %d, %02x\n", i, hex_cmd[i]);
    }
}

void TKO_CHARGER::print_rec_hex(uint8_t register_addr, uint8_t num_bytes) const
{
    // print
    for (int i = 0; i < num_bytes; i++)
    {
        printf("rec: %d, %02x\n", i, receive_hex[register_addr + i]);
    }
}
