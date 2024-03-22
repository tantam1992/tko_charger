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
    // convert string to hex
    for (uint8_t i = 0; i < return_byte_length; i++)
    {
        receive_hex[register_addr + i] = uint8_t(line[i]);
        // printf("rec %d, %02x\n", i, receive_hex[register_addr + i]);
    }

    // crc check of received data
    if (crc16(receive_hex + register_addr, return_byte_length) != 0)
    {
        _serial.flush();
        printf("crc check error read_charger\n");
        return 1;
    }
    // print_rec_hex(register_addr, return_byte_length);

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

uint8_t TKO_CHARGER::get_battery_warning()
{
    return receive_hex[6];
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

uint16_t TKO_CHARGER::get_battery_voltage()
{
    uint16_t battery_voltage = receive_hex[13] + (receive_hex[12] << 8);
    return battery_voltage;
}

uint16_t TKO_CHARGER::get_charger_voltage()
{
    uint16_t charger_voltage = receive_hex[15] + (receive_hex[14] << 8);
    return charger_voltage;
}

uint16_t TKO_CHARGER::get_load_voltage()
{
    uint16_t load_voltage = receive_hex[17] + (receive_hex[16] << 8);
    return load_voltage;
}

int TKO_CHARGER::get_charging_current()
{
    uint16_t charging_current = receive_hex[19] + (receive_hex[18] << 8);
    return ((int)charging_current - 8190) * 16;
}

int TKO_CHARGER::get_load_current()
{
    uint16_t load_current = receive_hex[21] + (receive_hex[20] << 8);
    return ((int)load_current - 1638) * 16;
}


uint16_t TKO_CHARGER::get_temperature()
{
    uint16_t temperature = receive_hex[23] + (receive_hex[22] << 8);
    return temperature;
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
