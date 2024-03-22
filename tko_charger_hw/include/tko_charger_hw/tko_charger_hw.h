#ifndef TKO_CHARGER_HW
#define TKO_CHARGER_HW

#include <string>
#include <iostream>
#include <cstdio>
#include <signal.h>
#include <cmath>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial/serial.h"
#include "crc_check.h"

#include <chrono>

class TKO_CHARGER
{
private:
    // chrono::time_point<chrono::steady_clock> start, end;
    const uint8_t COMMAND_LENGTH = 6;
    uint8_t hex_cmd[6] = {0};
    uint8_t receive_hex[26] = {0};

    // PROTOCOL
    uint8_t ID = 0x00;
    const uint8_t READ = 0x03;
    const uint8_t WRITE = 0x10;

    //COMMAND
    const uint8_t VERSION = 0x00;
    const uint8_t BATTERY_SOC = 0x01;
    const uint8_t BATTERY_WARNING = 0x02;
    const uint8_t CHARGER_DETECTION = 0x03;
    const uint8_t CHARGER_STEP = 0x04;
    const uint8_t EMERGENCY_BUTTON = 0x05;
    const uint8_t MANUAL_BREAK_BUTTON = 0x06;
    const uint8_t CHARGER_ERROR = 0x07;
    const uint8_t VOLTAGE_BATTERY = 0x08;
    const uint8_t VOLTAGE_CHARGER = 0x0A;
    const uint8_t VOLTAGE_LOAD = 0x0C;
    const uint8_t CURRENT_CHARGING = 0x0E;
    const uint8_t CURRENT_LOAD = 0x10;
    const uint8_t TEMPERATURE = 0x12;

// *****************************************************
// ------------------ HELPER FUNCTIONS -----------------
// *****************************************************

    /**
     * @brief calculates the crc and stores it in the hex_cmd array, so there is no return value
     */
    void calculate_crc();

    /**
     * @brief reads from the serial port and saves the string into the receive_hex array
     * @param num_bytes how many bytes to read from the buffer
     * @return return 0 when OK, 1 if crc error
     */
    uint8_t read_hex(uint8_t num_bytes);

    /**
     * @brief print the hex command for debugging
     * @param num_bytes how many bytes to print
     */
    void print_hex_cmd(uint8_t num_bytes) const;

    /**
     * @brief print received hex for debugging
     * @param num_bytes how many bytes to print
     */
    void print_rec_hex(uint8_t register_addr, uint8_t num_bytes) const;

public:
    void sleep(unsigned long milliseconds);

    serial::Serial _serial;

    /**
     * @brief open serial port communication
     * @param port COM port eg. "/dev/ttyUSB0"
     * @param baudRate default baudrate is 115200
     * @param _ID Set the modbus ID of the motor driver in HEX, default 0x00
     */
    void begin(std::string port, int baudrate, uint8_t _ID = 0x00);

    /**
     * @brief Read data form the charger
     * @param register_addr of the COMMAND
     * @param register_length how many register to read
     * @param return_length size of bytes returned by command, usually 1 or 2. If read multiple register -> sum of all commands
     * @return 0 if ok, 1 if crc read error
     */
    uint8_t read(uint8_t register_addr, uint8_t data_length);

// *****************************************************
// ------------------ GETTER FUNCTIONS -----------------
// *****************************************************
    
    /**
     * @return version number or ID?
     */
    uint8_t get_version();

    /**
     * @return battery state of charge
     * State of charge: 0 – 3 
     * 0 = 75%-100% 
     * 1 = 50%-75% 
     * 2 = 25%-50% 
     * 3 = 0%-25%
     */
    uint8_t get_battery_soc();

    /**
     * @return Warning of the battery 
     * Uvw: under voltage warning 
     * Ovw: over voltage warning 
     * Otw: over temperature warning 
     * (uint8_t)((battery_uvw<<2)|(battery_ovw<<1)|(battery_otw))
     */
    uint8_t get_battery_warning();

    /**
     * @return 0: no detect 1: detected
     */
    uint8_t get_charge_detected();

    /**
     * @return status of charger
     * CHARGER_STEP_OFF         0 
     * CHARGER_STEP_READY       1 
     * CHARGER_STEP_DETECTED    2 
     * CHARGER_STEP_WAIT        3 
     * CHARGER_STEP_CHARGING    4 
     * CHARGER_STEP_FULL        5 
     * CHARGER_STEP_STOP        6 
     * CHARGER_STEP_ERROR       9
     */
    uint8_t get_charge_step();

    /**
     * @return 0: Emergency button not pressed 1: Emergency button pressed
     */
    uint8_t get_emergency_button();

    /**
     * @return 0: Brake button not pressed 1: Brake button pressed
     */
    uint8_t get_manual_break_button();

    /**
     * @return CHARGER_ERR_BATTERY 1 CHARGER_ERR_VOLTAGE 2 
     * (TBC)
     */
    uint8_t get_charger_error();

    /**
     * @return Voltage of Battery (mV) 
     * 16 bits
     */
    uint16_t get_battery_voltage();

    /**
     * @return Charger Voltage (mV) 
     * 16 bits
     */
    uint16_t get_charger_voltage();

    /**
     * @return Voltage of Load on Battery (mV) 
     * 16 bits
     */
    uint16_t get_load_voltage();

    /**
     * @return Charging Current (No available) 
     */
    int get_charging_current();

    /**
     * @return Load Current (No available) 
     * 16 bits
     */
    int get_load_current();

    /**
     * @return Temperature (No available) 
     * 16 bits
     */
    uint16_t get_temperature();
};

#endif