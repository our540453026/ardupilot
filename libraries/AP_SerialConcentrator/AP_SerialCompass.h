/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <stack>
#define ID_5883 0xA1
#define ID_8310 0xA2
class AP_SerialCompass {

public:
    struct PACKED mag_t
        {
            uint8_t head;
            uint8_t id;	
            uint8_t len;
            int16_t mag_x;
            int16_t mag_y;
            int16_t mag_z;
            uint32_t times;
        };
    typedef struct PACKED {
            uint8_t instance;
            uint32_t time_us;
            int16_t magX; // mGauss, front
            int16_t magY; // mGauss, right
            int16_t magZ; // mGauss, down
    } serial_compass_data_message_t;
    uint32_t        _last_update_time;
    bool    Tx_urgent_statue = false;
   // ~AP_SerialCompass();

    /* Do not allow copies */
  //  AP_SerialCompass(const AP_SerialCompass &other) = delete;
    //AP_SerialCompass &operator=(const AP_SerialCompass&) = delete;

    // init - perform required initialisation
    AP_SerialCompass();
    int8_t get_port(void);
    bool init(const AP_SerialManager &serial_manager);
    void send_data(uint8_t *buffer,uint8_t size);
    bool update(void);
    serial_compass_data_message_t mag_state;
    static AP_SerialCompass *get_singleton(void) { return _singleton; }
private:
    int8_t port_num;
    uint16_t g_crc_val;
    int32_t swapInt32( int32_t value);
    int16_t swapInt16( int16_t value);
    void update_tx_buf(void);
    bool Data_Message_Process(void);
    void update_disconnect(void);
    uint8_t crcSum(uint8_t *dat,uint8_t n);
    bool parse(uint8_t temp);
    static const uint8_t DATA_PREAMBLE1 = 0x88;
    AP_HAL:: UARTDriver *_port;
    static AP_SerialCompass *_singleton;
    union PACKED msgbuffer {
                mag_t mag;
                uint8_t bytes[256];
            };
    struct PACKED data_msg_parser
        {
            enum
            {
                PREAMBLE1 = 0,
                MSGID,
                LENGTH1,
                PAYLOAD,
                CRC1,
            } rec_state;
            uint8_t length;
            uint8_t msgid;
            msgbuffer data;
            uint8_t crc;
            uint8_t read_count;
        } data_msg;

};

namespace AP {
AP_SerialCompass *serialCompass();
};
