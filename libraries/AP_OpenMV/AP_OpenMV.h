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
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <stack>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>

#define OPENMV_TYPE 	0x05
#define OPENMV_STATUE_MSG_ID   0x0401
class AP_OpenMV {
public:
    struct PACKED openmv_t
    {
        bool active;  //0--没有数据  1--有数据
        uint16_t type;
        uint16_t id;
        uint16_t x;
        uint16_t y; /*<  0x01--正常接收*/
        uint16_t r; /*< 0x01--正常接收*/ 
        uint32_t last_update_time;  
    };
    struct PACKED openmv_parse_t
    {
        uint16_t type;
        uint16_t id;
        uint16_t x;
        uint16_t y; /*<  0x01--正常接收*/
        uint16_t r; /*< 0x01--正常接收*/   
    };
    AP_OpenMV();
    // init - perform require initialisation including detecting which protocol to use
    bool init(const AP_SerialManager& serial_manager);

    // update flight control mode. The control mode is vehicle type specific
    bool update(void);
    openmv_t openmv_data[8];
    static AP_OpenMV *get_singleton(void) { return _singleton; }
private:
    uint16_t g_crc_val;
    AP_HAL::UARTDriver *_port;              // UART used to send data to receiver
    int16_t swapInt16( int16_t value);
    uint16_t swapuInt16( uint16_t value);
    static AP_OpenMV *_singleton;
    bool parse(uint8_t temp);
    bool Data_Message_Process(void);
    void update_disconnect(void);
    union PACKED msgbuffer {
                openmv_parse_t openmv_u;
                uint8_t bytes[20];
            };
    struct PACKED data_msg_parser
        {
            enum
            {
                PREAMBLE1 = 0,
                PREAMBLE2,
                LENGTH1,
                LENGTH2,
                COMPTYPE,
                MSGID1,
                MSGID2,
                PAYLOAD,
                CRC1,
                CRC2,
            } rec_state;
            uint16_t length;
            uint16_t msgid;
            uint8_t comptype;
            msgbuffer data;
            uint16_t crc;
            uint16_t read_count;
        } data_msg;
};
namespace AP {
AP_OpenMV *openmv();
};
