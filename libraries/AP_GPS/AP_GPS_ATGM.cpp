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

//  ATGMtel/Tersus/ComNav GPS driver for ArduPilot.
//  Code by Michael Oborne
//  Derived from http://www.ATGMtel.com/assets/Documents/Manuals/om-20000129.pdf

#include "AP_GPS.h"
#include "AP_GPS_ATGM.h"
#include <AP_Logger/AP_Logger.h>
#include <../ArduCopter/Copter.h>
#if AP_GPS_ATGM_ENABLED

extern const AP_HAL::HAL& hal;

#define ATGM_DEBUGGING 0

#if ATGM_DEBUGGING
#include <cstdio>
 # define Debug(fmt, args ...)                  \
do {                                            \
    printf("%s:%d: " fmt "\n",     \
                        __FUNCTION__, __LINE__, \
                        ## args);               \
    hal.scheduler->delay(1);                    \
} while(0)
#else
 # define Debug(fmt, args ...)
#endif

AP_GPS_ATGM::AP_GPS_ATGM(AP_GPS &_gps, AP_GPS::GPS_State &_state,
                       AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port)
{
    ATGM_msg.ATGM_state = ATGM_msg_parser::PREAMBLE1;

    //ATGM_msg.header.data[0] = ATGM_PREAMBLE1;
    //ATGM_msg.header.data[1] = ATGM_PREAMBLE2;

    /*if (gps._auto_config != AP_GPS::GPS_AUTO_CONFIG_DISABLE) {
        const char *init_str = _initialisation_blob[0];
        const char *init_str1 = _initialisation_blob[1];

        port->write((const uint8_t*)init_str, strlen(init_str));
        port->write((const uint8_t*)init_str1, strlen(init_str1));
    }*/
}

const uint8_t _initialisation[4][14]={
    {0xBA, 0xCE, 0x04, 0x00, 0x06, 0x01, 0x01, 0x01, 0x0A, 0x00, 0x05, 0x01, 0x10, 0x01}, //dop 5hz
    {0xBA, 0xCE, 0x04, 0x00, 0x06, 0x01, 0x01, 0x02, 0x0A, 0x00, 0x05, 0x02, 0x10, 0x01},	/* sol 5hz*/
    {0xBA, 0xCE, 0x04, 0x00, 0x06, 0x01, 0x01, 0x03, 0x0A, 0x00, 0x05, 0x03, 0x10, 0x01},	/* pv 5hz */
    {0xBA, 0xCE, 0x04, 0x00, 0x06, 0x01, 0x01, 0x10, 0x0A, 0x00, 0x05, 0x10, 0x10, 0x01},	/* timeutc 5hz */
    };
// Process all bytes available from the stream
//	/* BIN command, write(fd, cmdbuf, sizeof(cmdbuf)); */
	/* BIN command, write(fd, cmdbuf, sizeof(cmdbuf)); */
	/* BIN command, write(fd, cmdbuf, sizeof(cmdbuf)); */
const char* const AP_GPS_ATGM::_initialisation_blob[2] {
    "\r\n\r\n$PCAS02,100*1E\r\n", // cleanup enviroment
    "$PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0*02\r\n"
};

bool
AP_GPS_ATGM::read(void)
{
    if (gps._auto_config != AP_GPS::GPS_AUTO_CONFIG_DISABLE) {
        const uint32_t now = AP_HAL::millis();

        if (_init_blob_index < 6) {           
            if (now > _init_blob_time) {
                if(_init_blob_index <2)
                {
                    const char *init_str = _initialisation_blob[_init_blob_index];
                    port->write((const uint8_t*)init_str, strlen(init_str));
                }
                else
                {
                    port->write(_initialisation[_init_blob_index-2], 14);
                }
                _init_blob_time = now + 200;
                _init_blob_index++;
            }
        }
    }

    bool ret = false;
    while (port->available() > 0) {
        uint8_t temp = port->read();
        ret |= parse(temp);
        
    }
    
    return ret;
}

bool
AP_GPS_ATGM::parse(uint8_t temp)
{
    switch (ATGM_msg.ATGM_state)
    {
        default:
        case ATGM_msg_parser::PREAMBLE1:
            if (temp == ATGM_PREAMBLE1) {
                ATGM_msg.ATGM_state = ATGM_msg_parser::PREAMBLE2;
                //gcs().send_text(MAV_SEVERITY_INFO,"p1");
            }
            ATGM_msg.read = 0;
            break;
        case ATGM_msg_parser::PREAMBLE2:
            if (temp == ATGM_PREAMBLE2) {
                ATGM_msg.ATGM_state = ATGM_msg_parser::LENGTH1;
                //gcs().send_text(MAV_SEVERITY_INFO,"p2");
            } else {
                ATGM_msg.ATGM_state = ATGM_msg_parser::PREAMBLE1;
            }
            break;
        case ATGM_msg_parser::LENGTH1:
            ATGM_msg.length = (uint16_t) (temp << 0);
            ATGM_msg.ATGM_state = ATGM_msg_parser::LENGTH2;
            break;
        case ATGM_msg_parser::LENGTH2:
            ATGM_msg.length += (uint16_t) (temp << 8);   
            if(ATGM_msg.length >=100) 
            {
                ATGM_msg.ATGM_state = ATGM_msg_parser::PREAMBLE1;
                gcs().send_text(MAV_SEVERITY_INFO,"len_error");
            }
            else ATGM_msg.ATGM_state = ATGM_msg_parser::MSG1;
            break;
        case ATGM_msg_parser::MSG1:
            ATGM_msg.mesid = (uint16_t) (temp << 0);
            ATGM_msg.ATGM_state = ATGM_msg_parser::MSG2;
            break;
        case ATGM_msg_parser::MSG2:
            ATGM_msg.mesid += (uint16_t) (temp << 8);
            ATGM_msg.ATGM_state = ATGM_msg_parser::DATA;
            ATGM_msg.read = 0;
            break;
        case ATGM_msg_parser::DATA:
            ATGM_msg.data.bytes[ATGM_msg.read] = temp;
            ATGM_msg.read++;
            if (ATGM_msg.read >= ATGM_msg.length) {
                Debug("ATGM DATA exit\n");
                ATGM_msg.ATGM_state = ATGM_msg_parser::CRC1;
            }
            break;
        case ATGM_msg_parser::CRC1:
            ATGM_msg.crc = (uint32_t) (temp << 0);
            ATGM_msg.ATGM_state = ATGM_msg_parser::CRC2;
            break;
        case ATGM_msg_parser::CRC2:
            ATGM_msg.crc += (uint32_t) (temp << 8);
            ATGM_msg.ATGM_state = ATGM_msg_parser::CRC3;
            break;
        case ATGM_msg_parser::CRC3:
            ATGM_msg.crc += (uint32_t) (temp << 16);
            ATGM_msg.ATGM_state = ATGM_msg_parser::CRC4;
            break;
        case ATGM_msg_parser::CRC4:
            ATGM_msg.crc += (uint32_t) (temp << 24);
            ATGM_msg.ATGM_state = ATGM_msg_parser::PREAMBLE1;
            uint32_t crc = CalculateBlockCRC((uint32_t)ATGM_msg.length);
            if (ATGM_msg.crc == crc) {
                //gcs().send_text(MAV_SEVERITY_INFO,"atmg_com_ok");
                return process_message();
            } else {
                //gcs().send_text(MAV_SEVERITY_INFO,"atmg_com_error");
                crc_error_counter++;
            }
            break;
    }

    return false;
}

bool
AP_GPS_ATGM::process_message(void)
{
    const uint16_t messageid = ATGM_msg.mesid;

    Debug("ATGM process_message messid=%u\n",messageid);
    if (messageid == 0x0101) // psrdop
    {
        const psrdop &psrdopu = ATGM_msg.data.psrdopu;

        state.hdop = (uint16_t) (psrdopu.hdop*100);
        state.vdop = (uint16_t) (psrdopu.vdop*100);
        return false;
    }
    if (messageid == 0x0301) // bestpv
    {
        const bestpv &bestpvu = ATGM_msg.data.bestpvu;

        //state.time_week = ATGM_msg.header.ATGM_headeru.week;
        //state.time_week_ms = (uint32_t) ATGM_msg.header.ATGM_headeru.tow;
        state.last_gps_time_ms = AP_HAL::millis();

        state.location.lat = (int32_t) (bestpvu.lat * (double)1e7);
        state.location.lng = (int32_t) (bestpvu.lng * (double)1e7);
        state.location.alt = (int32_t) (bestpvu.hgt * 100);

        state.num_sats = bestpvu.numSV;

        state.horizontal_accuracy = bestpvu.hAcc;
        state.vertical_accuracy = bestpvu.vAcc;
        state.have_horizontal_accuracy = true;
        state.have_vertical_accuracy = true;
        
        state.ground_speed = (float) bestpvu.speed2D;
        state.ground_course    = wrap_360(bestpvu.heading );       // Heading 2D deg * 100000
        state.have_vertical_velocity = true;
        state.velocity.x = bestpvu.velN ;
        state.velocity.y = bestpvu.velE ;
        state.velocity.z = bestpvu.velU ;
        state.have_speed_accuracy = true;
        state.speed_accuracy = bestpvu.sAcc;
        _new_speed = true;
        
    }
    if (messageid == 0x1001) // bestvel
    {
        const timeutc &timeutcu = ATGM_msg.data.timeutcu;
        _new_date = timeutcu.year*10000 + timeutcu.month*100 + timeutcu.day;
        _new_time = timeutcu.hour*10000000 + timeutcu.min*100000 + timeutcu.sec*1000 + timeutcu.ms;
        make_gps_time(_new_date, _new_time * 10);
        //gcs().send_text(MAV_SEVERITY_INFO,"y=%d,m=%d,d%d,%ld,%ld",timeutcu.year,timeutcu.month,timeutcu.day,_new_date,_new_time);
    }
    if (messageid == 0x0201) // bestvel
    {
        const bestsol &bestsolu = ATGM_msg.data.bestsolu;
        state.time_week = ATGM_msg.data.bestsolu.week;
        switch (bestsolu.posValid){
            case 0: // Fix not available or invalid
                state.status = AP_GPS::NO_FIX;
                break;           
            case 6: // Estimated (dead reckoning) Mode
                state.status = AP_GPS::GPS_OK_FIX_2D;
                break;
            case 7: // Estimated (dead reckoning) Mode
                state.status = AP_GPS::GPS_OK_FIX_3D;
                break;
            default://to maintain compatibility with MAV_GPS_INPUT and others
                state.status = AP_GPS::NO_FIX;
                break;
        }
        _new_position = true;
    }
    if (_new_position && _new_speed ) {
        _new_speed = _new_position = false;
        
        return true;
    }
    return false;
}



uint32_t AP_GPS_ATGM::CalculateBlockCRC(uint32_t length)
{
    uint32_t ckSum;
    uint16_t i;
    ckSum = (ATGM_msg.mesid << 16) + ATGM_msg.length;
    for (i = 0; i < (length / 4); i++)
    {
        ckSum = ckSum + ATGM_msg.data.uint32s[i];
    }
    return( ckSum );
}
#endif
