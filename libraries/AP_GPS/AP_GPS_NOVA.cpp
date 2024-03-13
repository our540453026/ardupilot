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

//  Novatel/Tersus/ComNav GPS driver for ArduPilot.
//  Code by Michael Oborne
//  Derived from http://www.novatel.com/assets/Documents/Manuals/om-20000129.pdf

#include "AP_GPS.h"
#include "AP_GPS_NOVA.h"
#include <AP_Logger/AP_Logger.h>

#if AP_GPS_NOVA_ENABLED

extern const AP_HAL::HAL& hal;

#define NOVA_DEBUGGING 0

#if NOVA_DEBUGGING
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

AP_GPS_NOVA::AP_GPS_NOVA(AP_GPS &_gps, AP_GPS::GPS_State &_state,
                       AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port)
{
    nova_msg.nova_state = nova_msg_parser::PREAMBLE1;

    nova_msg.header.data[0] = NOVA_PREAMBLE1;
    nova_msg.header.data[1] = NOVA_PREAMBLE2;
    nova_msg.header.data[2] = NOVA_PREAMBLE3;

    if (gps._auto_config != AP_GPS::GPS_AUTO_CONFIG_DISABLE) {
        const char *init_str = _initialisation_blob[0];
        const char *init_str1 = _initialisation_blob[1];

        port->write((const uint8_t*)init_str, strlen(init_str));
        port->write((const uint8_t*)init_str1, strlen(init_str1));
    }
}

const char* const AP_GPS_NOVA::_initialisation_blob[31] {
     "\r\n\r\nunlogall\r\n", // cleanup enviroment
    "CONFIG HEADING OFFSET 0 0\r\n",
    "GPTHS 1\r\n",  //设置双天线定向
    "BESTNAVB 0.1\r\n", // poll message, as dop only changes when a sat is dropped/added to the visible list
    "STADOPB 0.2\r\n", // tersus
    "UNIHEADINGB 0.05\r\n"
    "COM COM1 460800\r\n",
    "unlogall com1 \r\n",
    "GPSEPHB COM1 10 \r\n",
	"BD3EPHB COM1 11 \r\n",
	"BDSEPHB COM1 12 \r\n",
	"GLOEPHB COM1 13 \r\n",
	"GALEPHB COM1 14 \r\n",
    "LOG COM1 GPSEPHEMB ONTIME 280\r\n",
    "LOG COM1 GLOEPHEMERISB ONTIME 290\r\n",
    "LOG COM1 GALEPHEMERISB ONTIME 300\r\n",
    "LOG COM1 BD2EPHEMB ONTIME 310\r\n",
    "LOG COM1 RANGECMPB ONTIME 0.2\r\n",
    "LOG COM1 GPGGA ONTIME 0.2\r\n",
    "LOG COM1 GPRMC ONTIME 1\r\n",
    "LOG COM1 GPZDA ONTIME 1\r\n",
    "log COM1 EVENTMARKA ONCHANGED\r\n",//482
    "log COM1 EVENTALLA ONCHANGED\r\n",//482
    "LOG COM1 EVENTMARKB ONCHANGED\r\n",
    "CONFIG EVENT ENABLE NEGATIVE 500\r\n",
    "CONFIG PPS ENABLE GPS POSITIVE 10000 1000 0 0\r\n",
    "config nmeaversion v31\r\n", 
    "CONFIG RTK TIMEOUT 600 \r\n",
    "SAVECONFIG \r\n",
    // "MASK GPS \r\n",
    // "MASK GLO \r\n",
    // "MASK GAL \r\n",
    // "MASK QZSS \r\n",

};

// Process all bytes available from the stream
//
bool
AP_GPS_NOVA::read(void)
{
    if (gps._auto_config != AP_GPS::GPS_AUTO_CONFIG_DISABLE) {
        const uint32_t now = AP_HAL::millis();

        if (_init_blob_index < (sizeof(_initialisation_blob) / sizeof(_initialisation_blob[0]))) {
            const char *init_str = _initialisation_blob[_init_blob_index];

            if (now > _init_blob_time) {
                port->write((const uint8_t*)init_str, strlen(init_str));
                _init_blob_time = now + 200;
                _init_blob_index++;
            }
        }
    }

    bool ret = false;
    while (port->available() > 0) {
        uint8_t temp = port->read();
#if AP_GPS_DEBUG_LOGGING_ENABLED
        log_data(&temp, 1);
#endif
        ret |= parse(temp);
    }
    
    return ret;
}

bool
AP_GPS_NOVA::parse(uint8_t temp)
{
    switch (nova_msg.nova_state)
    {
        default:
        case nova_msg_parser::PREAMBLE1:
            if (temp == NOVA_PREAMBLE1) {
                nova_msg.nova_state = nova_msg_parser::PREAMBLE2;
            }
            nova_msg.read = 0;
            break;
        case nova_msg_parser::PREAMBLE2:
            if (temp == NOVA_PREAMBLE2) {
                nova_msg.nova_state = nova_msg_parser::PREAMBLE3;
            } else {
                nova_msg.nova_state = nova_msg_parser::PREAMBLE1;
            }
            break;
        case nova_msg_parser::PREAMBLE3:
            if (temp == NOVA_PREAMBLE3) {
                nova_msg.header.data[2] = NOVA_PREAMBLE3;
                nova_msg.nova_state = nova_msg_parser::HEADERLENGTH;
            } 
            if(temp == UM_PREAMBLE3){
                    nova_msg.nova_state = nova_msg_parser::UMHEADERDATA;
                    nova_msg.header.data[2] = UM_PREAMBLE3;
                    nova_msg.read = 3;
                }else {
                nova_msg.nova_state = nova_msg_parser::PREAMBLE1;
            }
            break;
        case nova_msg_parser::HEADERLENGTH:
            Debug("NOVA HEADERLENGTH\n");
            nova_msg.header.data[3] = temp;
            nova_msg.header.nova_headeru.headerlength = temp;
            nova_msg.nova_state = nova_msg_parser::HEADERDATA;
            nova_msg.read = 4;
            break;
        case nova_msg_parser::HEADERDATA:
            if (nova_msg.read >= sizeof(nova_msg.header.data)) {
                Debug("parse header overflow length=%u\n", (unsigned)nova_msg.read);
                nova_msg.nova_state = nova_msg_parser::PREAMBLE1;
                break;
            }
            nova_msg.header.data[nova_msg.read] = temp;
            nova_msg.read++;
            if (nova_msg.read >= nova_msg.header.nova_headeru.headerlength) {
                nova_msg.nova_state = nova_msg_parser::DATA;
            }
            break;
        case nova_msg_parser::DATA:
            if (nova_msg.read >= sizeof(nova_msg.data)) {
                Debug("parse data overflow length=%u msglength=%u\n", (unsigned)nova_msg.read,nova_msg.header.nova_headeru.messagelength);
                nova_msg.nova_state = nova_msg_parser::PREAMBLE1;
                break;
            }
            nova_msg.data.bytes[nova_msg.read - nova_msg.header.nova_headeru.headerlength] = temp;
            nova_msg.read++;
            if (nova_msg.read >= (nova_msg.header.nova_headeru.messagelength + nova_msg.header.nova_headeru.headerlength)) {
                Debug("NOVA DATA exit\n");
                nova_msg.nova_state = nova_msg_parser::CRC1;
            }
            break;
        case nova_msg_parser::CRC1:
            nova_msg.crc = (uint32_t) (temp << 0);
            nova_msg.nova_state = nova_msg_parser::CRC2;
            break;
        case nova_msg_parser::CRC2:
            nova_msg.crc += (uint32_t) (temp << 8);
            nova_msg.nova_state = nova_msg_parser::CRC3;
            break;
        case nova_msg_parser::CRC3:
            nova_msg.crc += (uint32_t) (temp << 16);
            nova_msg.nova_state = nova_msg_parser::CRC4;
            break;
        case nova_msg_parser::CRC4:
            nova_msg.crc += (uint32_t) (temp << 24);
            nova_msg.nova_state = nova_msg_parser::PREAMBLE1;

            nova_crc = CalculateBlockCRC32((uint32_t)nova_msg.header.nova_headeru.headerlength, (uint8_t *)&nova_msg.header.data, (uint32_t)0);
            nova_crc = CalculateBlockCRC32((uint32_t)nova_msg.header.nova_headeru.messagelength, (uint8_t *)&nova_msg.data, nova_crc);

            if (nova_msg.crc == nova_crc) {
                return process_message();
            } else {
                Debug("crc failed");
                crc_error_counter++;
            }
            break;
        case nova_msg_parser::UMHEADERDATA:
            if (nova_msg.read >= sizeof(nova_msg.header.data)) {
                Debug("parse header overflow length=%u\n", (unsigned)nova_msg.read);
                nova_msg.nova_state = nova_msg_parser::PREAMBLE1;
                break;
            }
            nova_msg.header.data[nova_msg.read] = temp;
            nova_msg.read++;
            if (nova_msg.read >= 24) {
                nova_msg.nova_state = nova_msg_parser::UMDATA;
            }
            break;
        case nova_msg_parser::UMDATA:
            if (nova_msg.read >= sizeof(nova_msg.data)) {
                Debug("parse data overflow length=%u msglength=%u\n", (unsigned)nova_msg.read,nova_msg.header.um_headeru.messagelength);
                nova_msg.nova_state = nova_msg_parser::PREAMBLE1;
                break;
            }
            nova_msg.data.bytes[nova_msg.read - 24] = temp;
            nova_msg.read++;
            if (nova_msg.read >= (nova_msg.header.um_headeru.messagelength + 24)) {
                Debug("NOVA DATA exit\n");
                nova_msg.nova_state = nova_msg_parser::UMCRC1;
            }
            break;
        case nova_msg_parser::UMCRC1:
            nova_msg.crc = (uint32_t) (temp << 0);
            nova_msg.nova_state = nova_msg_parser::UMCRC2;
            break;
        case nova_msg_parser::UMCRC2:
            nova_msg.crc += (uint32_t) (temp << 8);
            nova_msg.nova_state = nova_msg_parser::UMCRC3;
            break;
        case nova_msg_parser::UMCRC3:
            nova_msg.crc += (uint32_t) (temp << 16);
            nova_msg.nova_state = nova_msg_parser::UMCRC4;
            break;
        case nova_msg_parser::UMCRC4:
            nova_msg.crc += (uint32_t) (temp << 24);
            nova_msg.nova_state = nova_msg_parser::PREAMBLE1;
            //gcs().send_text(MAV_SEVERITY_INFO,"u-crc=%ld",nova_crc);
            //nova_crc = UM_CRC32(24,(uint8_t *)&nova_msg.header.data,(uint32_t)nova_msg.header.um_headeru.messagelength, (uint8_t *)&nova_msg.data.bytes);
            nova_crc = CalculateBlockCRC32(24, (uint8_t *)&nova_msg.header.data, (uint32_t)0);
            nova_crc = CalculateBlockCRC32((uint32_t)nova_msg.header.um_headeru.messagelength, (uint8_t *)&nova_msg.data, nova_crc);
            //gcs().send_text(MAV_SEVERITY_INFO,"mslen=%d",nova_msg.header.um_headeru.messagelength);
            //for(int i=0;i<nova_msg.header.um_headeru.messagelength;i=i+10)
            //{

                //gcs().send_text(MAV_SEVERITY_INFO,"%x,%x,%x,%x,%x,%x,%x,%x,%x,%x",nova_msg.data.bytes[i+0],nova_msg.data.bytes[i+1],nova_msg.data.bytes[i+2],nova_msg.data.bytes[i+3],nova_msg.data.bytes[i+4],nova_msg.data.bytes[i+5],nova_msg.data.bytes[i+6],nova_msg.data.bytes[i+7],nova_msg.data.bytes[i+8],nova_msg.data.bytes[i+9]);
            //}
            //gcs().send_text(MAV_SEVERITY_INFO,"u-n=%ld,%ld",nova_msg.crc,nova_crc);
            
            if (nova_msg.crc == nova_crc) {
                return process_message();
                //gcs().send_text(MAV_SEVERITY_INFO,"umok");
            } else {
                Debug("crc failed");
                //gcs().send_text(MAV_SEVERITY_INFO,"umerror");
                crc_error_counter++;
            }
            break;
    }

    return false;
}

bool
AP_GPS_NOVA::process_message(void)
{
    const uint16_t messageid = nova_msg.header.nova_headeru.messageid;

    Debug("NOVA process_message messid=%u\n",messageid);

    //check_new_itow(nova_msg.header.nova_headeru.tow, nova_msg.header.nova_headeru.messagelength + nova_msg.header.nova_headeru.headerlength);
    check_new_itow(nova_msg.header.um_headeru.ms, nova_msg.header.um_headeru.messagelength + 24);
    
    if (messageid == 42) // bestpos
    {
        const bestpos &bestposu = nova_msg.data.bestposu;

        state.time_week = nova_msg.header.nova_headeru.week;
        state.time_week_ms = (uint32_t) nova_msg.header.nova_headeru.tow;
        state.last_gps_time_ms = AP_HAL::millis();

        state.location.lat = (int32_t) (bestposu.lat * (double)1e7);
        state.location.lng = (int32_t) (bestposu.lng * (double)1e7);
        state.location.alt = (int32_t) (bestposu.hgt * 100)-gps.alt_offset;
        state.have_undulation = true;
        state.undulation = bestposu.undulation;

        state.num_sats = bestposu.svsused;

        state.horizontal_accuracy = (float) ((bestposu.latsdev + bestposu.lngsdev)/2);
        state.vertical_accuracy = (float) bestposu.hgtsdev;
        state.have_horizontal_accuracy = true;
        state.have_vertical_accuracy = true;
        state.rtk_age_ms = bestposu.diffage * 1000;
        state.rtk_num_sats = bestposu.svsused;

        if (bestposu.solstat == 0) // have a solution
        {
            switch (bestposu.postype)
            {
                case 16:
                    state.status = AP_GPS::GPS_OK_FIX_3D;
                    break;
                case 17: // psrdiff
                case 18: // waas
                case 20: // omnistar
                case 68: // ppp_converg
                case 69: // ppp
                    state.status = AP_GPS::GPS_OK_FIX_3D_DGPS;
                    break;
                case 32: // l1 float
                case 33: // iono float
                case 34: // narrow float
                    state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT;
                    break;
                case 48: // l1 int
                case 49: // l1 int
                case 50: // narrow int
                    state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;
                    break;
                case 0: // NONE
                case 1: // FIXEDPOS
                case 2: // FIXEDHEIGHT
                default:
                    state.status = AP_GPS::NO_FIX;
                    break;
            }
        } else {
            state.status = AP_GPS::NO_FIX;
        }
        
        _new_position = true;
    }
    if (messageid == 2118) // bestnav
    {
        const bestnav &bestnavu = nova_msg.data.bestnavu;

        state.time_week = nova_msg.header.um_headeru.wn;
        //state.time_week_ms = (uint32_t) nova_msg.header.um_headeru.ms;
        state.last_gps_time_ms = AP_HAL::millis();

        state.location.lat = (int32_t) (bestnavu.lat * (double)1e7);
        state.location.lng = (int32_t) (bestnavu.lng * (double)1e7);
        state.location.alt = (int32_t) (bestnavu.hgt * 100)-gps.alt_offset;

        state.num_sats = bestnavu.svsused;

        state.horizontal_accuracy =norm(bestnavu.latsdev, bestnavu.lngsdev);// (float) ((bestnavu.latsdev + bestnavu.lngsdev)/4);
        state.vertical_accuracy = (float) bestnavu.hgtsdev;
        state.have_horizontal_accuracy = true;
        state.have_vertical_accuracy = true;
        state.rtk_age_ms = bestnavu.diffage * 1000;
        state.rtk_num_sats = bestnavu.svsused;

        if (bestnavu.solstat == 0) // have a solution
        {
            switch (bestnavu.postype)
            {
                case 16:
                    state.status = AP_GPS::GPS_OK_FIX_3D;
                    break;
                case 17: // psrdiff
                case 18: // waas
                case 20: // omnistar
                case 68: // ppp_converg
                case 69: // ppp
                    state.status = AP_GPS::GPS_OK_FIX_3D_DGPS;
                    break;
                case 32: // l1 float
                case 33: // iono float
                case 34: // narrow float
                    state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT;
                    break;
                case 48: // l1 int
                case 49: // l1 int
                case 50: // narrow int
                    state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;
                    break;
                case 0: // NONE
                case 1: // FIXEDPOS
                case 2: // FIXEDHEIGHT
                default:
                    state.status = AP_GPS::NO_FIX;
                    break;
            }
        } else {
            state.status = AP_GPS::NO_FIX;
        }
        state.ground_speed = (float) bestnavu.horspd;
        state.ground_course = (float) bestnavu.trkgnd;
        fill_3d_velocity();
        state.velocity.z = -(float) bestnavu.vertspd;
        state.have_vertical_velocity = true;
        
        _last_vel_time = (uint32_t) nova_msg.header.um_headeru.ms;
        state.time_week_ms= _last_vel_time;
        _new_speed = true;
        _new_position = true;
    }
    if (messageid == 99) // bestvel
    {
        const bestvel &bestvelu = nova_msg.data.bestvelu;

        state.ground_speed = (float) bestvelu.horspd;
        state.ground_course = (float) bestvelu.trkgnd;
        fill_3d_velocity();
        state.velocity.z = -(float) bestvelu.vertspd;
        state.have_vertical_velocity = true;
        
        _last_vel_time = (uint32_t) nova_msg.header.nova_headeru.tow;
        _new_speed = true;
    }

    if (messageid == 174) // psrdop
    {
        const psrdop &psrdopu = nova_msg.data.psrdopu;

        state.hdop = (uint16_t) (psrdopu.hdop*100);
        state.vdop = (uint16_t) (psrdopu.htdop*100);
        return false;
    }
    if (messageid == 954) // psrdop
    {
        const stadop &stadopu = nova_msg.data.stadopu;

        state.hdop = (uint16_t) (stadopu.hdop*100);
        state.vdop = (uint16_t) (stadopu.vdop*100);
        return false;
    }
    if (messageid == 1657 || messageid == 972) // head
    {
        const heading3a &heading3au = nova_msg.data.heading3au;
        if(heading3au.solstat == 0 )//&& heading3au.postype == 50)
        {
            state.gps_yaw = wrap_360(heading3au.heading + 180);
            state.have_gps_yaw = true;
            state.gps_yaw_accuracy =  heading3au.hdg_std;
            state.have_gps_yaw_accuracy = true;
            state.gps_yaw_time_ms = AP_HAL::millis();
            state.gps_yaw_configured = true;
        } else {
            //gcs().send_text(MAV_SEVERITY_INFO,"um head unsol");
            state.have_gps_yaw = false;
            state.have_gps_yaw_accuracy = false;
        }
        return false;
    }
    // ensure out position and velocity stay insync
    if (_new_position && _new_speed ) {
        _new_speed = _new_position = false;
        //gcs().send_text(MAV_SEVERITY_INFO,"um_t=%ld",_last_vel_time);
        return true;
    }
    
    return false;
}

#define CRC32_POLYNOMIAL 0xEDB88320L
uint32_t AP_GPS_NOVA::CRC32Value(uint32_t icrc)
{
    int i;
    uint32_t crc = icrc;
    for ( i = 8 ; i > 0; i-- ) {
        if ( crc & 1 )
            crc = ( crc >> 1 ) ^ CRC32_POLYNOMIAL;
        else
            crc >>= 1;
    }
    return crc;
}

uint32_t AP_GPS_NOVA::CalculateBlockCRC32(uint32_t length, uint8_t *buffer, uint32_t crc)
{
    while ( length-- != 0 ) {
        crc = ((crc >> 8) & 0x00FFFFFFL) ^ (CRC32Value(((uint32_t) crc ^ *buffer++) & 0xff));
    }
    return( crc );
}
#endif
