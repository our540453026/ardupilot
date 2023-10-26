/*

   Inspired by work done here
   https://github.com/PX4/Firmware/tree/master/src/drivers/frsky_telemetry from Stefan Rado <px4@sradonia.net>
   https://github.com/opentx/opentx/tree/2.3/radio/src/telemetry from the OpenTX team

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

/* 
   FRSKY Telemetry library
*/
#include <GCS_MAVLink/GCS.h>
#include <../ArduCopter/Copter.h>
#include "AP_SerialCompass.h"
#define AP_SerialCompass_BAUD           115200
#define AP_SerialCompass_BUFSIZE_RX     256
#define AP_SerialCompass_BUFSIZE_TX     256
#define IST8310_RESOLUTION 0.3

static const int16_t IST8310_MAX_VAL_XY = (1600 / IST8310_RESOLUTION) + 1;
static const int16_t IST8310_MIN_VAL_XY = -IST8310_MAX_VAL_XY;
static const int16_t IST8310_MAX_VAL_Z  = (2500 / IST8310_RESOLUTION) + 1;
static const int16_t IST8310_MIN_VAL_Z  = -IST8310_MAX_VAL_Z;
extern const AP_HAL::HAL& hal;

AP_SerialCompass::AP_SerialCompass()
{
   _port = NULL;
   if (_singleton != nullptr) {
           AP_HAL::panic("AP_SerialCompass must be singleton");
       }
       _singleton = this;
       auto &sm = AP::serialmanager();
       port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_Compass, 0);
}

int32_t AP_SerialCompass::swapInt32( int32_t value)
{
    return ((value & 0x000000FF) << 24) |
    ((value & 0x0000FF00) << 8) |
    ((value & 0x00FF0000) >> 8) |
    ((value & 0xFF000000) >> 24) ;
}
int16_t AP_SerialCompass::swapInt16( int16_t value)
{
    return ((value & 0x00FF) << 8) |
    ((value & 0xFF00) >> 8) ;
}
/*
 * init - perform required initialisation
 */
bool AP_SerialCompass::init(const AP_SerialManager &serial_manager)
{

    //const AP_SerialManager &serial_manager = AP::serialmanager();
    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)

    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Compass, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _port->begin(AP_SerialCompass_BAUD, AP_SerialCompass_BUFSIZE_RX, AP_SerialCompass_BUFSIZE_TX);
        return true;
    }
    return false;
}
int8_t AP_SerialCompass::get_port(void)
{
    if (_port ==NULL) {
        return -1;
    }
    return port_num;
};
bool AP_SerialCompass::update(void)
{

    if(_port ==NULL)
        return false;
    _last_update_time = AP_HAL::micros();
    int16_t numc = _port->available();
    uint8_t data;
    //gcs().send_text(MAV_SEVERITY_INFO,"compdata: %d", numc);
    for(int16_t i=0;i<numc;i++)
    {
        data = _port->read();
        parse(data);
    }

    update_disconnect();
    return true;
}

void AP_SerialCompass::send_data(uint8_t *buffer,uint8_t size)
{
    if(_port !=NULL)
    _port->write(buffer, size);
}
void AP_SerialCompass::update_disconnect(void)
{
    if((_last_update_time > mag_state.time_us) && ((_last_update_time - mag_state.time_us) >3000000))
    {
          memset(&mag_state, 0, sizeof(mag_state));

    }
}
uint8_t AP_SerialCompass::crcSum(uint8_t *dat,uint8_t n)
{
    uint8_t bcc_result=0;
    uint16_t i;
    for(i=0;i<n;i++)
    {
        bcc_result^=*(dat++);
    } 
    return bcc_result;
}
bool AP_SerialCompass::parse(uint8_t temp)
{
    
    switch(data_msg.rec_state)
    {
        case data_msg_parser::PREAMBLE1:
                if (temp == DATA_PREAMBLE1)
                {
                    data_msg.rec_state = data_msg_parser::MSGID;
                    data_msg.read_count = 0;
                    data_msg.data.bytes[data_msg.read_count++] = temp;
                }    
                break;
                case data_msg_parser::MSGID:
                    data_msg.msgid = temp;
                    data_msg.rec_state = data_msg_parser::LENGTH1;
                    data_msg.data.bytes[data_msg.read_count++] = temp;
                    //if(data_msg.msgid != ID_5883 || data_msg.msgid != ID_8310) data_msg.rec_state = data_msg_parser::PREAMBLE1;
                    break;
                case data_msg_parser::LENGTH1:
                    data_msg.rec_state = data_msg_parser::PAYLOAD;
                    data_msg.length = temp;
                    data_msg.data.bytes[data_msg.read_count++] = temp;
                    if(data_msg.length > 8) data_msg.rec_state = data_msg_parser::PREAMBLE1;
                    break;
                case data_msg_parser::PAYLOAD:
                    data_msg.data.bytes[data_msg.read_count++] = temp;
                    if (data_msg.read_count-3 >= data_msg.length)
                    {
                        data_msg.rec_state = data_msg_parser::CRC1;
                    }
                    break;
                case data_msg_parser::CRC1:
                    data_msg.crc = temp;
                    data_msg.rec_state = data_msg_parser::PREAMBLE1;
                    g_crc_val = crcSum(data_msg.data.bytes,data_msg.read_count);
                    if(g_crc_val==data_msg.crc)
                    {
                        //gcs().send_text(MAV_SEVERITY_CRITICAL,"com_ok");
                        Data_Message_Process();
                        return true;
                    }
                    else
                    {
                        //gcs().send_text(MAV_SEVERITY_CRITICAL,"com_error");
                        return false;
                    }
                    break;
                default:
                    data_msg.rec_state = data_msg_parser::PREAMBLE1;
                    break;
            }

            return false;
}
 
bool AP_SerialCompass::Data_Message_Process(void)
{
    const float range_scale = 1000.0f / 3000.0f;
    mag_state.time_us = AP_HAL::micros(); 
    if(data_msg.data.mag.id == ID_5883)
    {
        mag_state.magX = -static_cast<int16_t>(swapInt16(data_msg.data.mag.mag_x));   
        mag_state.magY = static_cast<int16_t>(swapInt16(data_msg.data.mag.mag_y));
        mag_state.magZ = -static_cast<int16_t>(swapInt16(data_msg.data.mag.mag_z)); 
        AP::compass().handle_serial(mag_state.magX*range_scale,mag_state.magY*range_scale,mag_state.magZ*range_scale); 
    }
    else
    {
        if(data_msg.data.mag.id == ID_8310)
        {
            mag_state.magX = static_cast<int16_t>(swapInt16(data_msg.data.mag.mag_x));   
            mag_state.magY = static_cast<int16_t>(swapInt16(data_msg.data.mag.mag_y));
            mag_state.magZ = static_cast<int16_t>(swapInt16(data_msg.data.mag.mag_z)); 
            if (mag_state.magX > IST8310_MAX_VAL_XY || mag_state.magX < IST8310_MIN_VAL_XY ||
                mag_state.magY > IST8310_MAX_VAL_XY || mag_state.magY < IST8310_MIN_VAL_XY ||
                mag_state.magZ > IST8310_MAX_VAL_Z  || mag_state.magZ < IST8310_MIN_VAL_Z) {
                return true;
            }
            mag_state.magZ = -mag_state.magZ;
            AP::compass().handle_serial(mag_state.magX* 3.0f,mag_state.magY* 3.0f,mag_state.magZ* 3.0f); 
        }

    }
     
    return true;
}

AP_SerialCompass *AP_SerialCompass::_singleton;

namespace AP {

AP_SerialCompass *serialCompass()
{
    return AP_SerialCompass::get_singleton();
}

}
