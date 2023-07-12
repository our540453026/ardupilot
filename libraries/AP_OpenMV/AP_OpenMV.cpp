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

/*
   OpenMV library
*/
#include <GCS_MAVLink/GCS.h>
#include <../ArduCopter/Copter.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include "AP_OpenMV.h"
#define AP_SERIALMANAGER_OPEN_MV_BAUD         115200
#define AP_SERIALMANAGER_OPENMV_BUFSIZE_RX        256
#define AP_SERIALMANAGER_OPENMV_BUFSIZE_TX        256



extern const AP_HAL::HAL& hal;

//constructor
AP_OpenMV::AP_OpenMV()
{
    _port = NULL;
    if (_singleton != nullptr) {
           AP_HAL::panic("AP_SerialConcentrator must be singleton");
       }
       _singleton = this;
}

// init - perform require initialisation including detecting which protocol to use
bool AP_OpenMV::init(const AP_SerialManager& serial_manager)
{
    // check for DEVO_DPort
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_OPEN_MV, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        // initialise uart
        _port->begin(AP_SERIALMANAGER_OPEN_MV_BAUD, AP_SERIALMANAGER_OPENMV_BUFSIZE_RX, AP_SERIALMANAGER_OPENMV_BUFSIZE_TX);
        return true;
    }
    return false;
}
int16_t AP_OpenMV::swapInt16( int16_t value)
{
    return ((value & 0x00FF) << 8) |
    ((value & 0xFF00) >> 8) ;
}
uint16_t AP_OpenMV::swapuInt16( uint16_t value)
{
    return ((value & 0x00FF) << 8) |
    ((value & 0xFF00) >> 8) ;
}
bool AP_OpenMV::update()
{
    if(_port == NULL)
        return false;
    int16_t numc = _port->available();
    uint8_t data;
    for(int16_t i=0;i<numc;i++)
    {
        data = _port->read();
        parse(data);
    }
    //gcs().send_text(MAV_SEVERITY_INFO,"openmv_num=%d",numc);
    update_disconnect();
    return false;
}
bool AP_OpenMV::parse(uint8_t temp)
{
    
    switch(data_msg.rec_state)
    {
        case data_msg_parser::PREAMBLE1:
                if (temp == 0xAB)
                {
                    data_msg.rec_state = data_msg_parser::PREAMBLE2;
                    data_msg.read_count = 0;
                }    
                break;
        case data_msg_parser::PREAMBLE2:
                    if (temp == 0xCB)
                    {
                        data_msg.rec_state = data_msg_parser::LENGTH1;
                        g_crc_val = 0;
                    }
                    else
                    {
                        if (temp == 0xAB)
                        {
                            data_msg.rec_state = data_msg_parser::PREAMBLE2;
                        }
                        else
                        {
                            data_msg.rec_state = data_msg_parser::PREAMBLE1;
                        }
                        
                    }
                    break;
                case data_msg_parser::LENGTH1:
                    data_msg.rec_state = data_msg_parser::LENGTH2;
                    data_msg.length = temp;
                    data_msg.length <<=8;
                    g_crc_val += temp;
                    break;
                case data_msg_parser::LENGTH2:
                    data_msg.rec_state = data_msg_parser::COMPTYPE;
                    data_msg.length += temp;
                    g_crc_val += temp;
                    if(data_msg.length >= 256) //错误处理
                    {                     
                        data_msg.rec_state = data_msg_parser::PREAMBLE1;
                    }
                    break;
                case data_msg_parser::COMPTYPE:
                    data_msg.comptype = temp;
                    data_msg.rec_state = data_msg_parser::MSGID1;
                    g_crc_val += temp;
                    break;
                case data_msg_parser::MSGID1:
                    data_msg.msgid = temp;
                    data_msg.msgid <<= 8;
                    data_msg.rec_state = data_msg_parser::MSGID2;
                    g_crc_val += temp;
                    break;
                 case data_msg_parser::MSGID2:
                    data_msg.msgid += temp;
                    data_msg.rec_state = data_msg_parser::PAYLOAD;
                    g_crc_val += temp;
                    break;
                case data_msg_parser::PAYLOAD:
                    g_crc_val += temp;
                    data_msg.data.bytes[data_msg.read_count++] = temp;
                    if (data_msg.read_count >= data_msg.length-5)
                    {
                        data_msg.rec_state = data_msg_parser::CRC1;
                    }
                    break;
                case data_msg_parser::CRC1:
                    data_msg.crc = temp;
                    data_msg.crc <<=8;
                    data_msg.rec_state = data_msg_parser::CRC2;
                    break;
                case data_msg_parser::CRC2:
                    data_msg.crc += temp;
                    data_msg.rec_state = data_msg_parser::PREAMBLE1;
                    if(g_crc_val==data_msg.crc)
                    {
                       // gcs().send_text(MAV_SEVERITY_INFO,"openmv_ok");
                        Data_Message_Process();
                        return true;
                    }
                    else
                    {
                        gcs().send_text(MAV_SEVERITY_INFO,"openmv_error len=%d,id=%x",data_msg.length,data_msg.msgid);
                        return false;
                    }
                    break;
                default:
                    data_msg.rec_state = data_msg_parser::PREAMBLE1;
                    break;
            }

            return false;
}
bool AP_OpenMV::Data_Message_Process(void)
{
    uint8_t num=0;
    switch(data_msg.msgid)
    {
        case OPENMV_STATUE_MSG_ID:
            num=swapInt16(data_msg.data.openmv_u.id);
            //gcs().send_text(MAV_SEVERITY_INFO,"MV_ID1=%d,X=%d,Y=%d,R=%d",data_msg.data.openmv_u.id,data_msg.data.openmv_u.x,data_msg.data.openmv_u.y,data_msg.data.openmv_u.r);
            if(num<8)
            {
                num--;
                openmv_data[num].active=1;
                openmv_data[num].id=num;
                openmv_data[num].x=swapInt16(data_msg.data.openmv_u.x);
                openmv_data[num].y=swapInt16(data_msg.data.openmv_u.y);
                openmv_data[num].r=swapInt16(data_msg.data.openmv_u.r);
                openmv_data[num].last_update_time= AP_HAL::millis();
                gcs().send_text(MAV_SEVERITY_INFO,"MV_ID=%d,X=%d,Y=%d,R=%d",openmv_data[num].id+1,openmv_data[num].x,openmv_data[num].y,openmv_data[num].r);
            }
            
                       
            break;
        
        default:
            break;
    }
    return true;
}
void AP_OpenMV::update_disconnect(void)
{
    for(uint8_t i=0;i<8;i++)
    {
        if(AP_HAL::millis()- openmv_data[i].last_update_time >2000)
        {
            openmv_data[i].active=0;
            openmv_data[i].id=0;
            openmv_data[i].x=0;
            openmv_data[i].y=0;
            openmv_data[i].r=0;
            openmv_data[i].last_update_time = AP_HAL::millis();
        }
    }
    
}
AP_OpenMV *AP_OpenMV::_singleton;

namespace AP {

AP_OpenMV *openmv()
{
    return AP_OpenMV::get_singleton();
}

}
