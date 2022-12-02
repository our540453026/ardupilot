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
#include "AP_SerialConcentrator.h"
#define AP_SerialConcentrator_BAUD           115200
#define AP_SerialConcentrator_BUFSIZE_RX     256
#define AP_SerialConcentrator_BUFSIZE_TX     256

extern const AP_HAL::HAL& hal;

AP_SerialConcentrator::AP_SerialConcentrator()
{
   _port = NULL;
   bms_state.cycle = 0;
   if (_singleton != nullptr) {
           AP_HAL::panic("AP_SerialConcentrator must be singleton");
       }
       _singleton = this;
   link_init();
}

int32_t AP_SerialConcentrator::swapInt32( int32_t value)
{
    return ((value & 0x000000FF) << 24) |
    ((value & 0x0000FF00) << 8) |
    ((value & 0x00FF0000) >> 8) |
    ((value & 0xFF000000) >> 24) ;
}
int16_t AP_SerialConcentrator::swapInt16( int16_t value)
{
    return ((value & 0x00FF) << 8) |
    ((value & 0xFF00) >> 8) ;
}
/*
 * init - perform required initialisation
 */
bool AP_SerialConcentrator::init(const AP_SerialManager &serial_manager)
{

    //const AP_SerialManager &serial_manager = AP::serialmanager();
    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)

    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Concentrator, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _port->begin(AP_SerialConcentrator_BAUD, AP_SerialConcentrator_BUFSIZE_RX, AP_SerialConcentrator_BUFSIZE_TX);
        return true;
    }
    return false;
}

bool AP_SerialConcentrator::update(void)
{
//    uint8_t test_buf[]={0xAB, 0xCB , 0x36 , 0x12 , 0x12 , 0x7A , 0x3A , 0x76 , 0xAD , 0xC3 , 0x3C , 0x10 , 0x4C , 0x10 , 0x4C , 0x10 , 0x4C , 0x10 , 0x4C , 0x10 , 0x4F , 0x10 , 0x43 ,
//                        0x10 , 0x4B , 0x10 , 0x4C , 0x10 , 0x4C , 0x10 , 0x4C , 0x10 , 0x4A , 0x10 , 0x4B , 0x00 , 0x00 , 0x00 , 0x21 , 0x00 , 0x1A , 0x1B , 0x21 , 0x1F , 0xBA , 0x00 ,
//                        0x00 , 0x0B , 0x6B , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x05 , 0x41};
    if(_port ==NULL)
        return false;
    _last_update_time = AP_HAL::micros();
    int16_t numc = _port->available();
    uint8_t data;
    //gcs().send_text(MAV_SEVERITY_CRITICAL,"serialdata: %d", numc);
    for(int16_t i=0;i<numc;i++)
    {
        data = _port->read();
        parse(data);
        //gcs().send_text(MAV_SEVERITY_CRITICAL,"serialdata: %x", data);
        EnMesQueue(&usart_rx_fifo,data);
    }

//    bms_state.cycle++;
//    if(bms_state.cycle >60)
//    {
//        for(int16_t i=0;i<58;i++)
//        {
//            if(parse(test_buf[i]))
//            {
//
//                return true;
//            }
//        }
//    }
    update_disconnect();
    return true;
}
void AP_SerialConcentrator::send_data(uint8_t *buffer,uint8_t size)
{
    _port->write(buffer, size);
}
//void AP_SerialConcentrator::update_tx_buf(void)
//{
//    EnMesQueue(&usart_rx_fifo,DATA_PREAMBLE1);
//    EnMesQueue(&usart_rx_fifo,DATA_PREAMBLE2);
//    EnMesQueue(&usart_rx_fifo,data_msg.length);
//    EnMesQueue(&usart_rx_fifo,data_msg.ID);
//    for(uint8_t i=0;i<data_msg.length-1;i++)
//    {
//        EnMesQueue(&usart_rx_fifo,data_msg.data.bytes[i]);
//
//    }
//    EnMesQueue(&usart_rx_fifo,data_msg.crc);
//}
void AP_SerialConcentrator::update_disconnect(void)
{
    if((_last_update_time > bms_state.time_s) && ((_last_update_time - bms_state.time_s) >5000000))
    {
          memset(&bms_state, 0, sizeof(bms_state));

    }
}
uint8_t AP_SerialConcentrator::crcSum(uint8_t *dat,uint8_t n)
{
    uint8_t i;
    uint8_t crc = 0 ;
    for(i=0;i<n;i++)
    {
        crc +=dat[i];
    }
    return crc;
}
bool AP_SerialConcentrator::parse(uint8_t temp)
{
    
    switch(data_msg.rec_state)
    {
        case data_msg_parser::PREAMBLE1:
                if (temp == DATA_PREAMBLE1)
                {
                    data_msg.rec_state = data_msg_parser::PREAMBLE2;
                    data_msg.read_count = 0;
                }    
                break;
        case data_msg_parser::PREAMBLE2:
                    if (temp == DATA_PREAMBLE2)
                    {
                        data_msg.rec_state = data_msg_parser::LENGTH1;
                        g_crc_val = 0;
                    }
                    else
                    {
                        if (temp == DATA_PREAMBLE1)
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
                    if(data_msg.length >= 512) //
                    {
                        data_msg.rec_state = data_msg_parser::PREAMBLE1;
                        //gcs().send_text(MAV_SEVERITY_INFO,"bms_len=%d",data_msg.length);
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
                        //gcs().send_text(MAV_SEVERITY_INFO,"bms_ok");
                        Data_Message_Process();
                        return true;
                    }
                    else
                    {
                        gcs().send_text(MAV_SEVERITY_INFO,"bms_error");
                        return false;
                    }
                    break;
                default:
                    data_msg.rec_state = data_msg_parser::PREAMBLE1;
                    break;
            }

            return false;
}
bool AP_SerialConcentrator::Data_Message_Process(void)
{
    switch(data_msg.msgid)
    {
        case BMS_STATUE_MSG_ID:
            bms_state.voltages = swapInt16(data_msg.data.bms.voltages);
            bms_state.charge_count = swapInt16(data_msg.data.bms.charge_count);
            bms_state.capacity_remaining = swapInt16(data_msg.data.bms.capacity_remaining);
            bms_state.current_battery = swapInt16(data_msg.data.bms.current_battery);
            bms_state.cycle = swapInt16(data_msg.data.bms.cycle);
            bms_state.discharge_count = swapInt16(data_msg.data.bms.discharge_count);
            bms_state.id = swapInt32(data_msg.data.bms.id);
            for(int i=0;i<15;i++)
            {
                bms_state.cell_vot[i] =swapInt16(data_msg.data.bms.cell_vot[i]);

            }
            bms_state.bat_temp1 = data_msg.data.bms.bat_temp1;
            bms_state.bat_temp2 = data_msg.data.bms.bat_temp2;
            bms_state.pcb_temp1 = data_msg.data.bms.pcb_temp1;
            bms_state.pcb_temp2 = data_msg.data.bms.pcb_temp2;
            bms_state.type = data_msg.data.bms.type;
            bms_state.time_s = AP_HAL::micros();
            //gcs().send_text(MAV_SEVERITY_INFO,"bms=%d",bms_state.voltages);
            break;
        case ACK_MSG_ID:
            Tx_urgent_statue = true;
            break;
        default:
            break;
    }
    return true;
}

void AP_SerialConcentrator::link_init(void)
{
    InitMesQueue(&usart_rx_fifo);

}
void AP_SerialConcentrator::InitMesQueue(LinkMes *L)
{
    L->front=0;
    L->rear=0;
}

char AP_SerialConcentrator::EnMesQueue(LinkMes *L,uint8_t dat)
{
    if((L->rear+1)%LINK_LENGTH == L->front)    return false;
    L->buf[L->rear]=dat;
    L->rear = (L->rear+1)%LINK_LENGTH;
    return true;

}
char AP_SerialConcentrator::DeMesQueue(LinkMes *L , uint8_t *dat)
{
    if(L->front == L->rear) return false;
    *dat=L->buf[L->front];
    L->front = (L->front + 1)%LINK_LENGTH;
    return true;
}

uint8_t AP_SerialConcentrator::usart_serial_available(uint8_t *dat)
{
    if(DeMesQueue(&usart_rx_fifo,dat)) return true;
    return false;
}
AP_SerialConcentrator *AP_SerialConcentrator::_singleton;

namespace AP {

AP_SerialConcentrator *serialConcentrator()
{
    return AP_SerialConcentrator::get_singleton();
}

}
