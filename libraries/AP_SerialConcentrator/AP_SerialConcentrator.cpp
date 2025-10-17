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
//#include <../ArduCopter/Copter.h>
#include "AP_SerialConcentrator.h"
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/AP_HAL.h>
#define AP_SerialConcentrator_BAUD           115200
#define AP_SerialConcentrator_BUFSIZE_RX     256
#define AP_SerialConcentrator_BUFSIZE_TX     256

extern const AP_HAL::HAL& hal;

AP_SerialConcentrator::AP_SerialConcentrator()
{
   _port = NULL;
   //bms_state.cycle = 0;
   test_flag =0;
   gimbal_state.Cam_num=0;
   if (_singleton != nullptr) {
           AP_HAL::panic("AP_SerialConcentrator must be singleton");
       }
       _singleton = this;
   link_init();
}

uint32_t AP_SerialConcentrator::swapInt32( uint32_t value)
{
    return ((value & 0x000000FF) << 24) |
    ((value & 0x0000FF00) << 8) |
    ((value & 0x00FF0000) >> 8) |
    ((value & 0xFF000000) >> 24) ;
}
uint16_t AP_SerialConcentrator::swapInt16( uint16_t value)
{
    return ((value & 0x00FF) << 8) |
    ((value & 0xFF00) >> 8) ;
}
uint16_t AP_SerialConcentrator::swapuInt16( uint16_t value)
{
    return ((value & 0x00FF) << 8) |
    ((value & 0xFF00) >> 8) ;
}
float AP_SerialConcentrator::swap_fl_bits(float value) 
{
		int32_t val;
    int32_t bits = *(int32_t*)&value; // 将float转换为unsigned int
		
		
    // 将高位对调到低位
		val = swapInt32(bits);
    // 将unsigned int转换回float并返回
    return *(float*)&val;
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
    
    int16_t numc = _port->available();
    uint8_t data;
    /*dug++;
    if(dug>10)
    {
        dug = 0;
        gcs().send_text(MAV_SEVERITY_INFO,"serial: %d,t=%ld", numc,AP_HAL::micros());
        gimbal_state.id = 12345678;
        gimbal_state.cam_type=0;
        gimbal_state.cam_cmd ++;
        gimbal_state.gim_cmd = 0x04;
        gimbal_state.pitch += 5;
        if(gimbal_state.cam_cmd >5)
        {

            gimbal_state.cam_cmd=1;
        }
        if(gimbal_state.pitch>=90)
        {
                gimbal_state.pitch = -90;
        }
        gimbal_state.yaw += 5;
        if(gimbal_state.yaw>=90)
        {
                gimbal_state.yaw = -90;
        }
        gimbal_state.roll = 0x0123;
        gimbal_state.cam_param = 0x0203;
        send_gimbal_data(gimbal_state);
    }*/
    for(int16_t i=0;i<numc;i++)
    {
        data = _port->read();
        parse(data);
        //EnMesQueue(&usart_rx_fifo,data);
    }
    if(numc) _last_update_time = AP_HAL::micros();
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
    resend_gimbal_hz++;
    if(resend_gimbal_hz>5)
    {
        resend_gimbal_hz = 0;
        if(gimbal_state.cam_cmd || gimbal_state.gim_cmd)
        {
            send_gimbal_data(gimbal_state);
        } 
        /*if(gimbal_state.Cam_num <200 && test_flag) 
        {
            gimbal_state.Cam_num++;
            gimbal_state.cam_cmd=1;
            gimbal_state.gim_cmd=1;
            gimbal_state.pitch++;
            if(gimbal_state.pitch >90)gimbal_state.pitch=0;           
            send_gimbal_data(gimbal_state);
        }*/
    }
    return true;
}
void AP_SerialConcentrator::send_data(uint8_t *buffer,uint8_t size)
{
    if(_port !=NULL)
    _port->write(buffer, size);
}
void AP_SerialConcentrator::send_gimbal_data(gimbal_t gimbal)
{
    uint8_t cam_recoder_on[17]={0xAB ,0xCB ,0x00 ,0x0B ,0x01 ,0x02 ,0x02 ,0x01 ,0x02 ,0x03 ,0x04 ,0x03 ,0x08 ,0x00 ,0x25 ,0xEE ,0xFF};
    uint8_t cam_recoder_off[17]={0xAB ,0xCB ,0x00 ,0x0b ,0x03 ,0x02 ,0x02 ,0x01 ,0x02 ,0x03 ,0x04 ,0x03 ,0x09 ,0x00 ,0x28 ,0xEE ,0xFF};
    //uint8_t cam_shutter[17]={0xAB ,0xCB ,0x00 ,0x0b ,0x03 ,0x02 ,0x02 ,0x01 ,0x02 ,0x03 ,0x04 ,0x01 ,0x06 ,0x00 ,0x23 ,0xEE ,0xFF };
    uint8_t buf[40];
	uint8_t n;
    uint16_t gim_crc;
	n = 0;
    buf[n++] =0xAB;
    buf[n++] =0xCB;
    buf[n++] =0x00;
    buf[n++] =0x19;
	buf[n++] = UAV_TYPE;
	buf[n++] = (uint8_t)(CAMERA_GIM_CMD_MSG_ID>>8);
	buf[n++] = (uint8_t) CAMERA_GIM_CMD_MSG_ID;
	buf[n++] = (uint8_t)(gimbal.id>>24);
	buf[n++] = (uint8_t)(gimbal.id>>16);
	buf[n++] = (uint8_t)(gimbal.id>>8);
	buf[n++] = (uint8_t)(gimbal.id);
	buf[n++] = gimbal.cam_type;
	buf[n++] = gimbal.cam_cmd;
	buf[n++] = (uint8_t)(gimbal.cam_param>>8);
	buf[n++] = (uint8_t)(gimbal.cam_param);
	buf[n++] = (uint8_t)(gimbal.gim_cmd>>8);
    buf[n++] = (uint8_t)gimbal.gim_cmd;
	buf[n++] = ((uint8_t*)&gimbal.pitch)[3];//(uint8_t)(gimbal.pitch>>24);
    buf[n++] = ((uint8_t*)&gimbal.pitch)[2];//(uint8_t)(gimbal.pitch>>16);
    buf[n++] = ((uint8_t*)&gimbal.pitch)[1];//(uint8_t)(gimbal.pitch>>8);
	buf[n++] = ((uint8_t*)&gimbal.pitch)[0];//(uint8_t)(gimbal.pitch);
	buf[n++] = ((uint8_t*)&gimbal.roll)[3];//(uint8_t)(gimbal.roll>>24);
    buf[n++] = ((uint8_t*)&gimbal.roll)[2];//(uint8_t)(gimbal.roll>>26);
    buf[n++] = ((uint8_t*)&gimbal.roll)[1];//(uint8_t)(gimbal.roll>>8);
	buf[n++] = ((uint8_t*)&gimbal.roll)[0];//(uint8_t)(gimbal.roll);
    buf[n++] = ((uint8_t*)&gimbal.yaw)[3];//(uint8_t)(gimbal.yaw>>24);
    buf[n++] = ((uint8_t*)&gimbal.yaw)[2];//(uint8_t)(gimbal.yaw>>16);
    buf[n++] = ((uint8_t*)&gimbal.yaw)[1];//(uint8_t)(gimbal.yaw>>8);
	buf[n++] = ((uint8_t*)&gimbal.yaw)[0];//(uint8_t)(gimbal.yaw);
    buf[n++] = (uint8_t)(gimbal.Cam_num>>8);
	buf[n++] = (uint8_t)(gimbal.Cam_num);
    buf[n++] = (uint8_t)(gimbal.Recoder_num>>8);
	buf[n++] = (uint8_t)(gimbal.Recoder_num);
    gim_crc = crcSum(&buf[2],n-2);
    buf[n++] = (uint8_t)(gim_crc>>8);
	buf[n++] = (uint8_t)(gim_crc);
    if(_port !=NULL)
    {
        _port->write(buf, n);
        switch(gimbal.cam_cmd)
        {
            case 1:
            //    _port->write(cam_shutter, 17);               
                break;
            case 4:
                _port->write(cam_recoder_on, 17);
                break;
            case 5:
                _port->write(cam_recoder_off, 17);
                break;
            default:
                break;
        }
        
        
    }
    

    //gcs().send_text(MAV_SEVERITY_INFO,"gimbalok=%d",gimbal.pitch);
}
void AP_SerialConcentrator::send_camare_ctl(uint8_t cmd, uint32_t id,uint32_t param)
{
    uint8_t buf[100];
	uint8_t n;
    uint16_t gim_crc;
	n = 0;
    buf[n++] =0xAB;
    buf[n++] =0xCB;
    buf[n++] =0x00;
    buf[n++] =0x18;
	buf[n++] = UAV_TYPE;
    buf[n++] = (uint8_t)(COMP_PASSTHROUGH_MSG_ID>>8);
	buf[n++] = (uint8_t) COMP_PASSTHROUGH_MSG_ID;
    buf[n++] =0xAB;
    buf[n++] =0xCB;
    buf[n++] =0x00;
    buf[n++] =0x0F;
	buf[n++] = CAMERA_TYPE;
	buf[n++] = (uint8_t)(CAMERA_GIM_CMD2_MSG_ID>>8);
	buf[n++] = (uint8_t) CAMERA_GIM_CMD2_MSG_ID;
	buf[n++] = (uint8_t)(id>>24);
	buf[n++] = (uint8_t)(id>>16);
	buf[n++] = (uint8_t)(id>>8);
	buf[n++] = (uint8_t)(id);
    buf[n++] = 0x04;
    buf[n++] = cmd;
	buf[n++] = (uint8_t)(param>>24);
	buf[n++] = (uint8_t)(param>>16);
	buf[n++] = (uint8_t)(param>>8);
	buf[n++] = (uint8_t)(param);
    gcs().send_text(MAV_SEVERITY_INFO,"camare_ctl:%d,%d,%ld", cmd,buf[n-1],param);
    gim_crc = crcSum(&buf[9],15);
    buf[n++] = (uint8_t)(gim_crc>>8);
	buf[n++] = (uint8_t)(gim_crc);
    gim_crc = crcSum(&buf[2],n-2);
    buf[n++] = (uint8_t)(gim_crc>>8);
	buf[n++] = (uint8_t)(gim_crc);
    if(_port !=NULL)
    {
        _port->write(buf, 28);
        _port->write(buf, 28);
        _port->write(buf, 28);
        
    }
}
void AP_SerialConcentrator::send_camare_record_on(void)
{
    uint8_t buf[]={0xAB ,0xCB ,0x00 ,0x18 ,0x01 ,0x0F ,0x07 ,0xAB ,0xCB ,0x00 ,0x0F ,0x00 ,0x02 ,0x08 ,0x00 ,0x00 ,0x00 ,0x01 ,0x01 ,0x08 ,0x00 ,0x00 ,0x00 ,0x02 ,0x00 ,0x25,0x01 ,0xEF};

    if(_port !=NULL)
    {
        _port->write(buf, 28);
        _port->write(buf, 28);
        _port->write(buf, 28);
        gcs().send_text(MAV_SEVERITY_INFO,"record_on");
    }
}
void AP_SerialConcentrator::send_camare_record_off(void)
{
    uint8_t buf[]={0xAB ,0xCB ,0x00 ,0x18 ,0x01 ,0x0F ,0x07 ,0xAB ,0xCB ,0x00 ,0x0F ,0x00 ,0x02 ,0x08 ,0x00 ,0x00 ,0x00 ,0x01 ,0x01 ,0x09 ,0x00 ,0x00 ,0x00 ,0x02 ,0x00 ,0x26,0x01 ,0xF1};

    if(_port !=NULL)
    {
        _port->write(buf, 28);
        _port->write(buf, 28);
        _port->write(buf, 28);
        gcs().send_text(MAV_SEVERITY_INFO,"record_off");
    }
}
void AP_SerialConcentrator::send_gimbal_st_data(gimbal_st_t gimbal)
{
    
    //uint8_t cam_shutter[17]={0xAB ,0xCB ,0x00 ,0x0b ,0x03 ,0x02 ,0x02 ,0x01 ,0x02 ,0x03 ,0x04 ,0x01 ,0x06 ,0x00 ,0x23 ,0xEE ,0xFF };
    uint8_t buf[40];
	uint8_t n;
    uint16_t gim_crc;
	n = 0;
    buf[n++] =0xAB;
    buf[n++] =0xCB;
    buf[n++] =0x00;
    buf[n++] =0x15;
	buf[n++] = UAV_TYPE;
	buf[n++] = (uint8_t)(CAMERA_GIM_ST_MSG_ID>>8);
	buf[n++] = (uint8_t) CAMERA_GIM_ST_MSG_ID;
	buf[n++] = (uint8_t)(gimbal.id>>24);
	buf[n++] = (uint8_t)(gimbal.id>>16);
	buf[n++] = (uint8_t)(gimbal.id>>8);
	buf[n++] = (uint8_t)(gimbal.id);
	buf[n++] = ((uint8_t*)&gimbal.pitch)[3];//(uint8_t)(gimbal.pitch>>24);
    buf[n++] = ((uint8_t*)&gimbal.pitch)[2];//(uint8_t)(gimbal.pitch>>16);
    buf[n++] = ((uint8_t*)&gimbal.pitch)[1];//(uint8_t)(gimbal.pitch>>8);
	buf[n++] = ((uint8_t*)&gimbal.pitch)[0];//(uint8_t)(gimbal.pitch);
	buf[n++] = ((uint8_t*)&gimbal.roll)[3];//(uint8_t)(gimbal.roll>>24);
    buf[n++] = ((uint8_t*)&gimbal.roll)[2];//(uint8_t)(gimbal.roll>>26);
    buf[n++] = ((uint8_t*)&gimbal.roll)[1];//(uint8_t)(gimbal.roll>>8);
	buf[n++] = ((uint8_t*)&gimbal.roll)[0];//(uint8_t)(gimbal.roll);
    buf[n++] = ((uint8_t*)&gimbal.yaw)[3];//(uint8_t)(gimbal.yaw>>24);
    buf[n++] = ((uint8_t*)&gimbal.yaw)[2];//(uint8_t)(gimbal.yaw>>16);
    buf[n++] = ((uint8_t*)&gimbal.yaw)[1];//(uint8_t)(gimbal.yaw>>8);
	buf[n++] = ((uint8_t*)&gimbal.yaw)[0];//(uint8_t)(gimbal.yaw);
    gim_crc = crcSum(&buf[2],n-2);
    buf[n++] = (uint8_t)(gim_crc>>8);
	buf[n++] = (uint8_t)(gim_crc);
    if(_port !=NULL)
    {
        _port->write(buf, n);
        
    }
    

    //gcs().send_text(MAV_SEVERITY_INFO,"pitch_byte=%x,%x,%x,%x",buf[11],buf[12],buf[13],buf[14]);
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
    if((_last_update_time > bms_state.time_s) && ((_last_update_time - bms_state.time_s) >10000000))
    {
           
          memset(&bms_state, 0, sizeof(bms_state));
          //gcs().send_text(MAV_SEVERITY_INFO,"bmsupdate3=%ld,%ld",_last_update_time,bms_state.time_s); 
          bms_state.time_s = _last_update_time +10000000; 
    }
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
    if(AP_HAL::millis()- _avoid_last_update_time >2000)
    {
        all_avoid_u.front_cm=0;
        all_avoid_u.back_cm=0;
        all_avoid_u.left_cm=0;
        all_avoid_u.right_cm=0;
        all_avoid_u.up_cm=0;
        all_avoid_u.down_cm=0;
        _avoid_last_update_time = AP_HAL::millis();
    }
}
uint16_t AP_SerialConcentrator::crcSum(uint8_t *dat,uint8_t n)
{
    uint8_t i;
    uint16_t crc = 0 ;
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
                    if(data_msg.length >= 250) //错误处理
                    {
                        gcs().send_text(MAV_SEVERITY_INFO,"bms_error len=%d,id=%x",data_msg.length,data_msg.msgid);
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
                        //gcs().send_text(MAV_SEVERITY_INFO,"bms_ok");
                        Data_Message_Process();
                        return true;
                    }
                    else
                    {
                        //gcs().send_text(MAV_SEVERITY_INFO,"bms_error len=%d,id=%x",data_msg.length,data_msg.msgid);
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
    uint16_t ack_id;
    int len; 
    int num;
    //gcs().send_text(MAV_SEVERITY_INFO,"bmsupdateok=%d",hal.gpio->read(61));
    switch(data_msg.msgid)
    {
        case UAV_ALL_AVOID_MSG_ID:
            all_avoid_u.front_cm = swapInt16(data_msg.data.all_avoid_u.front_cm);
            all_avoid_u.back_cm = swapInt16(data_msg.data.all_avoid_u.back_cm);
            all_avoid_u.left_cm = swapInt16(data_msg.data.all_avoid_u.left_cm);
            all_avoid_u.right_cm = swapInt16(data_msg.data.all_avoid_u.right_cm);
            all_avoid_u.up_cm = swapInt16(data_msg.data.all_avoid_u.up_cm);
            all_avoid_u.down_cm = swapInt16(data_msg.data.all_avoid_u.down_cm);

            len = data_msg.length-5;
            EnMesQueue(&usart_rx_fifo,0xAB);
            EnMesQueue(&usart_rx_fifo,0xCB);
            EnMesQueue(&usart_rx_fifo,(data_msg.length>>8));
            EnMesQueue(&usart_rx_fifo,data_msg.length);
            EnMesQueue(&usart_rx_fifo,data_msg.comptype);
            EnMesQueue(&usart_rx_fifo,0x00);
            EnMesQueue(&usart_rx_fifo,0x12);
            for(int i = 0; i <len;i++) 
            {
                EnMesQueue(&usart_rx_fifo,data_msg.data.bytes[i]);
            }
            EnMesQueue(&usart_rx_fifo,data_msg.crc>>8);
            EnMesQueue(&usart_rx_fifo,data_msg.crc);
            _avoid_last_update_time = AP_HAL::millis();
            delay_n++;
           if(delay_n > 10)
           {
                delay_n = 0;
              gcs().send_text(MAV_SEVERITY_INFO,"dis=%d,%d,%d,%d",all_avoid_u.front_cm,all_avoid_u.back_cm,all_avoid_u.left_cm,all_avoid_u.right_cm);
           }
            break;
        case BMS_STATUE_MSG_ID:
            bms_state.voltages = (swapInt16(data_msg.data.bms.voltages) & 0x0000FFFF);
            //bms_state.charge_count = swapInt16(data_msg.data.bms.charge_count);
            bms_state.capacity_remaining = swapInt16(data_msg.data.bms.capacity_remaining);
            bms_state.capacity_full = swapInt16(data_msg.data.bms.capacity_full);
            bms_state.current_battery = swapInt16(data_msg.data.bms.current_battery);
            // bms_state.cycle = swapInt16(data_msg.data.bms.cycle);
            // bms_state.discharge_count = swapInt16(data_msg.data.bms.discharge_count);
            bms_state.id = swapInt32(data_msg.data.bms.id);
            for(int i=0;i<15;i++)
            {
                bms_state.cell_vot[i] =swapInt16(data_msg.data.bms.cell_vot[i]);

            }
            bms_state.bat_temp = data_msg.data.bms.bat_temp1;
            bms_state.time_s = AP_HAL::micros();
            test_counter++;
            //gcs().send_text(MAV_SEVERITY_INFO,"bmsupdateok=%ld,%d",bms_state.voltages,bms_state.cell_vot[0]);
            //bool safety_pressed = hal.util->safety_switch_state();
            break;
        case BMS_STATUE2_MSG_ID:
            bms_state.voltages = (swapInt16(data_msg.data.bms2.voltages)& 0x0000FFFF)*10;
            //bms_state.charge_count = swapInt16(data_msg.data.bms.charge_count);
            bms_state.capacity_remaining = swapInt16(data_msg.data.bms2.capacity_remaining);
            bms_state.capacity_full = swapInt16(data_msg.data.bms2.capacity_full);
            bms_state.current_battery = swapInt16(data_msg.data.bms2.current_battery)*10;
            // bms_state.cycle = swapInt16(data_msg.data.bms.cycle);
            // bms_state.discharge_count = swapInt16(data_msg.data.bms.discharge_count);
            bms_state.id = swapInt32(data_msg.data.bms2.id);
            for(int i=0;i<18;i++)
            {
                bms_state.cell_vot[i] =swapInt16(data_msg.data.bms2.cell_vot[i]);

            }
            bms_state.bat_temp = data_msg.data.bms2.bat_temp1;
            bms_state.time_s = AP_HAL::micros();
            //gcs().send_text(MAV_SEVERITY_INFO,"bmsupdateok=%ld,%d,%d,%ld,%ld",bms_state.voltages,bms_state.cell_vot[0],bms_state.bat_temp,bms_state.capacity_remaining,bms_state.capacity_full);
            
            break;
        case OPENMV_STATUE_ID_MSG_ID:
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
            openmv_data[num].ipx_x_d=swapInt32(data_msg.data.openmv_u.ipx_x_d);
            openmv_data[num].ipx_y_d=swapInt32(data_msg.data.openmv_u.ipx_y_d);
            openmv_data[num].last_update_time= AP_HAL::millis();
            delay_n++;
           if(delay_n >10)
           {
                delay_n = 0;
            gcs().send_text(MAV_SEVERITY_INFO,"MV_ID=%d,X=%d,Y=%d,R=%d",openmv_data[num].id+1,openmv_data[num].x,openmv_data[num].y,openmv_data[num].r);
            }
            //gcs().send_text(MAV_SEVERITY_INFO,"MV_ID=%d,X=%d,Y=%d,R=%d,xd=%ld,yd=%ld",openmv_data[num].id+1,openmv_data[num].x,openmv_data[num].y,openmv_data[num].r,openmv_data[num].ipx_x_d,openmv_data[num].ipx_y_d);
        }
        break;
        case COMP_PASSTHROUGH_MSG_ID:
            
            len = data_msg.length-5;
            EnMesQueue(&usart_rx_fifo,0xAB);
            EnMesQueue(&usart_rx_fifo,0xCB);
            EnMesQueue(&usart_rx_fifo,(data_msg.length>>8));
            EnMesQueue(&usart_rx_fifo,data_msg.length);
            EnMesQueue(&usart_rx_fifo,data_msg.comptype);
            EnMesQueue(&usart_rx_fifo,0x0F);
            EnMesQueue(&usart_rx_fifo,0x07);
            for(int i = 0; i <len;i++) 
            {
                EnMesQueue(&usart_rx_fifo,data_msg.data.bytes[i]);
            }
            EnMesQueue(&usart_rx_fifo,data_msg.crc>>8);
            EnMesQueue(&usart_rx_fifo,data_msg.crc);
            //gcs().send_text(MAV_SEVERITY_INFO,"pass len:%x id:%x %x",data_msg.length,data_msg.data.bytes[5],data_msg.data.bytes[6]);
            break;
        case UAV_TEMP_MSG_ID:
            //delay_n++;
           // if(delay_n >2)
            {
                //delay_n = 0;
                pcbtemp[0]=data_msg.data.bytes[0];
                pcbtemp[1]=data_msg.data.bytes[1];
                pcbtemp[2]=data_msg.data.bytes[2];
                pcbtemp[3]=data_msg.data.bytes[3];
                //gcs().send_text(MAV_SEVERITY_INFO,"P900=%d,RTK1=%d,ecs1=%d,ecs2=%d",data_msg.data.bytes[0],data_msg.data.bytes[1],data_msg.data.bytes[2],data_msg.data.bytes[3]);
            }
            break;
         case CAMERA_GIM_ST_MSG_ID:
           /* gimbal_hz++;
            if(gimbal_hz >20)
            {
                gimbal_hz = 0;
                gimbal_state.yaw = swapInt16(data_msg.data.gimbal_st_u.yaw);
                gimbal_state.pitch = swapInt16(data_msg.data.gimbal_st_u.pitch);
                gimbal_state.roll = swapInt16(data_msg.data.gimbal_st_u.roll);
                gcs().send_text(MAV_SEVERITY_INFO,"yaw=%d,pitch=%d,roll=%d",gimbal_state.yaw,gimbal_state.pitch,gimbal_state.roll);
            }*/
            len = data_msg.length-5;
            EnMesQueue(&usart_rx_fifo,0xAB);
            EnMesQueue(&usart_rx_fifo,0xCB);
            EnMesQueue(&usart_rx_fifo,(data_msg.length>>8));
            EnMesQueue(&usart_rx_fifo,data_msg.length);
            EnMesQueue(&usart_rx_fifo,data_msg.comptype);
            EnMesQueue(&usart_rx_fifo,0x02);
            EnMesQueue(&usart_rx_fifo,0x0A);
            for(int i = 0; i <len;i++) 
            {
                EnMesQueue(&usart_rx_fifo,data_msg.data.bytes[i]);
            }
            EnMesQueue(&usart_rx_fifo,data_msg.crc>>8);
            EnMesQueue(&usart_rx_fifo,data_msg.crc);
            //gcs().send_text(MAV_SEVERITY_INFO,"rec:pitch=%f,roll=%f,yaw=%f",swap_fl_bits(data_msg.data.gimbal_st_u.pitch),swap_fl_bits(data_msg.data.gimbal_st_u.roll),wrap_360(swap_fl_bits(data_msg.data.gimbal_st_u.yaw)));
            test_flag=1;
            break;
        case UAV_RTK_RAW_MSG_ID:
            len = data_msg.length-5;
            EnMesQueue(&usart_rx_fifo,0xAB);
            EnMesQueue(&usart_rx_fifo,0xCB);
            EnMesQueue(&usart_rx_fifo,(data_msg.length>>8));
            EnMesQueue(&usart_rx_fifo,data_msg.length);
            EnMesQueue(&usart_rx_fifo,data_msg.comptype);
            EnMesQueue(&usart_rx_fifo,0x00);
            EnMesQueue(&usart_rx_fifo,0x0B);
            for(int i = 0; i <len;i++) 
            {
                EnMesQueue(&usart_rx_fifo,data_msg.data.bytes[i]);
            }
            EnMesQueue(&usart_rx_fifo,data_msg.crc>>8);
            EnMesQueue(&usart_rx_fifo,data_msg.crc);
            break;
        case UAV_ESC_MSG_ID:
            AP_ESC_Telem_Backend::TelemetryData  new_data;
            new_data.voltage = swapuInt16(data_msg.data.esc_st.vot1)/1000.0;
            new_data.current = swapuInt16(data_msg.data.esc_st.cur1)/1000.0;
            new_data.temperature_cdeg =pcbtemp[0]*100; //swapInt16(data_msg.data.esc_st.tem1)*100;
            update_telem_data(0,new_data,AP_ESC_Telem_Backend::TelemetryType::CURRENT
            | AP_ESC_Telem_Backend::TelemetryType::VOLTAGE
            | AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);
            update_rpm(0, swapuInt16(data_msg.data.esc_st.rpm1));

            new_data.voltage = swapuInt16(data_msg.data.esc_st.vot2)/1000.0;
            new_data.current = swapuInt16(data_msg.data.esc_st.cur2)/1000.0;
            new_data.temperature_cdeg = pcbtemp[1]*100; //swapInt16(data_msg.data.esc_st.tem2)*100;
            update_telem_data(1,new_data,AP_ESC_Telem_Backend::TelemetryType::CURRENT
            | AP_ESC_Telem_Backend::TelemetryType::VOLTAGE
            | AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);
            update_rpm(1, swapuInt16(data_msg.data.esc_st.rpm2));

            new_data.voltage = swapuInt16(data_msg.data.esc_st.vot3)/1000.0;
            new_data.current = swapuInt16(data_msg.data.esc_st.cur3)/1000.0;
            new_data.temperature_cdeg = pcbtemp[2]*100; //swapInt16(data_msg.data.esc_st.tem3)*100;
            update_telem_data(2,new_data,AP_ESC_Telem_Backend::TelemetryType::CURRENT
            | AP_ESC_Telem_Backend::TelemetryType::VOLTAGE
            | AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);
            update_rpm(2, swapuInt16(data_msg.data.esc_st.rpm3));

            new_data.voltage = swapuInt16(data_msg.data.esc_st.vot4)/1000.0;
            new_data.current = swapuInt16(data_msg.data.esc_st.cur4)/1000.0;
            new_data.temperature_cdeg = pcbtemp[3]*100; //swapInt16(data_msg.data.esc_st.tem4)*100;
            update_telem_data(3,new_data,AP_ESC_Telem_Backend::TelemetryType::CURRENT
            | AP_ESC_Telem_Backend::TelemetryType::VOLTAGE
            | AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);
            update_rpm(3, swapuInt16(data_msg.data.esc_st.rpm4));
            //gcs().send_text(MAV_SEVERITY_INFO,"v=%d,rpm=%d,%d,%d,%d",swapuInt16(data_msg.data.esc_st.vot1),swapuInt16(data_msg.data.esc_st.rpm1),swapuInt16(data_msg.data.esc_st.rpm2),swapuInt16(data_msg.data.esc_st.rpm3),swapuInt16(data_msg.data.esc_st.rpm4));
            break;
        case ACK_MSG_ID:
            len = data_msg.length-5;
            EnMesQueue(&usart_rx_fifo,0xAB);
            EnMesQueue(&usart_rx_fifo,0xCB);
            EnMesQueue(&usart_rx_fifo,(data_msg.length>>8));
            EnMesQueue(&usart_rx_fifo,data_msg.length);
            EnMesQueue(&usart_rx_fifo,data_msg.comptype);
            EnMesQueue(&usart_rx_fifo,0xfe);
            EnMesQueue(&usart_rx_fifo,0xfe);
            for(int i = 0; i <len;i++) 
            {
                EnMesQueue(&usart_rx_fifo,data_msg.data.bytes[i]);
            }
            EnMesQueue(&usart_rx_fifo,data_msg.crc>>8);
            EnMesQueue(&usart_rx_fifo,data_msg.crc);
            //gcs().send_text(MAV_SEVERITY_INFO,"ack=%02X,%02X",data_msg.data.bytes[0],data_msg.data.bytes[1]);
            Tx_urgent_statue = true;
            ack_id= swapuInt16(data_msg.data.gimbal_ack_u.ack_cmd);
            switch(ack_id)
            {
                case CAMERA_GIM_CMD_MSG_ID:
                    if(data_msg.data.gimbal_ack_u.cam_result)
                    {
                        gimbal_state.cam_cmd =0;
                        gcs().send_text(MAV_SEVERITY_INFO,"cam_ack=%d",gimbal_state.Cam_num);
                    }
                    if(data_msg.data.gimbal_ack_u.gim_result)
                    {
                        gimbal_state.gim_cmd =0;
                        gcs().send_text(MAV_SEVERITY_INFO,"gim_ack");
                    }
                    break;
                case CAMERA_CTR_MSG_ID:
                    gimbal_state.cam_cmd =0;
                    gcs().send_text(MAV_SEVERITY_INFO,"cam_ack=%d",gimbal_state.Cam_num);
                default:
                    break;

            }
            
            break;       
        default:
            len = data_msg.length-5;
            EnMesQueue(&usart_rx_fifo,0xAB);
            EnMesQueue(&usart_rx_fifo,0xCB);
            EnMesQueue(&usart_rx_fifo,(data_msg.length>>8));
            EnMesQueue(&usart_rx_fifo,data_msg.length);
            EnMesQueue(&usart_rx_fifo,data_msg.comptype);
            EnMesQueue(&usart_rx_fifo,(uint8_t) (data_msg.msgid>>8));
            EnMesQueue(&usart_rx_fifo,(uint8_t) data_msg.msgid);
            for(int i = 0; i <len;i++) 
            {
                EnMesQueue(&usart_rx_fifo,data_msg.data.bytes[i]);
            }
            EnMesQueue(&usart_rx_fifo,data_msg.crc>>8);
            EnMesQueue(&usart_rx_fifo,data_msg.crc);
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
