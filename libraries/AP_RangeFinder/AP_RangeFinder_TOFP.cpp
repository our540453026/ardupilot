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

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_TOFP.h"
#include <AP_Math/crc.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>
//#include <../ArduCopter/Copter.h>
extern const AP_HAL::HAL& hal;
#define TOFP_DIST_MAX_CM 2500
#define TOFP_OUT_OF_RANGE_ADD_CM 100
/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
// distance returned in reading_cm, signal_ok is set to true if sensor reports a strong signal

/**
 * CRC-16/MODBUS校验计算函数
 * @param data 待校验数据指针
 * @param length 数据长度（字节数）
 * @return 16位CRC校验值（低字节在前）
 */
uint16_t AP_RangeFinder_TOFP::crc16_modbus(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;  // 初始化CRC寄存器
    uint16_t i, j;
    
    for (i = 0; i < length; i++) {
        crc ^= data[i];  // 当前字节与CRC低字节异或
        
        for (j = 0; j < 8; j++) {
            if (crc & 0x0001) {  // 检查LSB是否为1
                crc >>= 1;       // 右移一位
                crc ^= 0xA001;   // 与反转多项式异或（0x8005按位反转）
            } else {
                crc >>= 1;       // 直接右移
            }
        }
    }
    
    return crc;
}
bool AP_RangeFinder_TOFP::get_reading(float &reading_m)
{
    uint8_t c;
    uint8_t get_cmd[]={0xA5,0x03,0x20,0x01,0x00,0x00,0x00,0x02,0x6E};
    float sum_cm = 0;
    uint16_t count = 0;
    if (uart == nullptr) {
        return false;
    }
    // read any available lines from the lidar
    int16_t nbytes = uart->available();
    //gcs().send_text(MAV_SEVERITY_INFO,"dis=%d,%x",nbytes,crc16_modbus(get_cmd,7));
    while (nbytes-- > 0) {
        c = uart->read();
        // if buffer is empty and this byte is 0x59, add to buffer
        //gcs().send_text(MAV_SEVERITY_INFO,"dis=%x",c);
        switch(data_msg.TOFP_state)
        {
            case data_msg_parser::PREAMBLE1:
                    if (c == 0xA5)
                    {
                        data_msg.TOFP_state = data_msg_parser::PREAMBLE2;
                        data_msg.buf[0]=c;
                    }                       
                    // data_msg.read_count = 0;
                    // data_msg.data.bytes[data_msg.read_count++] = c;
                    // data_msg.crc = 0;
                    // data_msg.crc += c;
                    break;
            case data_msg_parser::PREAMBLE2:
                    if (c == 0x03)
                    {
                        data_msg.TOFP_state = data_msg_parser::PREAMBLE3;
                        data_msg.buf[1]=c;
                    }
                    else
                    {
                        data_msg.TOFP_state = data_msg_parser::PREAMBLE1;
                    }
                    break;
                case data_msg_parser::PREAMBLE3:
                    if (c == 0x20)
                    {
                        data_msg.TOFP_state = data_msg_parser::CMD;
                        data_msg.buf[2]=c;
                    }
                    else
                    {
                        data_msg.TOFP_state = data_msg_parser::PREAMBLE1;
                    }
                    break;
                case data_msg_parser::CMD:
                    data_msg.buf[3]=c;
                    data_msg.cmd = c;
                    data_msg.TOFP_state = data_msg_parser::PREAMBLE4;
                    break;
                case data_msg_parser::PREAMBLE4:
                    data_msg.buf[4]=c;
                    data_msg.cmd = c;
                    data_msg.TOFP_state = data_msg_parser::LENGTH1;
                    break;
                case data_msg_parser::LENGTH1:
                    data_msg.buf[5]=c;
                    data_msg.length = c;
                    data_msg.TOFP_state = data_msg_parser::LENGTH2;
                    break;
                case data_msg_parser::LENGTH2:
                    data_msg.read_count =0;
                    data_msg.buf[6]=c;
                    data_msg.length = (data_msg.length << 8) | c;
                    data_msg.TOFP_state = data_msg_parser::DATA;
                    break;
                case data_msg_parser::DATA:
//                    if (data_msg.read_count >= TOFP_DATA_LENGHT) {
//                        data_msg.TOFP_state = data_msg_parser::PREAMBLE1;
//                        break;
//                    }
                    
                    data_msg.data.bytes[data_msg.read_count++] = c;
                    data_msg.buf[6+data_msg.read_count]=c;
                    if (data_msg.read_count >= data_msg.length)
                    {
                        data_msg.TOFP_state = data_msg_parser::CRC1;
                    }
                    break;
                case data_msg_parser::CRC1:
                    data_msg.crc = c;
                    data_msg.TOFP_state = data_msg_parser::CRC2;
                    break;
                case data_msg_parser::CRC2:
                    data_msg.crc = (data_msg.crc << 8) | c;
                    TOFP_crc=crc16_modbus(data_msg.buf,7+data_msg.read_count);
                    //gcs().send_text(MAV_SEVERITY_INFO,"crc:%x,count=%d,dis:%d",c,6+data_msg.read_count,data_msg.data.tofp.dis);
                    if(data_msg.crc == TOFP_crc)
                    {
                        sum_cm += data_msg.data.tofp.dis;
                        count++;
                        // if(decode(data_msg.data.bytes)==1 && data_msg.tofp.range_precision < 40)
                        // {                      
                        //     sum_cm += data_msg.tofp.dis;
                        //     count++;
                        //     //gcs().send_text(MAV_SEVERITY_INFO,"disance1=%f,count=%d",reading_m,count);
                        // }
                        // else
                        // {

                        //     sum_cm = 1;
                        //     count = 1;
                        // }
                        //return true;
                    }
                    else
                    {
                        gcs().send_text(MAV_SEVERITY_INFO,"secerror=%x,%x",TOFP_crc,data_msg.crc);
                    }
                    data_msg.TOFP_state = data_msg_parser::PREAMBLE1;
                    break;
                default:
                    data_msg.TOFP_state = data_msg_parser::PREAMBLE1;
                    break;
        }

    }
    if (count > 0) {
        // return average distance of readings
        reading_m = (sum_cm * 0.001f) / count;
        //copter.tof_val = reading_m;
        //if((data_msg.tofp.System_time -last_time) >= 1000)
        //{
        //    gcs().send_text(MAV_SEVERITY_INFO,"preci=%d",data_msg.tofp.range_precision);
         //   last_time = data_msg.tofp.System_time;
        //}
        
        return true;
    }
    uart->write(get_cmd,9);
    return false; // no readings so return false
}
/* 
   update the state of the sensor
*/
bool AP_RangeFinder_TOFP::detect()
{
    // Assume that if the user set the RANGEFINDER_TYPE parameter to MSP,
    // there is an attached MSP rangefinder
    return true;
}
uint8_t AP_RangeFinder_TOFP::verifyCheckSum(uint8_t *data, int32_t length)
{ 
    uint8_t sum = 0; 
    for(int32_t i=0;i<length-1;++i)
    { sum += data[i]; }
    return sum == data[length-1]; 
}

// uint8_t AP_RangeFinder_TOFP::decode( uint8_t *data)
// {
//     // data_msg.tofp.reserved = data[2];
//     // data_msg.tofp.id= data[3]; 
//     // data_msg.tofp.System_time = data[7];
//     // data_msg.tofp.System_time <<= 8;
//     // data_msg.tofp.System_time += data[6];
//     // data_msg.tofp.System_time <<= 8;
//     // data_msg.tofp.System_time += data[5];
//     // data_msg.tofp.System_time <<= 8;
//     // data_msg.tofp.System_time += data[4];
    
//     // data_msg.tofp.dis = data[10];
//     // data_msg.tofp.dis <<= 8;
//     // data_msg.tofp.dis += data[9];
//     // data_msg.tofp.dis <<= 8;
//     // data_msg.tofp.dis += data[8];
//     // data_msg.tofp.dis_status= data[11];
//     // data_msg.tofp.signal_strength= data[12] + (data[13]<<8);
//     // data_msg.tofp.range_precision= data[14];
//     // //gcs().send_text(MAV_SEVERITY_INFO,"data=%d,%ld,%ld,%d,%d,%d",data_msg.tofp.id,data_msg.tofp.System_time,data_msg.tofp.dis,data_msg.tofp.dis_status,data_msg.tofp.signal_strength,data_msg.tofp.range_precision);
//     // return data_msg.tofp.dis_status;
// }

