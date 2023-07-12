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
#include "AP_RangeFinder_MR72.h"
#include <AP_Math/crc.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;
#define MR72_DIST_MAX_CM 65536
#define MR72_OUT_OF_RANGE_ADD_CM 100
/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
// distance returned in reading_cm, signal_ok is set to true if sensor reports a strong signal
bool AP_RangeFinder_MR72::get_reading(float &reading_m)
{
    uint8_t c;
    float sum_cm = 0;
    uint16_t count = 0;
    if (uart == nullptr) {
        return false;
    }
    // read any available lines from the lidar
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        c = uart->read();
        // if buffer is empty and this byte is 0x59, add to buffer
        switch(data_msg.mr72_state)
        {
            case data_msg_parser::PREAMBLE1:
                    if (c == MR72_FRAME_HEADER1)
                        data_msg.mr72_state = data_msg_parser::PREAMBLE2;
                    data_msg.read_count = 0;
                    data_msg.data.bytes[data_msg.read_count++] = c;
                    break;
            case data_msg_parser::PREAMBLE2:
                    if (c == MR72_FRAME_HEADER2)
                    {
                        data_msg.mr72_state = data_msg_parser::DATA;
                        data_msg.data.bytes[data_msg.read_count++] = c;
                    }
                    else
                    {
                        data_msg.mr72_state = data_msg_parser::PREAMBLE1;
                    }
                    break;
                case data_msg_parser::DATA:
//                    if (data_msg.read_count >= MR72_DATA_LENGHT) {
//                        data_msg.mr72_state = data_msg_parser::PREAMBLE1;
//                        break;
//                    }
                    data_msg.data.bytes[data_msg.read_count++] = c;
                    if (data_msg.read_count >= MR72_DATA_LENGHT)
                    {
                        data_msg.mr72_state = data_msg_parser::CRC1;
                    }
                    break;
                case data_msg_parser::CRC1:

                    if((uint8_t)crc_crc8(data_msg.data.bytes,MR72_DATA_LENGHT)==c)
                    {
                        //reading_cm = swapInt16(data_msg.data.three_sec.sector_2);
                        uint16_t dist = swapInt16(data_msg.data.three_sec.sector_2);
                        sum_cm += dist;
                        count++;
                        //return true;
                    }
                    else
                    {
                        //gcs().send_text(MAV_SEVERITY_CRITICAL,"secerror=%d",state.distance_cm);
                    }
                    data_msg.mr72_state = data_msg_parser::PREAMBLE1;
                    break;
                default:
                    data_msg.mr72_state = data_msg_parser::PREAMBLE1;
                    break;
        }

    }
    if (count > 0) {
        // return average distance of readings
        reading_m = (sum_cm * 0.01f) / count;
        return true;
    }
    
    return false; // no readings so return false
}
/* 
   update the state of the sensor
*/
bool AP_RangeFinder_MR72::detect()
{
    // Assume that if the user set the RANGEFINDER_TYPE parameter to MSP,
    // there is an attached MSP rangefinder
    return true;
}

uint8_t AP_RangeFinder_MR72::crc_crc8(const uint8_t *p, uint8_t len)
{
    const uint8_t crc8_table[256] = {
            0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a,
            0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53,
            0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4,
            0xc3, 0xca, 0xcd, 0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
            0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1,
            0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88,
            0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a, 0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f,
            0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
            0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b,
            0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4, 0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2,
            0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75,
            0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
            0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34, 0x4e, 0x49, 0x40,
            0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39,
            0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae,
            0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
            0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3};
    uint16_t i;
    uint16_t crc = 0x0;
    while (len--) {
    i = (crc ^ *p++) & 0xFF;
    crc = (crc8_table[i] ^ (crc << 8)) & 0xFF;
    }
    return crc & 0xFF;
}
int16_t AP_RangeFinder_MR72::swapInt16( int16_t value)
{
    return ((value & 0x00FF) << 8) |
    ((value & 0xFF00) >> 8) ;
}

