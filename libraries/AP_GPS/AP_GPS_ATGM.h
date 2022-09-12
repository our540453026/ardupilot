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

#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

#ifndef AP_GPS_ATGM_ENABLED
  #define AP_GPS_ATGM_ENABLED AP_GPS_BACKEND_DEFAULT_ENABLED
#endif

#if AP_GPS_ATGM_ENABLED
class AP_GPS_ATGM : public AP_GPS_Backend
{
public:
    AP_GPS_ATGM(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    AP_GPS::GPS_Status highest_supported_status(void) override { return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED; }

    // Methods
    bool read() override;

    const char *name() const override { return "ATGM"; }

private:

    bool parse(uint8_t temp);
    bool process_message();
    uint32_t CalculateBlockCRC(uint32_t length);

    static const uint8_t ATGM_PREAMBLE1 = 0xba;
    static const uint8_t ATGM_PREAMBLE2 = 0xce;

    // do we have new position information?
    bool            _new_position:1;
    // do we have new speed information?
    bool            _new_speed:1;
    int32_t _new_time;                                                  ///< time parsed from a term
    int32_t _new_date; 
    uint32_t        _last_vel_time;
    
    uint8_t _init_blob_index = 0;
    uint32_t _init_blob_time = 0;
    static const char* const _initialisation_blob[2];
   
    uint32_t crc_error_counter = 0;
   

    struct PACKED psrdop
    {
        uint32_t runtime;
        float pdop;
        float hdop;
        float vdop;
        float ndop;
        float edop;
        float tdop;       
    };
    struct PACKED bestsol
    {
        uint32_t runtime;
        uint8_t posValid;
        uint8_t velValid;
        uint8_t timeSrc;
        uint8_t system;
        uint8_t numSV;
        uint8_t numSVGPS;   
        uint8_t numSVBDS;
        uint8_t numSVGLN;
        uint16_t res;
        uint16_t week;
        double tow;
        double ecefX;  
        double ecefY;
        double ecefZ;   
        float pAcc;
        float ecefVX;
        float ecefVY;
        float ecefVZ;
        float sAcc;
        float  pDop;    
    };
    struct PACKED bestpv
    {
        uint32_t runTime;      ///< Solution status
        uint8_t  posValid;      ///< Position type
        uint8_t  velValid;
        uint8_t  system;
        uint8_t  numSV;
        uint8_t  numSVGPS;
        uint8_t  numSVBDS;
        uint8_t  numSVGLN;
        uint8_t  res;
        float    pdop;
        double lng;            ///< longitude (deg)
        double lat;            ///< latitude (deg)   
        float hgt;            ///< height above mean sea level (m)
        float sepGeoid;      ///< relationship between the geoid and the ellipsoid (m)
        float  hAcc;
        float  vAcc;
        float velN;
        float velE;
        float velU;         ///< latitude standard deviation (m)
        float speed3D;         ///< longitude standard deviation (m)
        float speed2D;         ///< height standard deviation (m)
        float heading;         ///< differential position age (sec)
        float  sAcc;         ///< solution age (sec)
        float  cAcc;
    };
    struct PACKED timeutc
        {
            uint32_t runTime;      ///< Solution status
            float tAcc;            ///<
            float msErr;            ///< 
            uint16_t ms;
            uint16_t year;
            uint8_t month;      ///< base station id
            uint8_t day;         ///<
            uint8_t hour;         ///
            uint8_t min;    ///< number of satellites tracked
            uint8_t sec;       ///< number of satellites used in solution
            uint8_t  valid;         ///< number of GPS plus GLONASS L1 satellites used in solution
            uint8_t timeSrc;   ///< number of GPS plus GLONASS L2 satellites used in solution
            uint8_t dateValid;          ///< reserved
        };
    
    union PACKED msgbuffer {
        bestpv bestpvu;
        timeutc timeutcu;
        psrdop psrdopu;
        bestsol bestsolu;
        uint8_t bytes[256];
        uint32_t uint32s[64];
    };
    
    struct PACKED ATGM_msg_parser
    {
        enum
        {
            PREAMBLE1 = 0,
            PREAMBLE2,
            LENGTH1,
            LENGTH2,
            MSG1,
            MSG2,
            DATA,
            CRC1,
            CRC2,
            CRC3,
            CRC4,
        } ATGM_state;
        
        msgbuffer data;
        uint32_t crc;
        uint16_t read;
        uint16_t length;
        uint16_t mesid;
    } ATGM_msg;
};
#endif
