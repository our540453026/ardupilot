#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_TOFP : public AP_RangeFinder_Backend_Serial
{

public:

    struct PACKED tofp_t
        {
            // uint8_t reserved;
            // uint8_t id;
            // uint32_t System_time;
            // uint32_t dis;
            // uint8_t dis_status;
            // uint16_t signal_strength;
            // uint8_t range_precision;
            //sdam
            uint16_t reserved1;
            uint16_t reserved2;
            uint16_t reserved3;
            uint16_t dis;
            uint16_t reserved4;
            uint16_t signal_strength;
            uint16_t reserved5;

        };
    // constructor
     using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;
    static bool detect();
    // Lanbao is always 115200:
    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return 115200;
    }
   
protected:
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }   

private:

    // get a reading
    // distance returned in reading_cm
    uint8_t SendHz;
    uint16_t TOFP_crc;
    uint32_t last_time;
    //uint8_t decode( uint8_t *data);
    uint16_t crc16_modbus(uint8_t *data, uint16_t length);
    bool get_reading(float &reading_m) override;
    union PACKED TOFPbuffer {
                    uint8_t bytes[32];
                    tofp_t tofp;
                };
    struct PACKED data_msg_parser
        {
            enum
            {
                PREAMBLE1 = 0,
                PREAMBLE2,
                PREAMBLE3,
                CMD,
                PREAMBLE4,
                LENGTH1,
                LENGTH2,
                DATA,
                CRC1,
                CRC2,
            } TOFP_state;
            TOFPbuffer data;
            uint16_t crc;
            uint16_t read_count;
            uint16_t length;
            uint8_t cmd;
            uint8_t buf[32];
            
        } data_msg;
        static const uint8_t  TOFP_FRAME_HEADER1 = 0x57;
        static const uint8_t TOFP_FRAME_HEADER2 = 0x00;
        static const uint8_t TOFP_DATA_LENGHT = 15;

        /*
        crc8 from trone driver by Luis Rodrigues
        */
        uint8_t verifyCheckSum(uint8_t *data, int32_t length);
};
