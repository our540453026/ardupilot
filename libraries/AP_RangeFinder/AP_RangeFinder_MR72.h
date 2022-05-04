#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_MR72 : public AP_RangeFinder_Backend
{

public:

    struct PACKED three_sector_t
        {
            uint16_t header;
            uint16_t sector_2;
            uint16_t sector_3;
            uint16_t sector_90deg;
            uint16_t sector_135deg;
            uint16_t sector_180deg;
            uint16_t sector_225deg;
            uint16_t sector_270deg;
            uint16_t sector_1;

        };
    // constructor
    AP_RangeFinder_MR72(RangeFinder::RangeFinder_State &_state,
                            AP_RangeFinder_Params &_params,
                            uint8_t serial_instance );

    // static detection function
    static bool detect(uint8_t serial_instance);

    // update state
    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    // get a reading
    // distance returned in reading_cm
    uint8_t SendHz;
    int16_t swapInt16( int16_t value);
    bool get_reading(uint16_t &reading_cm);
    AP_HAL::UARTDriver *uart = nullptr;
    union PACKED mr72buffer {
                    three_sector_t three_sec;
                    uint8_t bytes[19];
                };
    struct PACKED data_msg_parser
        {
            enum
            {
                PREAMBLE1 = 0,
                PREAMBLE2,
                DATA,
                CRC1,
            } mr72_state;
            mr72buffer data;
            uint8_t crc;
            uint16_t read_count;
        } data_msg;
        static const uint8_t  MR72_FRAME_HEADER1 = 0x54;
        static const uint8_t MR72_FRAME_HEADER2 = 0x48;
        static const uint8_t MR72_DATA_LENGHT = 0x12;

        /*
        crc8 from trone driver by Luis Rodrigues
        */
        uint8_t crc_crc8(const uint8_t *p, uint8_t len);
};
