#pragma once

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"
#include <AP_SerialConcentrator/AP_SerialCompass.h>

//#if HAL_Serial_COMPASS_ENABLED

class AP_Compass_Serial : public AP_Compass_Backend
{
public:
    AP_Compass_Serial(uint8_t serial_instance);

    void read(void) override;

private:
    void handle_serial(int16_t mag_x,int16_t mag_y,int16_t mag_z) override;
    uint8_t serial_instance;
    uint8_t instance;
};

//#endif // HAL_MSP_COMPASS_ENABLED
