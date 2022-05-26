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
#include "AP_Compass_Serial.h"

#if HAL_MSP_COMPASS_ENABLED

AP_Compass_Serial::AP_Compass_Serial(uint8_t _serial_instance)
{
    serial_instance = _serial_instance;

    auto devid = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL, 0, _serial_instance, 0);
    register_compass(devid, instance);

    set_dev_id(instance, devid);
    set_external(instance, true);
}

void AP_Compass_Serial::handle_serial(int16_t mag_x,int16_t mag_y,int16_t mag_z )
{
    Vector3f field(mag_x, mag_y, mag_z);
    accumulate_sample(field, instance);
    //gcs().send_text(MAV_SEVERITY_INFO,"magx=%d,y=%d,z=%d,s=%d", mag_x,mag_y,mag_z,instance);
}

void AP_Compass_Serial::read(void)
{
    //gcs().send_text(MAV_SEVERITY_INFO,"magxread");
    drain_accumulated_samples(instance);
}

#endif // HAL_MSP_COMPASS_ENABLED

