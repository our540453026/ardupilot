/*
  PCA9632LED I2C driver
*/
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
#include "PCA9632LED_I2C.h"

#include <AP_HAL/AP_HAL.h>
//#include <../ArduCopter/Copter.h>
#define NAVIO_LED_BRIGHT 0x64    // full brightness
#define NAVIO_LED_MEDIUM 0x32    // medium brightness
#define NAVIO_LED_DIM    0x28    // dim brightness
#define NAVIO_LED_OFF    0x00    // off

#define PCA9632_ADDRESS                     0x62
#define REG_PCA9632_MODE1                   0x00
#define REG_PCA9632_MODE2                   0x01
#define REG_PCA9632_PWM0                    0x02
#define REG_PCA9632_PWM1                    0x03
#define REG_PCA9632_PWM2                    0x04
#define REG_PCA9632_PWM3                    0x05
#define REG_PCA9632_GRPPWM                  0x06
#define REG_PCA9632_GRPFREQ                 0x07
#define REG_PCA9632_LEDOUT                  0x08
#define REG_PCA9632_SUBADR1                 0x09
#define REG_PCA9632_SUBADR2                 0x0A
#define REG_PCA9632_SUBADR3                 0x0B
#define REG_PCA9632_ALLCALLADDR             0x0C

/*----------------------------- Modle register 1 -----------------------------*/
#define PCA9632_AUTOINC_ENABLED             0x80
#define PCA9632_AUTOINC_DISABLED            0x00

#define PCA9632_AUTOINC_MODE0               0x00
#define PCA9632_AUTOINC_MODE1               0x20
#define PCA9632_AUTOINC_MODE2               0x40
#define PCA9632_AUTOINC_MODE3               0x60

#define PCA9632_SLEEP_MODE                  0x10
#define PCA9632_ACTIVE_MODE                 0x00

#define PCA9632_RESPOND_SUBADR1             0x08
#define PCA9632_RESPOND_SUBADR2             0x04
#define PCA9632_RESPOND_SUBADR3             0x02
#define PCA9632_RESPOND_ALLCALL             0x01

/*----------------------------- Modle register 2 -----------------------------*/
#define PCA9632_GROUPCTRL_BLINKING          0x20
#define PCA9632_GROUPCTRL_DIMMING           0x00

#define PCA9632_OUTPUT_INVERT               0x10
#define PCA9632_OUTPUT_NORMAL               0x00

#define PCA9632_OUTPUT_ON_ACK               0x08
#define PCA9632_OUTPUT_ON_STOP              0x00

#define PCA9632_OUTPUT_PUSHPULL             0x04
#define PCA9632_OUTPUT_OPENDRAIN            0x00

#define PCA9632_OUTNE                       0x01
#define CHANNEL0_SHIFT                      0
#define CHANNEL1_SHIFT                      2
#define CHANNEL2_SHIFT                      4
#define CHANNEL3_SHIFT                      6

#define PCA9632_OUTPUT_OFF                  0x00
#define PCA9632_OUTPUT_FULL_ON              0x01
#define PCA9632_OUTPUT_INDIVIDUAL           0x02
#define PCA9632_OUTPUT_INDIVIDUAL_AND_GROUP 0x03
extern const AP_HAL::HAL& hal;

PCA9632LED_I2C::PCA9632LED_I2C() : 
    RGBLed(NAVIO_LED_OFF, NAVIO_LED_BRIGHT, NAVIO_LED_MEDIUM, NAVIO_LED_DIM)
{
    //st |= 0x01;
}

bool PCA9632LED_I2C::hw_init()
{
    uint8_t mode1 = ( PCA9632_AUTOINC_DISABLED 
                   | PCA9632_AUTOINC_MODE0
                   | PCA9632_ACTIVE_MODE);
	
    uint8_t mode2 = ( PCA9632_GROUPCTRL_BLINKING
                | PCA9632_OUTPUT_ON_ACK
                | PCA9632_OUTNE
                | PCA9632_OUTPUT_NORMAL
                | PCA9632_OUTPUT_OPENDRAIN);  

    uint8_t ledout = ( PCA9632_OUTPUT_INDIVIDUAL<<CHANNEL0_SHIFT 
                | PCA9632_OUTPUT_INDIVIDUAL<<CHANNEL1_SHIFT
                | PCA9632_OUTPUT_INDIVIDUAL<<CHANNEL2_SHIFT
                | PCA9632_OUTPUT_OFF       <<CHANNEL3_SHIFT);
    _dev = hal.i2c_mgr->get_device(1, PCA9632_ADDRESS);
    //gcs().send_text(MAV_SEVERITY_INFO,"led1");
    //st |= 0x02;
    if (!_dev) {
        return false;
    }
    //st |= 0x04;
    //gcs().send_text(MAV_SEVERITY_INFO,"led2");
    _dev->get_semaphore()->take_blocking();

    _dev->set_retries(5);
    
    /* Initialize PCA9632 mode1 register */
        
        _dev->write_register(REG_PCA9632_MODE1, mode1);
        /* Initialize PCA9632 mode2 register */
        _dev->write_register(REG_PCA9632_MODE2, mode2);
        
        
        /* Initialize PCA9632 GRPPWM register */
        _dev->write_register(REG_PCA9632_GRPPWM, 0xFF);
        /* Initialize PCA9632 GRPFREQ register */
        _dev->write_register(REG_PCA9632_GRPFREQ, 0x00);
        /* Initialize PCA9632 LEDOUT register */
       _dev->write_register(REG_PCA9632_LEDOUT, ledout);
    // read the current mode1 configuration
    //if (!_dev->read_registers(PCA9632_MODE1, &mode1, sizeof(mode1))) {
      //  _dev->get_semaphore()->give();
        //return false;
    //}

    // bring the device out of sleep, and enable auto register increment
    //uint8_t new_mode1 = (mode1 | PCA9632_MODE_AUTO_INCREMENT) & ~PCA9632_MODE_SLEEP;
    //const uint8_t config[2] = {PCA9632_MODE1, new_mode1};
    //if (!_dev->transfer(config, sizeof(config), nullptr, 0)) {
    //    _dev->get_semaphore()->give();
     //   return false;
    //}

    _dev->set_retries(1);

    _dev->get_semaphore()->give();

    _dev->register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&PCA9632LED_I2C::_timer, void));

    return true;
}

// set_rgb - set color as a combination of red, green and blue values
bool PCA9632LED_I2C::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    rgb.r = red;
    rgb.g = green;
    rgb.b = blue;
    _need_update = true;
    return true;
}

void PCA9632LED_I2C::_timer(void)
{
    if (!_need_update) {
        return;
    }
    _need_update = false;
    
    uint8_t red_adjusted = rgb.r * 0x10;
    uint8_t green_adjusted = rgb.g * 0x10;
    uint8_t blue_adjusted = rgb.b * 0x10;
    //gcs().send_text(MAV_SEVERITY_INFO,"led3=%d",st);
    //uint8_t blue_channel_lsb = blue_adjusted & 0xFF;
    //uint8_t blue_channel_msb = blue_adjusted >> 8;

    //uint8_t green_channel_lsb = green_adjusted & 0xFF;
    //uint8_t green_channel_msb = green_adjusted >> 8;

    //uint8_t red_channel_lsb = red_adjusted & 0xFF;
    //uint8_t red_channel_msb = red_adjusted >> 8;


    //uint8_t transaction[] = {PCA9632_PWM, 0x00, 0x00, blue_channel_lsb, blue_channel_msb,
	//		     0x00, 0x00, green_channel_lsb, green_channel_msb,
	//		     0x00, 0x00, red_channel_lsb, red_channel_msb};

    //_dev->transfer(transaction, sizeof(transaction), nullptr, 0);
    _dev->write_register(REG_PCA9632_PWM0, red_adjusted);
    _dev->write_register(REG_PCA9632_PWM1, green_adjusted);
    _dev->write_register(REG_PCA9632_PWM2, blue_adjusted);
}
