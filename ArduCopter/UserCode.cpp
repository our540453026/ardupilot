#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
      serial_concentrator.update();
      serial_compass.update();
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
    serial_compass.update();
     serial_concentrator.update();
    //Log_Write_MotBattBMS();
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
    float yaw1 = 0.0;
    float accel = 0.0;
    uint32_t t;
    gps.gps_yaw_deg(yaw1,accel,t);
    gcs().send_text(MAV_SEVERITY_INFO,"yaw0=%f,%f,%ld",yaw1,accel,t);
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
