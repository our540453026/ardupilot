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
    //uint8_t proximity_valid_bitmask=0;
    //mavlink_channel_t           chan;
    //uint32_t mod;
    //float dis;
    if (copter.g2.proximity.get_status() == AP_Proximity::Status::Good) {
        AP_Proximity::Proximity_Distance_Array dist_array;
        if (copter.g2.proximity.get_horizontal_distances(dist_array)) {

            //for (uint8_t i = 0; i < PROXIMITY_MAX_DIRECTION; i++) {
                //if (!HAVE_PAYLOAD_SPACE(chan, DISTANCE_SENSOR)) {
                //    return;
                //}
                //gcs().send_text(MAV_SEVERITY_INFO,"mode=%d",gcs().custom_mode());
                if(gcs().custom_mode() == 3 && ahrs_view->groundspeed() > 1.0 && dist_array.distance[0]>5 && dist_array.distance[0] < 35)
                {               
                    //test_dis_home = tof_val;//home_distance()/100.0;
                    //test_dis_tof = dist_array.distance[i] * 100.0f;
                    //proximity_bitmask_val = i;
                   // set_mode_SmartRTL_or_RTL(ModeReason::AUTO_RTL_EXIT);
                    set_mode(Mode::Number::RTL, ModeReason::AVOIDANCE);
                    
                }   
                //if(copter.g2.proximity.get_upward_distance(dis))
                //{
                   // dis = dist_array.distance[0];
                    //gcs().send_text(MAV_SEVERITY_INFO,"dis=%f",dis);
                //}           
            //}
        }
    }
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
    serial_compass.update();
    serial_concentrator.update();
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
    serial_compass.update();
     serial_concentrator.update();
    Log_Write_MotBattBMS();
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
    //float yaw1 = 0.0;
    //float accel = 0.0;
   // gps.gps_yaw_deg(yaw1,accel);
    //gcs().send_text(MAV_SEVERITY_INFO,"yaw=%.1f,%.1f",yaw1,accel);
    // put your 1Hz code here
    
    Log_Write_MotBattBMS();
    //mod = gcs().custom_mode();
    //gcs().send_text(MAV_SEVERITY_INFO,"mode=%d",mod);
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
