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
    serial_compass.update();
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
    uint8_t proximity_valid_bitmask=0;
    if (copter.g2.proximity.get_status() == AP_Proximity::Status::Good) {
        Proximity_Distance_Array dist_array;
        if (copter.g2.proximity.get_horizontal_distances(dist_array)) {
            for (uint8_t i = 0; i < PROXIMITY_MAX_DIRECTION; i++) {
                if (dist_array.valid(i)) {
                    proximity_valid_bitmask |= (1U << i);
                } else if (!(proximity_valid_bitmask & (1U << i))) {
                    // we've never sent this distance out, so we don't
                    // need to send an invalid one.
                    continue;
                }

                if(get_mode() == 3 || get_mode() == 29 )
                {               
                    //test_dis_home = tof_val;//home_distance()/100.0;
                    //test_dis_tof = dist_array.distance[i] * 100.0f;
                    //proximity_bitmask_val = i;
                   // set_mode_SmartRTL_or_RTL(ModeReason::AUTO_RTL_EXIT);
                   if(ahrs_view->groundspeed() > 0.5 && dist_array.distance[i]>0.2 && dist_array.distance[i] < g.avoid_distan_m)
                   {
                        if(g.avoid_behavior==1)
                        {
                                set_mode(Mode::Number::RTL, ModeReason::AVOIDANCE);
                        }
                        else if(g.avoid_behavior==2)
                        {
                                set_mode(Mode::Number::SMART_RTL, ModeReason::AVOIDANCE);
                        }
                        else
                        {
                                set_mode(Mode::Number::LOITER, ModeReason::AVOIDANCE);
                        }
                    }
                    
                }
                // if(get_mode() == 3 && ahrs_view->groundspeed() > 0.5 && dist_array.distance[i]>0.2 && dist_array.distance[i] < g.avoid_distan_m)
                // {
                //     set_mode(Mode::Number::LOITER, ModeReason::AVOIDANCE);
                //     if(avoid_rtl_count < 4)
                //     {
                //         g.rtl_altitude.set_value()=g.rtl_altitude+5000;
                //         avoid_rtl_flag = true;
                //         avoid_rtl_delay = 0;
                //     }
                    
                // }


            }
        }
    }
    // avoid_rtl_delay++;
    // if(avoid_rtl_flag && avoid_rtl_delay >150)
    // {
    //     avoid_rtl_flag = false;
    //     avoid_rtl_delay = 0;
    //     set_mode(Mode::Number::RTL, ModeReason::AVOIDANCE);
    // }
    
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
    Vector3f veh;
    serial_concentrator.update();
    if(get_mode() == 6)
    {
        if( copter.serial_concentrator.openmv_data[0].active==1 || copter.serial_concentrator.openmv_data[1].active==1||copter.serial_concentrator.openmv_data[2].active==1||copter.serial_concentrator.openmv_data[3].active==1||copter.serial_concentrator.openmv_data[6].active==1)
            {
                rc_openmv_count++;
                
                if(rc_openmv_count>3)
                {
                    set_mode(Mode::Number::OPENMV, ModeReason::AVOIDANCE);
                    have_openmv_flag = true;
                    rc_openmv_count=0;
                }
                copter.serial_concentrator.openmv_data[0].active=0;
                copter.serial_concentrator.openmv_data[1].active=0;
                copter.serial_concentrator.openmv_data[2].active=0;
                copter.serial_concentrator.openmv_data[3].active=0;
                copter.serial_concentrator.openmv_data[6].active=0;
            }
    }
    if(get_mode() == 6 && !have_openmv_flag )
    {
        if(g2.openmv_max_time)
        {
            if( copter.serial_concentrator.openmv_data[0].active==1 || copter.serial_concentrator.openmv_data[1].active==1||copter.serial_concentrator.openmv_data[2].active==1||copter.serial_concentrator.openmv_data[3].active==1||copter.serial_concentrator.openmv_data[6].active==1)
            {
                // if(copter.serial_concentrator.openmv_data[6].active!=1)
                // {
                     rc_openmv_count++;
                // }
                // else
                // {
                //     if(copter.serial_concentrator.openmv_data[6].x<285 || copter.serial_concentrator.openmv_data[6].x>375 || copter.serial_concentrator.openmv_data[6].y<185|| copter.serial_concentrator.openmv_data[6].y>275)
                //     set_mode(Mode::Number::OPENMV, ModeReason::AVOIDANCE);
                //     rc_openmv_count=0;
                    
                // }
                if(rc_openmv_count>3)
                {
                    set_mode(Mode::Number::OPENMV, ModeReason::AVOIDANCE);
                    have_openmv_flag = true;
                    rc_openmv_count=0;
                }
                copter.serial_concentrator.openmv_data[0].active=0;
                copter.serial_concentrator.openmv_data[1].active=0;
                copter.serial_concentrator.openmv_data[2].active=0;
                copter.serial_concentrator.openmv_data[3].active=0;
                copter.serial_concentrator.openmv_data[6].active=0;
            }
            else
            {
               if(copter.ahrs_view->get_relative_position_NED_home(veh))
                {
                    if(abs(veh.z)>2 && abs(veh.z)<8)
                    {
                        set_mode(Mode::Number::OPENMV, ModeReason::AVOIDANCE);
                        have_openmv_flag = true;
                    }
                } 
            }
        }
        //if(abs(copter.serial_concentrator.gimbal_st_u.pitch)> 85 && abs(copter.serial_concentrator.gimbal_st_u.pitch) <95)
        {
            AP_Mount *mount = AP::mount();
            if (mount != nullptr) {
                mavlink_command_long_t packet;
                packet.command = MAV_CMD_DO_MOUNT_CONTROL;
                packet.param1 = -90;
                packet.param2 = 0;
                packet.param3 = 0;
                packet.param7 = 2;
                mount->handle_command_long(packet);
            }
            gcs().send_text(MAV_SEVERITY_INFO,"rtl ctl pitch");
            
        }
        have_move_flag=true;
    }
    if(get_mode() == 5)
    {
        have_move_flag=false;
        have_openmv_flag=false;
    }
    else if(ahrs_view->groundspeed() > 1)
    {
        have_move_flag=true;
    }
    if(get_mode() == 3 && have_move_flag)
    {
        if(g2.openmv_max_time)
        {
            if( copter.serial_concentrator.openmv_data[0].active==1 || copter.serial_concentrator.openmv_data[1].active==1||copter.serial_concentrator.openmv_data[2].active==1||copter.serial_concentrator.openmv_data[3].active==1||copter.serial_concentrator.openmv_data[6].active==1)
            {
                // if(copter.serial_concentrator.openmv_data[6].active!=1)
                // {
                     rc_openmv_count++;
                // }
                // else
                // {
                //     if(copter.serial_concentrator.openmv_data[6].x<285 || copter.serial_concentrator.openmv_data[6].x>375 || copter.serial_concentrator.openmv_data[6].y<185|| copter.serial_concentrator.openmv_data[6].y>275)
                //     set_mode(Mode::Number::OPENMV, ModeReason::AVOIDANCE);
                //     rc_openmv_count=0;
                    
                // }
                if(rc_openmv_count>3)
                {
                    set_mode(Mode::Number::OPENMV, ModeReason::AVOIDANCE);
                    rc_openmv_count=0;
                    have_openmv_flag = true;
                }
                copter.serial_concentrator.openmv_data[0].active=0;
                copter.serial_concentrator.openmv_data[1].active=0;
                copter.serial_concentrator.openmv_data[2].active=0;
                copter.serial_concentrator.openmv_data[3].active=0;
                copter.serial_concentrator.openmv_data[6].active=0;
            }
            else
            {
               if(copter.ahrs_view->get_relative_position_NED_home(veh))
                {
                    if(abs(veh.z)>2 && abs(veh.z)<8)
                    {
                        set_mode(Mode::Number::OPENMV, ModeReason::AVOIDANCE);
                        have_openmv_flag = true;
                    }
                } 
            }
        }
    }
    // if(get_mode() == 3 && g.rtk_behavior)
    // {
    //     if( ahrs_view->groundspeed() > 0.5 && gps.status()!=AP_GPS::GPS_OK_FIX_3D_RTK_FIXED)
    //     {
    //         set_mode(Mode::Number::LOITER, ModeReason::AVOIDANCE);
    //     } 
    // }
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
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
