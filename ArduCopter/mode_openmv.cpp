#include "Copter.h"

#if MODE_OPENMV_ENABLED == ENABLED
#define OPENMV_MID_X 320.0
#define OPENMV_MID_Y 240.0

// #define OPENMV_MID_X 544.0
// #define OPENMV_MID_Y 360.0
/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
bool ModeOpenmv::init(bool ignore_checks)
{
    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }
    loiter_nav->init_target();

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

#if PRECISION_LANDING == ENABLED
    _precision_loiter_active = false;
#endif
    ctr_timer = g2.openmv_max_time*500;
    ctr_count = g2.openmv_max_count;
     gcs().send_text(MAV_SEVERITY_INFO,"openmv max time %ld",ctr_timer);
    return true;
}

#if PRECISION_LANDING == ENABLED
bool ModeOpenmv::do_precision_loiter()
{
    if (!_precision_loiter_enabled) {
        return false;
    }
    if (copter.ap.land_complete_maybe) {
        return false;        // don't move on the ground
    }
    // if the pilot *really* wants to move the vehicle, let them....
    if (loiter_nav->get_pilot_desired_acceleration().length() > 50.0f) {
        return false;
    }
    if (!copter.precland.target_acquired()) {
        return false; // we don't have a good vector
    }
    return true;
}

void ModeOpenmv::precision_loiter_xy()
{
    loiter_nav->clear_pilot_desired_acceleration();
    Vector2f target_pos, target_vel;
    if (!copter.precland.get_target_position_cm(target_pos)) {
        target_pos = inertial_nav.get_position_xy_cm();
    }
    // get the velocity of the target
    copter.precland.get_target_velocity_cms(inertial_nav.get_velocity_xy_cms(), target_vel);

    Vector2f zero;
    Vector2p landing_pos = target_pos.topostype();
    // target vel will remain zero if landing target is stationary
    pos_control->input_pos_vel_accel_xy(landing_pos, target_vel, zero);
    // run pos controller
    pos_control->update_xy_controller();
}
#endif
// loiter_run - runs the loiter controller
// should be called at 100hz or more
void ModeOpenmv::run()
{
    //static bool mv_mid_flag = false;
    static float target_roll_mv=0;
    static float target_pitch_mv=0;
    static float target_climb_rate_mv=0;
    float target_roll_rc, target_pitch_rc;
    static float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;
    static int mv_time = 0;
    static int mv_alt_check_time = 0;
    //static int mv_pix_x_check_time = 0;
    //static int mv_check_v_time = 0;
    int16_t target_body_frame_x=0;
    int16_t target_body_frame_y=0;
    Vector3f vef;
    Vector3f veh;
    float pos_d_h;
    static int32_t max_x=100;
    static int32_t max_y=100;
    static int32_t max_speed=50;
    // copter.serial_concentrator.openmv_data[6].active=1;
    // copter.serial_concentrator.openmv_data[6].r =80;
    // copter.serial_concentrator.openmv_data[6].ipx_x_d = 10000;
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    //r+ you  r- zuo  y-qian y+hou z- xia
    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll_rc, target_pitch_rc, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());
        ctr_timer--;
        if(ctr_timer == 0)
        {
            //gcs().send_text(MAV_SEVERITY_INFO,"openmv ctr timer out");
            ctr_timer=1;
            target_climb_rate_mv=100;
             if(copter.ahrs_view->get_relative_position_NED_home(veh))
            {
                if(ctr_count)
                {
                    if(abs(veh.z)>12)
                    {
                        target_climb_rate_mv=-30;
                        ctr_count--;
                        ctr_timer = g2.openmv_max_time*500;
                        gcs().send_text(MAV_SEVERITY_INFO,"openmv ctr count: %d",ctr_count);
                    }
                }
                else
                {
                    if(abs(veh.z)>20)
                    {
                        gcs().send_text(MAV_SEVERITY_INFO,"openmv ctr timer out");
                        target_climb_rate_mv=0;
                        Location new_home_loc;
                        new_home_loc.lat=g2.openmv_lat;
                        new_home_loc.lng=g2.openmv_lng;
                        new_home_loc.alt=g2.openmv_alt*100;//cm
                        if(new_home_loc.lat !=0 && new_home_loc.lng!=0 && new_home_loc.alt !=0)
                        {
                            if(copter.set_home(new_home_loc, false))
                            {
                                gcs().send_text(MAV_SEVERITY_INFO,"openmv set home lat: %ld lng: %ld,alt:%ld",new_home_loc.lat,new_home_loc.lng,new_home_loc.alt);
                                set_mode(Mode::Number::RTL, ModeReason::AVOIDANCE);
                            }
                        }
                        ctr_timer = g2.openmv_max_time*500;
                        ctr_count = g2.openmv_max_count;
                    }
                }
                
            }
            
        }
        else
        {
            if(copter.serial_concentrator.openmv_data[4].active)
            {
                //mv_mid_flag = true;
                mv_time = 0;
                target_climb_rate_mv=0;
                target_body_frame_x = copter.serial_concentrator.openmv_data[4].x-OPENMV_MID_X;  // 480*720  往左偏移一点
                target_body_frame_y = copter.serial_concentrator.openmv_data[4].y-OPENMV_MID_Y;
                // copter.serial_concentrator.openmv_data[0].active=0;
                // copter.serial_concentrator.openmv_data[1].active=0;
                // copter.serial_concentrator.openmv_data[2].active=0;
                copter.serial_concentrator.openmv_data[4].active=0;
                copter.serial_concentrator.openmv_data[6].active=0;
                if(copter.serial_concentrator.openmv_data[4].r>357 || copter.serial_concentrator.openmv_data[4].r<3)
                {
                    target_roll_mv = target_body_frame_x/OPENMV_MID_X*max_speed;
                    target_pitch_mv= target_body_frame_y/OPENMV_MID_Y*max_speed;
                    // target_roll_mv = roll_pid.calculate(target_body_frame_x)*max_speed;
                    // target_pitch_mv = pitch_pid.calculate(target_body_frame_y)*max_speed;
                    target_yaw_rate=0;

                }
                else
                {

                    target_roll_mv=0;
                    target_pitch_mv=0;
                    if(copter.serial_concentrator.openmv_data[4].r >=180)
                    {
                        if(copter.serial_concentrator.openmv_data[4].r>315)
                        {
                            target_yaw_rate =-100;
                        }
                        else if(copter.serial_concentrator.openmv_data[4].r>270)
                        {
                            target_yaw_rate =-500;
                        }
                        else
                        {
                            target_yaw_rate =-1000;
                        }
                        
                    }
                    else
                    {
                        if(copter.serial_concentrator.openmv_data[4].r<45)
                        {
                            target_yaw_rate =100;
                        }
                        else if(copter.serial_concentrator.openmv_data[4].r<90)
                        {
                            target_yaw_rate =500;
                        }
                        else
                        {
                            target_yaw_rate =1000;
                        }
                    }
                }
                
                //float distance_x =target_body_frame_x*copter.serial_concentrator.openmv_data[6].ipx_x_d/1000;
                //float distance_y =target_body_frame_y*copter.serial_concentrator.openmv_data[6].ipx_y_d/1000;
                if(abs(target_body_frame_x)<max_x && abs(target_body_frame_y)<max_y)
                //if((0<target_body_frame_x && target_body_frame_x<20) && abs(target_body_frame_y)<max_y)
                {
                    
                    /*if (copter.ahrs_view->get_velocity_NED(vef)) 
                    {
                        
                        if(abs(vef.z) < 0.09 )
                        {
                            mv_check_v_time++;
                            if(mv_check_v_time >10)
                            {
                                target_roll_mv=0;
                                target_pitch_mv=0;
                                //target_climb_rate_mv=0;
                                if(mv_check_v_time > 12)
                                {
                                    max_x=100;
                                    max_y=100;
                                    set_mode(Mode::Number::LAND, ModeReason::AVOIDANCE);
                                    gcs().send_text(MAV_SEVERITY_INFO,"*************vef land*********");
                                }
                            }
                        }
                        else
                        {
                            mv_check_v_time=0;
                        }
                        
                    }*/
                    // if(target_body_frame_x*copter.serial_concentrator.openmv_data[6].ipx_x_d<1200)
                    // {
                    //     mv_pix_x_check_time++;
                    //     if(mv_pix_x_check_time >1000)
                    //     {
                    //         target_roll_mv=0;
                    //         target_pitch_mv=0;
                    //         target_climb_rate_mv=0;
                    //         if(mv_pix_x_check_time >1500)
                    //         set_mode(Mode::Number::LAND, ModeReason::AVOIDANCE);
                    //     }
                    // }
                    // else
                    // {
                    //     mv_pix_x_check_time=0;
                    // }
                    //if(copter.ahrs_view->get_relative_gps_D_home(pos_d_h))
                   // copter.ahrs.get_relative_gps_D_home(pos_d_h);
                    AP::ahrs().get_relative_position_D_home(pos_d_h);
                    {
                        //if(abs(pos_d_h)<0.8)
                        if(pos_d_h>=-0.8)
                        {
                            mv_alt_check_time++;
                            if(mv_alt_check_time >=2)
                            {
                                target_roll_mv=0;
                                target_pitch_mv=0;
                                max_x=50;
                                max_y=50;
                                //target_climb_rate_mv=0;
                                if(mv_alt_check_time >=10)
                                {
                                    max_x=100;
                                    max_y=100;
                                    max_speed=100;
                                    ctr_timer = g2.openmv_max_time*500;
                                    ctr_count = g2.openmv_max_count;
                                    set_mode(Mode::Number::LAND, ModeReason::AVOIDANCE);
                                    gcs().send_text(MAV_SEVERITY_INFO,"*************pos land*********");
                                }
                                
                            }
                            
                        }
                        else 
                        {
                            mv_alt_check_time=0;
                        
                            if((pos_d_h)>=-1.6)
                            {
                                target_climb_rate_mv=-10;
                                max_x=35;
                                max_y=20;
                                max_speed=20;
                            }
                            else
                            {
                                target_climb_rate_mv = -30;
                            }
                        }
                    }
                    // else
                    // {
                    //     target_climb_rate_mv = -10;
                    //     veh.z=0;
                    // }
                }
                else
                {
                    mv_alt_check_time=0;
                    //mv_check_v_time=0;
                    //mv_pix_x_check_time=0;
                    
                }
                // gcs().send_text(MAV_SEVERITY_INFO,"ID5:ti_al=%d,ti_v=%d",mv_alt_check_time,mv_check_v_time);
                //gcs().send_text(MAV_SEVERITY_INFO,"ID5:x=%d,y=%d,P_d=%.2f,V=%.2f",target_body_frame_x,target_body_frame_y,pos_d_h,vef.z);
                // gcs().send_text(MAV_SEVERITY_INFO,"ID5:t_r=%.1f,t_p=%.1f,t_y=%.1f,t_z=%.1f",target_roll_mv,target_pitch_mv,target_yaw_rate,target_climb_rate_mv);

            }
            else if(copter.serial_concentrator.openmv_data[6].active)
            {
                //mv_mid_flag = true;
                mv_time = 0;
                target_climb_rate_mv=0;
                target_body_frame_x = copter.serial_concentrator.openmv_data[6].x-OPENMV_MID_X;  // 480*720  往左偏移一点
                target_body_frame_y = copter.serial_concentrator.openmv_data[6].y-OPENMV_MID_Y;
                // copter.serial_concentrator.openmv_data[0].active=0;
                // copter.serial_concentrator.openmv_data[1].active=0;
                // copter.serial_concentrator.openmv_data[2].active=0;
                copter.serial_concentrator.openmv_data[4].active=0;
                copter.serial_concentrator.openmv_data[6].active=0;
                if(copter.serial_concentrator.openmv_data[6].r>357 || copter.serial_concentrator.openmv_data[6].r<3)
                {
                    target_roll_mv = target_body_frame_x/OPENMV_MID_X*max_speed;
                    target_pitch_mv= target_body_frame_y/OPENMV_MID_Y*max_speed;
                    // target_roll_mv = roll_pid.calculate(target_body_frame_x)*max_speed;
                    // target_pitch_mv = pitch_pid.calculate(target_body_frame_y)*max_speed;
                    target_yaw_rate=0;

                }
                else
                {

                    target_roll_mv=0;
                    target_pitch_mv=0;
                    if(copter.serial_concentrator.openmv_data[6].r >=180)
                    {
                        if(copter.serial_concentrator.openmv_data[6].r>315)
                        {
                            target_yaw_rate =-100;
                        }
                        else if(copter.serial_concentrator.openmv_data[6].r>270)
                        {
                            target_yaw_rate =-500;
                        }
                        else
                        {
                            target_yaw_rate =-1000;
                        }
                        
                    }
                    else
                    {
                        if(copter.serial_concentrator.openmv_data[6].r<45)
                        {
                            target_yaw_rate =100;
                        }
                        else if(copter.serial_concentrator.openmv_data[6].r<90)
                        {
                            target_yaw_rate =500;
                        }
                        else
                        {
                            target_yaw_rate =1000;
                        }
                    }
                }
                
                //float distance_x =target_body_frame_x*copter.serial_concentrator.openmv_data[6].ipx_x_d/1000;
                //float distance_y =target_body_frame_y*copter.serial_concentrator.openmv_data[6].ipx_y_d/1000;
                if(abs(target_body_frame_x)<max_x && abs(target_body_frame_y)<max_y)
                {
                    
                    /*if (copter.ahrs_view->get_velocity_NED(vef)) 
                    {
                        
                        if(abs(vef.z) < 0.09 )
                        {
                            mv_check_v_time++;
                            if(mv_check_v_time >10)
                            {
                                target_roll_mv=0;
                                target_pitch_mv=0;
                                //target_climb_rate_mv=0;
                                if(mv_check_v_time > 12)
                                {
                                    max_x=100;
                                    max_y=100;
                                    set_mode(Mode::Number::LAND, ModeReason::AVOIDANCE);
                                    gcs().send_text(MAV_SEVERITY_INFO,"*************vef land*********");
                                }
                            }
                        }
                        else
                        {
                            mv_check_v_time=0;
                        }
                        
                    }*/
                    // if(target_body_frame_x*copter.serial_concentrator.openmv_data[6].ipx_x_d<1200)
                    // {
                    //     mv_pix_x_check_time++;
                    //     if(mv_pix_x_check_time >1000)
                    //     {
                    //         target_roll_mv=0;
                    //         target_pitch_mv=0;
                    //         target_climb_rate_mv=0;
                    //         if(mv_pix_x_check_time >1500)
                    //         set_mode(Mode::Number::LAND, ModeReason::AVOIDANCE);
                    //     }
                    // }
                    // else
                    // {
                    //     mv_pix_x_check_time=0;
                    // }
                    //if(copter.ahrs_view->get_relative_gps_D_home(pos_d_h))
                    //copter.ahrs.get_relative_gps_D_home(pos_d_h);
                    AP::ahrs().get_relative_position_D_home(pos_d_h);
                    {
                        if((pos_d_h)>=-1)
                        {
                            // mv_alt_check_time++;
                            // if(mv_alt_check_time >=5)
                            // {
                                target_roll_mv=0;
                                target_pitch_mv=0;
                                target_climb_rate_mv=0;
                            //     if(mv_alt_check_time >=10)
                            //     {
                            //         max_x=100;
                            //         max_y=100;
                            //         max_speed=50;
                            //         set_mode(Mode::Number::LAND, ModeReason::AVOIDANCE);
                            //         gcs().send_text(MAV_SEVERITY_INFO,"*************pos land*********");
                            //     }
                                
                            // }
                        }
                        else 
                        {
                            mv_alt_check_time=0;
                        
                            if((pos_d_h)>=-2)
                            {
                                target_climb_rate_mv=-10;
                                max_x=35;
                                max_y=20;
                                max_speed=20;
                            }
                            else
                            {
                                target_climb_rate_mv = -40;
                            }
                        }
                    }
                    // else
                    // {
                    //     target_climb_rate_mv = -10;
                    //     veh.z=0;
                    // }
                }
                else
                {
                    mv_alt_check_time=0;
                    //mv_check_v_time=0;
                    //mv_pix_x_check_time=0;
                    
                }
                // gcs().send_text(MAV_SEVERITY_INFO,"ID7:ti_al=%d,ti_v=%d",mv_alt_check_time,mv_check_v_time);
                // gcs().send_text(MAV_SEVERITY_INFO,"ID7:x=%d,y=%d,P_d=%.2f,V=%.2f",target_body_frame_x,target_body_frame_y,pos_d_h,vef.z);
                // gcs().send_text(MAV_SEVERITY_INFO,"ID7:t_r=%.1f,t_p=%.1f,t_y=%.1f,t_z=%.1f",target_roll_mv,target_pitch_mv,target_yaw_rate,target_climb_rate_mv);

            }/*
            else if(copter.serial_concentrator.openmv_data[0].active&&!mv_mid_flag)
            {
                mv_time = 0;
                mv_alt_check_time=0;
                mv_check_v_time=0;
                //mv_pix_x_check_time=0;
                target_climb_rate_mv=0;
                target_body_frame_x = copter.serial_concentrator.openmv_data[0].x-OPENMV_MID_X;  //480*720
                target_body_frame_y = copter.serial_concentrator.openmv_data[0].y-OPENMV_MID_Y;
                copter.serial_concentrator.openmv_data[0].active=0;
                copter.serial_concentrator.openmv_data[1].active=0;
                copter.serial_concentrator.openmv_data[2].active=0;
                copter.serial_concentrator.openmv_data[3].active=0;
                if(copter.serial_concentrator.openmv_data[0].r>340 || copter.serial_concentrator.openmv_data[0].r<20)
                {
                    target_roll_mv = target_body_frame_x/OPENMV_MID_X*100;
                    target_pitch_mv= target_body_frame_y/OPENMV_MID_Y*100;
                    target_yaw_rate=0;
                }
                else
                {
                    target_roll_mv=0;
                    target_pitch_mv=0;
                    if(copter.serial_concentrator.openmv_data[0].r >=180)
                    {
                        if(copter.serial_concentrator.openmv_data[0].r>315)
                        {
                            target_yaw_rate =-100;
                        }
                        else if(copter.serial_concentrator.openmv_data[0].r>270)
                        {
                            target_yaw_rate =-500;
                        }
                        else
                        {
                            target_yaw_rate =-1000;
                        }
                        
                    }
                    else
                    {
                        if(copter.serial_concentrator.openmv_data[0].r<45)
                        {
                            target_yaw_rate =100;
                        }
                        else if(copter.serial_concentrator.openmv_data[0].r<90)
                        {
                            target_yaw_rate =500;
                        }
                        else
                        {
                            target_yaw_rate =1000;
                        }
                    }
                }
                if(copter.ahrs_view->get_relative_position_NED_home(veh))
                {
                    if(abs(veh.z)>6)
                    {
                        target_climb_rate_mv=-10;
                    }
                }
                gcs().send_text(MAV_SEVERITY_INFO,"ID1:x=%d,y=%d",target_body_frame_x,target_body_frame_y);
                gcs().send_text(MAV_SEVERITY_INFO,"ID1:target_roll=%f,target_pitch=%f,Yaw=%f",target_roll_mv,target_pitch_mv,target_yaw_rate);
            }
            else if(copter.serial_concentrator.openmv_data[1].active&&!mv_mid_flag)
            {
                mv_time = 0;
                mv_alt_check_time=0;
                mv_check_v_time=0;
                //mv_pix_x_check_time=0;
                target_climb_rate_mv=0;
                target_body_frame_x = copter.serial_concentrator.openmv_data[1].x-OPENMV_MID_X;  // 480*720
                target_body_frame_y = copter.serial_concentrator.openmv_data[1].y-OPENMV_MID_Y;
                copter.serial_concentrator.openmv_data[0].active=0;
                copter.serial_concentrator.openmv_data[1].active=0;
                copter.serial_concentrator.openmv_data[2].active=0;
                copter.serial_concentrator.openmv_data[3].active=0;
                //copter.serial_concentrator.openmv_data[6].active=0;
                if(copter.serial_concentrator.openmv_data[1].r>340 || copter.serial_concentrator.openmv_data[1].r<20)
                {
                    target_roll_mv = target_body_frame_x/OPENMV_MID_X*100;
                    target_pitch_mv= target_body_frame_y/OPENMV_MID_Y*100;
                    target_yaw_rate=0;
                }
                else
                {
                    target_roll_mv=0;
                    target_pitch_mv=0;
                    if(copter.serial_concentrator.openmv_data[1].r >=180)
                    {
                        if(copter.serial_concentrator.openmv_data[1].r>315)
                        {
                            target_yaw_rate =-100;
                        }
                        else if(copter.serial_concentrator.openmv_data[1].r>270)
                        {
                            target_yaw_rate =-500;
                        }
                        else
                        {
                            target_yaw_rate =-1000;
                        }
                        
                    }
                    else
                    {
                        if(copter.serial_concentrator.openmv_data[1].r<45)
                        {
                            target_yaw_rate =100;
                        }
                        else if(copter.serial_concentrator.openmv_data[1].r<90)
                        {
                            target_yaw_rate =500;
                        }
                        else
                        {
                            target_yaw_rate =1000;
                        }
                    }
                }
                if(copter.ahrs_view->get_relative_position_NED_home(veh))
                {
                    if(abs(veh.z)>6)
                    {
                        target_climb_rate_mv=-10;
                    }
                }
                gcs().send_text(MAV_SEVERITY_INFO,"ID2:x=%d,y=%d",target_body_frame_x,target_body_frame_y);
                gcs().send_text(MAV_SEVERITY_INFO,"ID2:target_roll=%f,target_pitch=%f,Yaw=%f",target_roll_mv,target_pitch_mv,target_yaw_rate);
            }
            else if(copter.serial_concentrator.openmv_data[2].active&&!mv_mid_flag)
            {
                mv_time = 0;
                mv_alt_check_time=0;
                mv_check_v_time=0;
                //mv_pix_x_check_time=0;
                target_climb_rate_mv=0;
                target_body_frame_x = copter.serial_concentrator.openmv_data[2].x-OPENMV_MID_X;  // 480*720
                target_body_frame_y = copter.serial_concentrator.openmv_data[2].y-OPENMV_MID_Y;
                copter.serial_concentrator.openmv_data[0].active=0;
                copter.serial_concentrator.openmv_data[1].active=0;
                copter.serial_concentrator.openmv_data[2].active=0;
                copter.serial_concentrator.openmv_data[3].active=0;
                //copter.serial_concentrator.openmv_data[6].active=0;
                if(copter.serial_concentrator.openmv_data[2].r>340 || copter.serial_concentrator.openmv_data[2].r<20)
                {
                    target_roll_mv = target_body_frame_x/OPENMV_MID_X*100;
                    target_pitch_mv= target_body_frame_y/OPENMV_MID_Y*100;
                    target_yaw_rate=0;
                }
                else
                {
                    target_roll_mv=0;
                    target_pitch_mv=0;
                    if(copter.serial_concentrator.openmv_data[2].r >=180)
                    {
                        if(copter.serial_concentrator.openmv_data[2].r>315)
                        {
                            target_yaw_rate =-100;
                        }
                        else if(copter.serial_concentrator.openmv_data[2].r>270)
                        {
                            target_yaw_rate =-500;
                        }
                        else
                        {
                            target_yaw_rate =-1000;
                        }
                        
                    }
                    else
                    {
                        if(copter.serial_concentrator.openmv_data[2].r<45)
                        {
                            target_yaw_rate =100;
                        }
                        else if(copter.serial_concentrator.openmv_data[2].r<90)
                        {
                            target_yaw_rate =500;
                        }
                        else
                        {
                            target_yaw_rate =1000;
                        }
                    }
                }
                if(copter.ahrs_view->get_relative_position_NED_home(veh))
                {
                    if(abs(veh.z)>6)
                    {
                        target_climb_rate_mv=-10;
                    }
                }
                gcs().send_text(MAV_SEVERITY_INFO,"ID3:x=%d,y=%d",target_body_frame_x,target_body_frame_y);
                gcs().send_text(MAV_SEVERITY_INFO,"ID3:target_roll=%f,target_pitch=%f,Yaw=%f",target_roll_mv,target_pitch_mv,target_yaw_rate);
            }
            else if(copter.serial_concentrator.openmv_data[3].active &&!mv_mid_flag)
            {
                mv_time = 0;
                mv_alt_check_time=0;
                mv_check_v_time=0;
                //mv_pix_x_check_time=0;
                target_climb_rate_mv=0;
                target_body_frame_x = copter.serial_concentrator.openmv_data[3].x-OPENMV_MID_X;  // 720*480
                target_body_frame_y = copter.serial_concentrator.openmv_data[3].y-OPENMV_MID_Y;
                copter.serial_concentrator.openmv_data[0].active=0;
                copter.serial_concentrator.openmv_data[1].active=0;
                copter.serial_concentrator.openmv_data[2].active=0;
                copter.serial_concentrator.openmv_data[3].active=0;
                //copter.serial_concentrator.openmv_data[6].active=0;
                if(copter.serial_concentrator.openmv_data[3].r>340 || copter.serial_concentrator.openmv_data[3].r<20)
                {
                    target_roll_mv = target_body_frame_x/OPENMV_MID_X*100;
                    target_pitch_mv= target_body_frame_y/OPENMV_MID_Y*100;
                    target_yaw_rate=0;
                }
                else
                {
                    target_roll_mv=0;
                    target_pitch_mv=0;
                    if(copter.serial_concentrator.openmv_data[3].r >=180)
                    {
                        if(copter.serial_concentrator.openmv_data[3].r>315)
                        {
                            target_yaw_rate =-100;
                        }
                        else if(copter.serial_concentrator.openmv_data[3].r>270)
                        {
                            target_yaw_rate =-500;
                        }
                        else
                        {
                            target_yaw_rate =-1000;
                        }
                        
                    }
                    else
                    {
                        if(copter.serial_concentrator.openmv_data[3].r<45)
                        {
                            target_yaw_rate =100;
                        }
                        else if(copter.serial_concentrator.openmv_data[3].r<90)
                        {
                            target_yaw_rate =500;
                        }
                        else
                        {
                            target_yaw_rate =1000;
                        }
                    }
                }
                if(copter.ahrs_view->get_relative_position_NED_home(veh))
                {
                    if(abs(veh.z)>6)
                    {
                        target_climb_rate_mv=-10;
                    }
                }
                gcs().send_text(MAV_SEVERITY_INFO,"ID4:x=%d,y=%d",target_body_frame_x,target_body_frame_y);
                gcs().send_text(MAV_SEVERITY_INFO,"ID4:target_roll=%f,target_pitch=%f,Yaw=%f",target_roll_mv,target_pitch_mv,target_yaw_rate);
            }
        
            */
            mv_time++;
            if(mv_time==1000) gcs().send_text(MAV_SEVERITY_INFO,"clear mv target=%f",target_climb_rate);
            if(mv_time >=1000)
            {
                //mv_mid_flag=false;
                //copter.serial_concentrator.openmv_data[6].active=1;
                // target_roll_mv=target_roll_rc;
                // target_pitch_mv=target_pitch_rc;
                // get pilot desired climb rate
                //target_climb_rate_mv = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
                
                target_yaw_rate = 0;
                target_roll_mv=0;
                target_pitch_mv=0;
                 target_climb_rate_mv=0;
                if(copter.ahrs_view->get_relative_position_NED_home(veh))
                {
                    if(abs(veh.z)<5)
                    {
                        // mv_alt_check_time++;
                        // if(mv_alt_check_time >1000)
                        // {
                        //     target_roll_mv=0;
                        //     target_pitch_mv=0;
                        //     target_climb_rate_mv=0;
                        //     // if(mv_alt_check_time >1500)
                        //     // {
                        //     //     max_x=100;
                        //     //     max_y=100;
                        //     //     max_speed=100;
                        //     //     ctr_timer=1;
                        //     //    // set_mode(Mode::Number::LAND, ModeReason::AVOIDANCE);
                        //     //     gcs().send_text(MAV_SEVERITY_INFO,"*************target land*********");
                        //     // }
                        // }
                        target_climb_rate_mv=0;
                    }
                    else
                    {
                        //mv_alt_check_time=0;
                        target_climb_rate_mv = -30;
                    }
                }
                // if (copter.ahrs_view->get_velocity_NED(vef)) 
                // {
                    
                //     if(abs(vef.z) < 0.07 )
                //     {
                //         mv_check_v_time++;
                //         if(mv_check_v_time>1000)
                //         {
                //             target_roll_mv=0;
                //             target_pitch_mv=0;
                //             target_climb_rate_mv=0;
                //             if(mv_check_v_time > 1500)
                //             {
                //                 ctr_timer=1;
                //                 //set_mode(Mode::Number::LAND, ModeReason::AVOIDANCE);  
                //             }
                //         }
                //     }
                //     else
                //     {
                //         mv_check_v_time=0;
                //     }
                    
                // }
                // Vector3f vef;
                // if (copter.ahrs_view->get_velocity_NED(vef)) {
                //     if(vef.z < 0.2 && mv_time > 2000)
                //     {
                //         set_mode(Mode::Number::LAND, ModeReason::AVOIDANCE);
                //     }
                //     // 在成功获取速度信息后执行的逻辑
                // } else {
                //     // 处理未成功获取速度信息的情况
                // }
                
                // if(mv_time %100==0)
                // gcs().send_text(MAV_SEVERITY_INFO,"mv.z=%f,psh=%f",vef.z,veh.z);
                
            }
        }
        
        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll_mv, target_pitch_mv);

        // get pilot's desired yaw rate
        //target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

        // get pilot desired climb rate
        // target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());

        // mv_time++;
        
        // if(mv_time <10000)
        // {
        //     target_climb_rate=100;
        // }
        // else if(mv_time <20000)
        // {
        //     target_climb_rate=-100;
        // }
        // else if(mv_time <30000)
        // {
        //     target_climb_rate=0;
        //     mv_time = 0;
        // }
        // if(mv_time % 200==0)
        // {
        //     gcs().send_text(MAV_SEVERITY_INFO,"target_climb_rate=%f",target_climb_rate);
        // }
        target_climb_rate = constrain_float(target_climb_rate_mv, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine Determination
    AltHoldModeState loiter_state = get_alt_hold_state(target_climb_rate);

    // Loiter State Machine
    switch (loiter_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);

        // run loiter controller
        loiter_nav->update();

        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Flying:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if PRECISION_LANDING == ENABLED
        bool precision_loiter_old_state = _precision_loiter_active;
        if (do_precision_loiter()) {
            precision_loiter_xy();
            _precision_loiter_active = true;
        } else {
            _precision_loiter_active = false;
        }
        if (precision_loiter_old_state && !_precision_loiter_active) {
            // prec loiter was active, not any more, let's init again as user takes control
            loiter_nav->init_target();
        }
        // run loiter controller if we are not doing prec loiter
        if (!_precision_loiter_active) {
            loiter_nav->update();
        }
#else
        loiter_nav->update();
#endif

        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

uint32_t ModeOpenmv::wp_distance() const
{
    return loiter_nav->get_distance_to_target();
}

int32_t ModeOpenmv::wp_bearing() const
{
    return loiter_nav->get_bearing_to_target();
}
// land_start - initialise controllers to loiter over home
// void ModeOpenmv::land_start()
// {
//     _state = SubMode::LAND;
//     _state_complete = false;

//     // set horizontal speed and acceleration limits
//     pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
//     pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

//     // initialise loiter target destination
//     if (!pos_control->is_active_xy()) {
//         pos_control->init_xy_controller();
//     }

//     // initialise the vertical position controller
//     if (!pos_control->is_active_z()) {
//         pos_control->init_z_controller();
//     }

//     // initialise yaw
//     auto_yaw.set_mode(AUTO_YAW_HOLD);

// #if LANDING_GEAR_ENABLED == ENABLED
//     // optionally deploy landing gear
//     copter.landinggear.deploy_for_landing();
// #endif

// #if AP_FENCE_ENABLED
//     // disable the fence on landing
//     copter.fence.auto_disable_fence_for_landing();
// #endif
// }
// // land_run - run the landing controllers to put the aircraft on the ground
// // called by rtl_run at 100hz or more
// void ModeOpenmv::land_run(bool disarm_on_land)
// {
//     // check if we've completed this stage of RTL
//     _state_complete = copter.ap.land_complete;

//     // disarm when the landing detector says we've landed
//     if (disarm_on_land && copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
//         copter.arming.disarm(AP_Arming::Method::LANDED);
//     }

//     // if not armed set throttle to zero and exit immediately
//     if (is_disarmed_or_landed()) {
//         make_safe_ground_handling();
//         return;
//     }

//     // set motors to full range
//     motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

//     // run normal landing or precision landing (if enabled)
//     land_run_normal_or_precland();
// }
#endif
