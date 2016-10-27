/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


#include "Copter.h"

/*
 * control_newflightmode.cpp - init and run calls for new flight mode
 */


#ifndef DRIFT_SPEEDGAIN
 # define DRIFT_SPEEDGAIN 8.0f
#endif
#ifndef DRIFT_SPEEDLIMIT
 # define DRIFT_SPEEDLIMIT 560.0f
#endif

#ifndef DRIFT_THR_ASSIST_GAIN
 # define DRIFT_THR_ASSIST_GAIN 0.0018f    // gain controlling amount of throttle assistance
#endif

#ifndef DRIFT_THR_ASSIST_MAX
 # define DRIFT_THR_ASSIST_MAX  0.3f    // maximum assistance throttle assist will provide
#endif

#ifndef DRIFT_THR_MIN
 # define DRIFT_THR_MIN         0.213f  // throttle assist will be active when pilot's throttle is above this value
#endif
#ifndef DRIFT_THR_MAX
 # define DRIFT_THR_MAX         0.787f  // throttle assist will be active when pilot's throttle is below this value
#endif



// newflightmode_init - initialise flight mode
bool Copter::testmode2_init(bool ignore_checks)
{
    // do any required initialisation of the flight mode here
    // this code will be called whenever the operator switches into this mode

    // return true initialisation is successful, false if it fails
    // if false is returned here the vehicle will remain in the previous flight mode

    // initialize vertical speeds and leash lengths, from althold
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity, from althold
    pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

    // stop takeoff if running
    takeoff_stop();

    if (position_ok() || ignore_checks) {
        return true;
    }else{
        return false;
    }
}

// newflightmode_run - runs the main controller
// will be called at 100hz or more
void Copter::testmode2_run()
{
    // if not armed or throttle at zero, set throttle to zero and exit immediately
    //if(!motors.armed() || g.rc_3.control_in <= 0) {
    //if(!motors.armed() || ap.throttle_zero ) {
        //attitude_control.relax_bf_rate_controller();
    //    attitude_control.set_yaw_target_to_current_heading();
        //attitude_control.set_throttle_out(0, false);
    //    return;
    //}

    //from althold
    AltHoldModeState althold_state;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration, from althold
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);


    static float breaker = 0.0f;
    static float roll_input = 0.0f;
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;


    // if landed and throttle at zero, set throttle to zero and exit immediately
    if (!motors.armed() || !motors.get_interlock() || (ap.land_complete && ap.throttle_zero)) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    // clear landing flag above zero throttle
    if (!ap.throttle_zero) {
        set_land_complete(false);
    }

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // get pilot desired climb rate, from althold
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

    // get pilot's desired throttle, remove for althold
    //pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

    // Grab inertial velocity
    const Vector3f& vel = inertial_nav.get_velocity();

    // rotate roll, pitch input from north facing to vehicle's perspective
    float roll_vel =  vel.y * ahrs.cos_yaw() - vel.x * ahrs.sin_yaw(); // body roll vel
    float pitch_vel = vel.y * ahrs.sin_yaw() + vel.x * ahrs.cos_yaw(); // body pitch vel

    // gain sceduling for Yaw
    float pitch_vel2 = MIN(fabsf(pitch_vel), 2000);
    target_yaw_rate = ((float)target_roll/1.0f) * (1.0f - (pitch_vel2 / 5000.0f)) * g.acro_yaw_p;

    roll_vel = constrain_float(roll_vel, -DRIFT_SPEEDLIMIT, DRIFT_SPEEDLIMIT);
    pitch_vel = constrain_float(pitch_vel, -DRIFT_SPEEDLIMIT, DRIFT_SPEEDLIMIT);
    
    roll_input = roll_input * .96f + (float)channel_yaw->get_control_in() * .04f;

    //convert user input into desired roll velocity
    float roll_vel_error = roll_vel - (roll_input / DRIFT_SPEEDGAIN);

    // Roll velocity is feed into roll acceleration to minimize slip
    target_roll = roll_vel_error * -DRIFT_SPEEDGAIN;
    target_roll = constrain_float(target_roll, -4500.0f, 4500.0f);

    // If we let go of sticks, bring us to a stop
    if(is_zero(target_pitch)){
        // .14/ (.03 * 100) = 4.6 seconds till full breaking
        breaker += .03f;
        breaker = MIN(breaker, DRIFT_SPEEDGAIN);
        target_pitch = pitch_vel * breaker;
    }else{
        breaker = 0.0f;
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // call attitude controller
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // adjust climb rate using rangefinder, from althold
    if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
        target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
    }

    // call position controller, from althold    
    pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
    pos_control.update_z_controller();


    // output pilot's throttle with angle boost, removed for althold
    //attitude_control.set_throttle_out(get_throttle_assist(vel.z, pilot_throttle_scaled), true, g.throttle_filt);
    // convert pilot input into desired vehicle angles or rotation rates
    //   g.rc_1.control_in : pilots roll input in the range -4500 ~ 4500
    //   g.rc_2.control_in : pilot pitch input in the range -4500 ~ 4500
    //   g.rc_3.control_in : pilot's throttle input in the range 0 ~ 1000
    //   g.rc_4.control_in : pilot's yaw input in the range -4500 ~ 4500

    // call one of attitude controller's attitude control functions like
    //   attitude_control.angle_ef_roll_pitch_rate_yaw(roll angle, pitch angle, yaw rate);

    // call position controller's z-axis controller or simply pass through throttle
    //   attitude_control.set_throttle_out(desired throttle, true);
}


//float Copter::get_throttle_assist(float velz, float pilot_throttle_scaled) //editted by MQ
//{
    // throttle assist - adjusts throttle to slow the vehicle's vertical velocity
    //      Only active when pilot's throttle is between 213 ~ 787
    //      Assistance is strongest when throttle is at mid, drops linearly to no assistance at 213 and 787
//    float thr_assist = 0.0f;
//   if (pilot_throttle_scaled > DRIFT_THR_MIN && pilot_throttle_scaled < DRIFT_THR_MAX) {
        // calculate throttle assist gain
//        thr_assist = 1.2f - ((float)fabsf(pilot_throttle_scaled - 0.5f) / 0.24f);
//       thr_assist = constrain_float(thr_assist, 0.0f, 1.0f) * -DRIFT_THR_ASSIST_GAIN * velz;

        // ensure throttle assist never adjusts the throttle by more than 300 pwm
//        thr_assist = constrain_float(thr_assist, -DRIFT_THR_ASSIST_MAX, DRIFT_THR_ASSIST_MAX);
//    }
    
//    return constrain_float(pilot_throttle_scaled + thr_assist, 0.0f, 1.0f);
//}