/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "Copter.h"


/*
 * control_testmode1.cpp - init and run calls for new flight mode
   created by MQ
   just a copy of control_althold to test if it can really work
 */

// newflightmode_init - initialise flight mode
bool Copter::testmode1_init(bool ignore_checks)
{
    // do any required initialisation of the flight mode here
    // this code will be called whenever the operator switches into this mode

    // return true initialisation is successful, false if it fails
    // if false is returned here the vehicle will remain in the previous flight mode

    #if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to enter Alt Hold if the Rotor Runup is not complete
        if (!ignore_checks && !motors.rotor_runup_complete()){
            return false;
        }
    #endif

    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

    // stop takeoff if running
    takeoff_stop();

    I_accumulation_right = 0.0; //mq obstacle avoidance
    I_accumulation_left = 0.0; //mq obstacle avoidance

    return true;
}

// newflightmode_run - runs the main controller
// will be called at 100hz or more
void Copter::testmode1_run()
{
    // if not armed or throttle at zero, set throttle to zero and exit immediately
    // if(!motors.armed() || g.rc_3.control_in <= 0) {
    //    attitude_control.relax_bf_rate_controller();
    //    attitude_control.set_yaw_target_to_current_heading();
    //    attitude_control.set_throttle_out(0, false);
    //    return;
    //}

    float obstacle_distance_limit = 100;  //mq, in cm parameter
    float scaling_factor = 10;  //mq
    float Kp = 6.0;     //parameter
    float Ki = 0.02;    //parameter
    float Kd = 0.005;   //parameter


    AltHoldModeState althold_state;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control.get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

    const Vector3f& vel = inertial_nav.get_velocity();    //from drift mode

    // rotate roll, pitch input from north facing to vehicle's perspective
    float roll_vel =  vel.y * ahrs.cos_yaw() - vel.x * ahrs.sin_yaw(); // mq, body roll vel in cm/s
    float target_roll_vel = scaling_factor *(distance_k_plus - obstacle_distance_limit);  //mq, in cm/s

#if FRAME_CONFIG == HELI_FRAME
    // helicopters are held on the ground until rotor speed runup has finished
    bool takeoff_triggered = (ap.land_complete && (target_climb_rate > 0.0f) && motors.rotor_runup_complete());
#else
    bool takeoff_triggered = ap.land_complete && (target_climb_rate > 0.0f);
#endif

    // Alt Hold State Machine Determination
    if (!motors.armed() || !motors.get_interlock()) {
        althold_state = AltHold_MotorStopped;
    } else if (takeoff_state.running || takeoff_triggered) {
        althold_state = AltHold_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        althold_state = AltHold_Landed;
    } else {
        althold_state = AltHold_Flying;
    }

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:

        motors.set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
#if FRAME_CONFIG == HELI_FRAME    
        // force descent rate and call position controller
        pos_control.set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
#else
        attitude_control.reset_rate_controller_I_terms();
        attitude_control.set_yaw_target_to_current_heading();
        pos_control.relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
        pos_control.update_z_controller();
        break;

    case AltHold_Takeoff:
        // set motors to full range
        motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // call position controller
        pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control.add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control.update_z_controller();
        break;

    case AltHold_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }

        attitude_control.reset_rate_controller_I_terms();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        pos_control.relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control.update_z_controller();
        break;

    case AltHold_Flying:
        motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        // call attitude controller


        // start of object detection and ovoidance part, mq
        if (distance_k_plus < obstacle_distance_limit)  //if object if very close
        {

            target_roll = PID_control(roll_vel, target_roll_vel, target_roll, Kp, Ki, Kd, &I_accumulation_right);

        }


        else{
            if (roll_vel>target_roll_vel/5.0)   //activate only if veloity of approach limit is too high
                {target_roll = PID_control(roll_vel, target_roll_vel/5.0, target_roll, Kp, Ki, Kd, &I_accumulation_right);

                }

            else {I_accumulation_right = 0.0; }

            }

            target_roll = constrain_float(target_roll, -4500.0, 4500.0);  //mq, constrain the output angle to +-40degree

           // distance_1 = target_roll;
            //end of object detection and avoidance part, mq

        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // adjust climb rate using rangefinder
        if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }

        // call position controller
        pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control.update_z_controller();
        break;
    }
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


/*
void Copter::object_fence_right(float obstacle_distance_limit)
{
    if (sonar_distance1 < obstacle_distance_limit)  //if object if very close
    {

        target_roll = PID_control(roll_vel, target_roll_vel, target_roll, Kp, Ki, Kd, I_accumulation_right);

    }


    else{
        if (roll_vel>target_roll_vel)   //activate only if veloity of approach limit is too high
            {target_roll = PID_control(roll_vel, target_roll_vel, target_roll, Kp, Ki, Kd, I_accumulation_right);

            }

        else {I_accumulation_right = 0.0; }

        }

}

*/

float Copter::PID_control(float current_tate, float target_state, float input, float Kp, float Ki, float Kd, float *I_previous)
{   float output;
    float dt = 0.0025; //400hz loop
    float P_term, I_term, D_term;
    float I_term_previous = *I_previous;
    float error =target_state - current_tate; //in cm/s
    float error_factor=1.0; //
    float outside_fence = 130.0;  //parameter
    float inside_fence = 110.0;     //parameter

    if ( distance_k_plus< outside_fence)  //scale down the input when error approach limit
        {error_factor = ( distance_k_plus - inside_fence)/(outside_fence - inside_fence); //proportionally scale down the input between fence and outskirt

            if ( distance_k_plus < inside_fence)
            {
                error_factor = 0.0; //no input when it is within the fence
            }

        }

    P_term = Kp*error;
    D_term = 0;
    I_term = I_term_previous + error*Ki;
    output = input*error_factor + P_term + I_term + D_term; //scaled input plus PID terms
    *I_previous = I_term;
    return output;
}