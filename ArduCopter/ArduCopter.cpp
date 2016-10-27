/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

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
/*
 *  ArduCopter Version 3.0
 *  Creator:        Jason Short
 *  Lead Developer: Randy Mackay
 *  Lead Tester:    Marco Robustini 
 *  Based on code and ideas from the Arducopter team: Leonard Hall, Andrew Tridgell, Robert Lefebvre, Pat Hickey, Michael Oborne, Jani Hirvinen, 
                                                      Olivier Adler, Kevin Hester, Arthur Benemann, Jonathan Challinger, John Arne Birkeland,
                                                      Jean-Louis Naudin, Mike Smith, and more
 *  Thanks to:	Chris Anderson, Jordi Munoz, Jason Short, Doug Weibel, Jose Julio
 *
 *  Special Thanks to contributors (in alphabetical order by first name):
 *
 *  Adam M Rivera       :Auto Compass Declination
 *  Amilcar Lucas       :Camera mount library
 *  Andrew Tridgell     :General development, Mavlink Support
 *  Angel Fernandez     :Alpha testing
 *  AndreasAntonopoulous:GeoFence
 *  Arthur Benemann     :DroidPlanner GCS
 *  Benjamin Pelletier  :Libraries
 *  Bill King           :Single Copter
 *  Christof Schmid     :Alpha testing
 *  Craig Elder         :Release Management, Support
 *  Dani Saez           :V Octo Support
 *  Doug Weibel	        :DCM, Libraries, Control law advice
 *  Emile Castelnuovo   :VRBrain port, bug fixes
 *  Gregory Fletcher    :Camera mount orientation math
 *  Guntars             :Arming safety suggestion
 *  HappyKillmore       :Mavlink GCS
 *  Hein Hollander      :Octo Support, Heli Testing
 *  Igor van Airde      :Control Law optimization
 *  Jack Dunkle         :Alpha testing
 *  James Goppert       :Mavlink Support
 *  Jani Hiriven        :Testing feedback
 *  Jean-Louis Naudin   :Auto Landing
 *  John Arne Birkeland	:PPM Encoder
 *  Jose Julio          :Stabilization Control laws, MPU6k driver
 *  Julien Dubois       :PosHold flight mode
 *  Julian Oes          :Pixhawk
 *  Jonathan Challinger :Inertial Navigation, CompassMot, Spin-When-Armed
 *  Kevin Hester        :Andropilot GCS
 *  Max Levine          :Tri Support, Graphics
 *  Leonard Hall        :Flight Dynamics, Throttle, Loiter and Navigation Controllers
 *  Marco Robustini     :Lead tester
 *  Michael Oborne      :Mission Planner GCS
 *  Mike Smith          :Pixhawk driver, coding support
 *  Olivier Adler       :PPM Encoder, piezo buzzer
 *  Pat Hickey          :Hardware Abstraction Layer (HAL)
 *  Robert Lefebvre     :Heli Support, Copter LEDs
 *  Roberto Navoni      :Library testing, Porting to VRBrain
 *  Sandro Benigno      :Camera support, MinimOSD
 *  Sandro Tognana      :PosHold flight mode
 *  ..and many more.
 *
 *  Code commit statistics can be found here: https://github.com/ArduPilot/ardupilot/graphs/contributors
 *  Wiki: http://copter.ardupilot.org/
 *  Requires modified version of Arduino, which can be found here: http://ardupilot.com/downloads/?category=6
 *
 */

#include "Copter.h"

#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(Copter, &copter, func, rate_hz, max_time_micros)

/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called (in hz)
  and the maximum time they are expected to take (in microseconds)
 */
const AP_Scheduler::Task Copter::scheduler_tasks[] = {
    SCHED_TASK(rc_loop,              100,    130),
    SCHED_TASK(throttle_loop,         50,     75),
    SCHED_TASK(update_GPS,            50,    200),
#if OPTFLOW == ENABLED
    SCHED_TASK(update_optical_flow,  200,    160),
#endif
    SCHED_TASK(update_batt_compass,   10,    120),
    SCHED_TASK(read_aux_switches,     10,     50),
    SCHED_TASK(arm_motors_check,      10,     50),
    SCHED_TASK(auto_disarm_check,     10,     50),
    SCHED_TASK(auto_trim,             10,     75),
    SCHED_TASK(read_rangefinder,      20,    100),
    SCHED_TASK(update_altitude,       10,    100),
    SCHED_TASK(run_nav_updates,       50,    100),
    SCHED_TASK(update_throttle_hover,100,     90),
    SCHED_TASK(three_hz_loop,          3,     75),
    SCHED_TASK(compass_accumulate,   100,    100),
    SCHED_TASK(barometer_accumulate,  50,     90),
#if PRECISION_LANDING == ENABLED
    SCHED_TASK(update_precland,      400,     50),
#endif
#if FRAME_CONFIG == HELI_FRAME
    SCHED_TASK(check_dynamic_flight,  50,     75),
#endif
    SCHED_TASK(update_notify,         50,     90),
    SCHED_TASK(one_hz_loop,            1,    100),
    SCHED_TASK(ekf_check,             10,     75),
    SCHED_TASK(landinggear_update,    10,     75),
    SCHED_TASK(lost_vehicle_check,    10,     50),
    SCHED_TASK(gcs_check_input,      400,    180),
    SCHED_TASK(gcs_send_heartbeat,     1,    110),
    SCHED_TASK(gcs_send_deferred,     50,    550),
    SCHED_TASK(gcs_data_stream_send,  50,    550),
    SCHED_TASK(update_mount,          50,     75),
    SCHED_TASK(update_trigger,        50,     75),
    SCHED_TASK(ten_hz_logging_loop,   10,    350),
    SCHED_TASK(twentyfive_hz_logging, 25,    110),
    SCHED_TASK(dataflash_periodic,    400,    300),
    SCHED_TASK(perf_update,           0.1,    75),
    SCHED_TASK(read_receiver_rssi,    10,     75),
    SCHED_TASK(rpm_update,            10,    200),
    SCHED_TASK(compass_cal_update,   100,    100),
    SCHED_TASK(accel_cal_update,      10,    100),
#if ADSB_ENABLED == ENABLED
    SCHED_TASK(avoidance_adsb_update, 10,    100),
#endif
#if ADVANCED_FAILSAFE == ENABLED
    SCHED_TASK(afs_fs_check,          10,    100),
#endif
    SCHED_TASK(terrain_update,        10,    100),
#if EPM_ENABLED == ENABLED
    SCHED_TASK(epm_update,            10,     75),
#endif
#ifdef USERHOOK_FASTLOOP
    SCHED_TASK(userhook_FastLoop,    100,     75),
#endif
#ifdef USERHOOK_50HZLOOP
    SCHED_TASK(userhook_50Hz,         50,     75),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    SCHED_TASK(userhook_MediumLoop,   10,     75),
#endif
#ifdef USERHOOK_SLOWLOOP
    SCHED_TASK(userhook_SlowLoop,     3.3,    75),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    SCHED_TASK(userhook_SuperSlowLoop, 1,   75),
#endif
    SCHED_TASK(button_update,          5,    100),
};


void Copter::setup() 
{
    cliSerial = hal.console;

    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    // setup storage layout for copter
    StorageManager::set_layout_copter();

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));

    // setup initial performance counters
    perf_info_reset();
    fast_loopTimer = AP_HAL::micros();

    buffer_ctr=0;   //mq tera
    hal.uartE->begin(921600);  //added by MQ for teraranger
    hal.uartE->println("BBB");
    hal.uartE->printf("PPP");

}

/*
  if the compass is enabled then try to accumulate a reading
 */
void Copter::compass_accumulate(void)
{
    if (g.compass_enabled) {
        compass.accumulate();
    }
}

/*
  try to accumulate a baro reading
 */
void Copter::barometer_accumulate(void)
{
    barometer.accumulate();
}

void Copter::perf_update(void)
{
    if (should_log(MASK_LOG_PM))
        Log_Write_Performance();
    if (scheduler.debug()) {
        gcs_send_text_fmt(MAV_SEVERITY_WARNING, "PERF: %u/%u %lu %lu\n",
                          (unsigned)perf_info_get_num_long_running(),
                          (unsigned)perf_info_get_num_loops(),
                          (unsigned long)perf_info_get_max_time(),
                          (unsigned long)perf_info_get_min_time());
    }
    perf_info_reset();
    pmTest1 = 0;
}

void Copter::loop()
{
    // wait for an INS sample
    ins.wait_for_sample();

    uint32_t timer = micros();

    // check loop time
    perf_info_check_loop_time(timer - fast_loopTimer);

    // used by PI Loops
    G_Dt                    = (float)(timer - fast_loopTimer) / 1000000.0f;
    fast_loopTimer          = timer;

    // for mainloop failure monitoring
    mainLoop_count++;

    // Execute the fast loop
    // ---------------------
    fast_loop();

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    uint32_t time_available = (timer + MAIN_LOOP_MICROS) - micros();
    scheduler.run(time_available > MAIN_LOOP_MICROS ? 0u : time_available);
}


// Main loop - 400hz
void Copter::fast_loop()
{

    // IMU DCM Algorithm
    // --------------------
    read_AHRS();

    // run low level rate controllers that only require IMU data
    attitude_control.rate_controller_run();
    
#if FRAME_CONFIG == HELI_FRAME
    update_heli_control_dynamics();
#endif //HELI_FRAME

    // send outputs to the motors library
    motors_output();

    // Inertial Nav
    // --------------------
    read_inertia();

    // check if ekf has reset target heading
    check_ekf_yaw_reset();

    // run the attitude controllers
    update_flight_mode();

    // update home from EKF if necessary
    update_home_from_EKF();

    // check if we've landed or crashed
    update_land_and_crash_detectors();

#if MOUNT == ENABLED
    // camera mount's fast update
    camera_mount.update_fast();
#endif

    // log sensor health
    if (should_log(MASK_LOG_ANY)) {
        Log_Sensor_Health();

    }


     //process_input_teraranger2();   //MQ, teraranger
     //hal.uartE->printf("BBB");       //mq tera

}

// rc_loops - reads user input from transmitter/receiver
// called at 100hz
void Copter::rc_loop()
{
    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    read_control_switch();
}

// throttle_loop - should be run at 50 hz
// ---------------------------
void Copter::throttle_loop()
{
    // update throttle_low_comp value (controls priority of throttle vs attitude control)
    update_throttle_thr_mix();

    // check auto_armed status
    update_auto_armed();

#if FRAME_CONFIG == HELI_FRAME
    // update rotor speed
    heli_update_rotor_speed_targets();

    // update trad heli swash plate movement
    heli_update_landing_swash();
#endif

    // compensate for ground effect (if enabled)
    update_ground_effect_detector();

    process_input_teraranger2();   //MQ, teraranger,set at 100hz

}

// update_mount - update camera mount position
// should be run at 50hz
void Copter::update_mount()
{
#if MOUNT == ENABLED
    // update camera mount's position
    camera_mount.update();
#endif
}

// update camera trigger
void Copter::update_trigger(void)
{
#if CAMERA == ENABLED
    camera.trigger_pic_cleanup();
    if (camera.check_trigger_pin()) {
        gcs_send_message(MSG_CAMERA_FEEDBACK);
        if (should_log(MASK_LOG_CAMERA)) {
            DataFlash.Log_Write_Camera(ahrs, gps, current_loc);
        }
    }    
#endif
}

// update_batt_compass - read battery and compass
// should be called at 10hz
void Copter::update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    read_battery();

    if(g.compass_enabled) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle(motors.get_throttle());
        compass.read();
        // log compass information
        if (should_log(MASK_LOG_COMPASS) && !ahrs.have_ekf_logging()) {
            DataFlash.Log_Write_Compass(compass);
        }
    }
}

// ten_hz_logging_loop
// should be run at 10hz
void Copter::ten_hz_logging_loop()
{
    // log attitude data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        DataFlash.Log_Write_Rate(ahrs, motors, attitude_control, pos_control);
        if (should_log(MASK_LOG_PID)) {
            DataFlash.Log_Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDA_MSG, g.pid_accel_z.get_pid_info() );
        }
    }
    if (should_log(MASK_LOG_MOTBATT)) {
        Log_Write_MotBatt();
    }
    if (should_log(MASK_LOG_RCIN)) {
        DataFlash.Log_Write_RCIN();
        if (rssi.enabled()) {
            DataFlash.Log_Write_RSSI(rssi);
        }
    }
    if (should_log(MASK_LOG_RCOUT)) {
        DataFlash.Log_Write_RCOUT();
    }
    if (should_log(MASK_LOG_NTUN) && (mode_requires_GPS(control_mode) || landing_with_GPS())) {
        Log_Write_Nav_Tuning();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        DataFlash.Log_Write_Vibration(ins);
    }
    if (should_log(MASK_LOG_CTUN)) {
        attitude_control.control_monitor_log();
    }
#if FRAME_CONFIG == HELI_FRAME
    Log_Write_Heli();
#endif
}

// fifty_hz_logging_loop
// should be run at 50hz
void Copter::twentyfive_hz_logging()
{
#if HIL_MODE != HIL_MODE_DISABLED
    // HIL for a copter needs very fast update of the servo values
    gcs_send_message(MSG_RADIO_OUT);
#endif

#if HIL_MODE == HIL_MODE_DISABLED
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        DataFlash.Log_Write_Rate(ahrs, motors, attitude_control, pos_control);
        if (should_log(MASK_LOG_PID)) {
            DataFlash.Log_Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDA_MSG, g.pid_accel_z.get_pid_info() );
        }
    }

    // log IMU data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_IMU) && !should_log(MASK_LOG_IMU_RAW)) {
        DataFlash.Log_Write_IMU(ins);
    }
#endif

#if PRECISION_LANDING == ENABLED
    // log output
    Log_Write_Precland();
#endif
}

void Copter::dataflash_periodic(void)
{
    DataFlash.periodic_tasks();
}

// three_hz_loop - 3.3hz loop
void Copter::three_hz_loop()
{
    // check if we've lost contact with the ground station
    failsafe_gcs_check();

    // check if we've lost terrain data
    failsafe_terrain_check();

#if AC_FENCE == ENABLED
    // check if we have breached a fence
    fence_check();
#endif // AC_FENCE_ENABLED

#if SPRAYER == ENABLED
    sprayer.update();
#endif

    update_events();

    // update ch6 in flight tuning
    tuning();
}

// one_hz_loop - runs at 1Hz
void Copter::one_hz_loop()
{
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_AP_STATE, ap.value);
    }

    update_arming_checks();

    if (!motors.armed()) {
        // make it possible to change ahrs orientation at runtime during initial config
        ahrs.set_orientation();

        update_using_interlock();

#if FRAME_CONFIG != HELI_FRAME
        // check the user hasn't updated the frame orientation
        motors.set_frame_orientation(g.frame_orientation);

        // set all throttle channel settings
        motors.set_throttle_range(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
#endif
    }

    // update assigned functions and enable auxiliary servos
    RC_Channel_aux::enable_aux_servos();

    check_usb_mux();

    // update position controller alt limits
    update_poscon_alt_max();

    // enable/disable raw gyro/accel logging
    ins.set_raw_logging(should_log(MASK_LOG_IMU_RAW));

    // log terrain data
    terrain_logging();

    adsb.set_is_flying(!ap.land_complete);
}

// called at 50hz
void Copter::update_GPS(void)
{
    static uint32_t last_gps_reading[GPS_MAX_INSTANCES];   // time of last gps message
    bool gps_updated = false;

    gps.update();

    // log after every gps message
    for (uint8_t i=0; i<gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);

            // log GPS message
            if (should_log(MASK_LOG_GPS) && !ahrs.have_ekf_logging()) {
                DataFlash.Log_Write_GPS(gps, i);
            }

            gps_updated = true;
        }
    }

    if (gps_updated) {
        // set system time if necessary
        set_system_time_from_GPS();

        // checks to initialise home and take location based pictures
        if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {

#if CAMERA == ENABLED
            if (camera.update_location(current_loc, copter.ahrs) == true) {
                do_take_picture();
            }
#endif
        }
    }
}

void Copter::init_simple_bearing()
{
    // capture current cos_yaw and sin_yaw values
    simple_cos_yaw = ahrs.cos_yaw();
    simple_sin_yaw = ahrs.sin_yaw();

    // initialise super simple heading (i.e. heading towards home) to be 180 deg from simple mode heading
    super_simple_last_bearing = wrap_360_cd(ahrs.yaw_sensor+18000);
    super_simple_cos_yaw = simple_cos_yaw;
    super_simple_sin_yaw = simple_sin_yaw;

    // log the simple bearing to dataflash
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_INIT_SIMPLE_BEARING, ahrs.yaw_sensor);
    }
}

// update_simple_mode - rotates pilot input if we are in simple mode
void Copter::update_simple_mode(void)
{
    float rollx, pitchx;

    // exit immediately if no new radio frame or not in simple mode
    if (ap.simple_mode == 0 || !ap.new_radio_frame) {
        return;
    }

    // mark radio frame as consumed
    ap.new_radio_frame = false;

    if (ap.simple_mode == 1) {
        // rotate roll, pitch input by -initial simple heading (i.e. north facing)
        rollx = channel_roll->get_control_in()*simple_cos_yaw - channel_pitch->get_control_in()*simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*simple_sin_yaw + channel_pitch->get_control_in()*simple_cos_yaw;
    }else{
        // rotate roll, pitch input by -super simple heading (reverse of heading to home)
        rollx = channel_roll->get_control_in()*super_simple_cos_yaw - channel_pitch->get_control_in()*super_simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*super_simple_sin_yaw + channel_pitch->get_control_in()*super_simple_cos_yaw;
    }

    // rotate roll, pitch input from north facing to vehicle's perspective
    channel_roll->set_control_in(rollx*ahrs.cos_yaw() + pitchx*ahrs.sin_yaw());
    channel_pitch->set_control_in(-rollx*ahrs.sin_yaw() + pitchx*ahrs.cos_yaw());
}

// update_super_simple_bearing - adjusts simple bearing based on location
// should be called after home_bearing has been updated
void Copter::update_super_simple_bearing(bool force_update)
{
    // check if we are in super simple mode and at least 10m from home
    if(force_update || (ap.simple_mode == 2 && home_distance > SUPER_SIMPLE_RADIUS)) {
        // check the bearing to home has changed by at least 5 degrees
        if (labs(super_simple_last_bearing - home_bearing) > 500) {
            super_simple_last_bearing = home_bearing;
            float angle_rad = radians((super_simple_last_bearing+18000)/100);
            super_simple_cos_yaw = cosf(angle_rad);
            super_simple_sin_yaw = sinf(angle_rad);
        }
    }
}

void Copter::read_AHRS(void)
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before ahrs update
    gcs_check_input();
#endif

    ahrs.update();
}

// read baro and rangefinder altitude at 10hz
void Copter::update_altitude()
{
    // read in baro altitude
    read_barometer();

    // write altitude info to dataflash logs
    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
    }
}

AP_HAL_MAIN_CALLBACKS(&copter);


    bool Copter::process_input_teraranger()   //added by MQ for teraranger
{
    uart_T = hal.uartE;

    //distance_1 = 20;  //test test
    // read any available lines from the lidar
    //    if CR (i.e. \r), LF (\n) it means we have received a full packet so send for processing
    //    lines starting with # are ignored because this is the echo of a set-motor request which has no reply
    //    lines starting with ? are the echo back of our distance request followed by the sensed distance
    //        distance data appears after a <space>
    //    distance data is comma separated so we put into separate elements (i.e. <space>angle,distance)
    uint16_t count = 0;
    int16_t nbytes = uart_T->available();
    while (nbytes-- > 0) {
        char c = uart_T->read();
        // check for end of packet
        if (c == '\r' || c == '\n') {
            if ((element_len_T[0] > 0)) {
                if (process_reply_T()) {
                    count++;
                }
            }
            // clear buffers after processing
            clear_buffers_T();

        // if message starts with # ignore it
        } else if (c == '#' || ignore_reply_T) {
            ignore_reply_T = true;

        // if waiting for <space>
        } else if (c == 'T') {
            wait_for_H = true;
            //distance_1 = -1; //test test

        } else if (wait_for_H) {
            if (c == 'H') {
                wait_for_H = false;
                //distance_2 = -1; //test test
            }

        // if comma, move onto filling in 2nd element
        } else if (c == '\t') {
            if ((element_len_T[element_num_T] > 0)) {
                element_num_T++;
            } else {
                // don't support 3rd element so clear buffers
                clear_buffers_T();
                ignore_reply_T = true;  //test
            }

        // if part of a number, add to element buffer
        }

        else if (c == char(-1)) {
        
        }

         else if (isdigit(c)) {
            //distance_3 = 2;  //test test
            element_buf_T[element_num_T][element_len_T[element_num_T]] = c;
            element_len_T[element_num_T]++;
            if (element_len_T[element_num_T] >= sizeof(element_buf_T[element_num_T])-1) {
                // too long, discard the line
                clear_buffers_T();
                ignore_reply_T = true;
            }
        }
    }

    return (count > 0);
}


// process reply
bool Copter::process_reply_T()   //added by MQ for teraranger
{
    if (uart_T == nullptr) {
        return false;
    }

    bool success = false;

 
            distance_1 =  (float)atof(element_buf_T[0]);
            distance_2 =  (float)atof(element_buf_T[1]);
            distance_3 = (float)atof(element_buf_T[2]);
            distance_4 = (float)atof(element_buf_T[3]);
            
            uint8_t sector;
            if ((distance_1 != 0)&& (distance_2 != 0) && (distance_3 != 0)) {

                success = true;

               }

    // mark request as cleared
    return success;
}

// clear buffers ahead of processing next message
void Copter::clear_buffers_T()   //added by MQ for teraranger
{
    element_len_T[0] = 0;
    element_len_T[1] = 0;
    element_len_T[2] = 0;
    element_len_T[3] = 0;
    element_num_T = 0;
    memset(element_buf_T, 0, sizeof(element_buf_T));
}


uint8_t Copter::crc8(uint8_t *p, uint8_t len)   //MQ for teraranger
{
  uint16_t i;
  uint16_t crc = 0x0;
    const uint8_t crc_table[]={  0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23,
                                    0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41,
                                    0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf,
                                    0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd, 0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85,
                                    0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc,
                                    0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
                                    0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a, 0x27, 0x20,
                                    0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
                                    0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74,
                                    0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8,
                                    0xad, 0xaa, 0xa3, 0xa4, 0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6,
                                    0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
                                    0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10, 0x05, 0x02,
                                    0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34, 0x4e, 0x49, 0x40, 0x47,
                                    0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39,
                                    0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
                                    0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d,
                                    0x84, 0x83, 0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
                                    0xfa, 0xfd, 0xf4, 0xf3};   //mq for tera

  while (len--)
  {
    i = (crc ^ *p++) & 0xFF;
    crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
  }
  return crc & 0xFF;
}

bool Copter::process_input_teraranger2()  //mq FOR teraranger
{
    uart_T = hal.uartE;

static int count = 0;    //test test
int16_t nbytes = uart_T->available();
 while (nbytes-- > 0) {

    char single_character = uart_T->read();

 if (single_character != 'T' && buffer_ctr < 19)
  {
    // not begin of serial feed so add char to buffer
    input_buffer[buffer_ctr++] = single_character;
    //return true;
    //distance_1 = count++;   //test test
  }


  else if (single_character == 'T')
  {
    
    if (buffer_ctr == 19)
    {
      // end of feed, calculate
      int16_t crc = crc8(input_buffer, 18);

      if (crc == input_buffer[18])
      {
        int16_t range0 = input_buffer[2] << 8;
        range0 |= input_buffer[3];
        int16_t range1 = input_buffer[4] << 8;
        range1 |= input_buffer[5];
        int16_t range2 = input_buffer[6] << 8;
        range2 |= input_buffer[7];
        int16_t range3 = input_buffer[8] << 8;
        range3 |= input_buffer[9];
        int16_t range4 = input_buffer[10] << 8;
        range4 |= input_buffer[11];
        int16_t range5 = input_buffer[12] << 8;
        range5 |= input_buffer[13];
        int16_t range6 = input_buffer[14] << 8;
        range6 |= input_buffer[15];
        int16_t range7 = input_buffer[16] << 8;
        range7 |= input_buffer[17];

        if (range0 < 14000 && range0 > 200)
        {
            distance_4 = range0 * 0.001;
        }
    else{
            distance_4 = 100;
    
        }
        if (range1 < 14000 && range1 > 200)
        {
             distance_2 = range1 * 0.001;
        }
        else{

            distance_2 = 100;
        }
        if (range2 < 14000 && range2 > 200)
        {
              distance_3 = range2 * 0.001;
        }
    else{

            distance_3 = 100;
        }
        if (range3 < 14000 && range3 > 200)
        {
             distance_1 = range3 * 0.001;
        }
    else{
         distance_1 = 100;

        }
        if (range4 < 14000 && range4 > 200)
        {

        }
    else{

        }
        if (range5 < 14000 && range5 > 200)
        {
        }
    else{

        }
        if (range6 < 14000 && range6 > 200)
        {
        }
    else{
        }
        if (range7 < 14000 && range7 > 200)
        {
    }
    else{

    }


      
      }
      else
      {
        distance_2 = 100;
      }
    }
    else
    {
        distance_3 = 100;
      //ROS_DEBUG("[%s] reveived T but did not expect it, reset buffer without evaluating data",
               //ros::this_node::getName().c_str());
    }

    buffer_ctr = 0;  //reset
    memset(&input_buffer, '\0', BUFFER_SIZE);
    input_buffer[buffer_ctr++] = 'T';
  }
  else
  {
    //distance_1 = 124;
    //ROS_DEBUG("[%s] buffer_overflowed without receiving T, reset input_buffer", ros::this_node::getName().c_str());
  }

 
  // clear struct
  //bzero(&input_buffer, BUFFER_SIZE);

}

  return true;

}  //