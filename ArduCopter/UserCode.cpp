/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    teraranger_enable = true;

    if (teraranger_enable)
    {
    buffer_ctr=0;   //mq tera
    hal.uartE->begin(921600);  //added by MQ for teraranger
    hal.uartE->println("BBB");
    //hal.uartE->printf("PPP");/* code */
    }


    //tera_distance_right = 30.0; //testing

}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    
    if (teraranger_enable)
    {
    	process_input_teraranger2();   //MQ, teraranger,set at 100hz 
    }
    
    //tera_distance_right = 20.0;
    //tera_distance_left = 20.0;
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{

	    if (teraranger_enable)
    {
    	process_input_teraranger2();   //MQ, teraranger,set at 50hz 
    }
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{	
	int16_t sonar_fence = 80; //mq, in cm

	 if(rangefinder.has_data(1))                   //mq, object detection}
    {   

        /*
        sonar_distance10 = sonar_distance9; //apply median filter, mq
        sonar_distance9 = sonar_distance8; 
        sonar_distance8 = sonar_distance7; 
        sonar_distance7 = sonar_distance6; 
        sonar_distance6 = sonar_distance5; 
        sonar_distance5 = sonar_distance4; 
        sonar_distance4 = sonar_distance3; 
        */         
        sonar_distance3 = sonar_distance2;
        sonar_distance2 = sonar_distance1;
        sonar_distance1 = rangefinder.distance_cm(1); 
        
        
        if (sonar_distance1 < sonar_fence)      //mq, simple filter test
            {   if (sonar_distance2 > sonar_fence)
                {
                  sonar_distance_used = sonar_distance2;  
                }
                else    {sonar_distance_used = sonar_distance1;}
            }
        else    {sonar_distance_used = sonar_distance1;}

        

        //int16_t sonar_distance_sum = sonar_distance1 + sonar_distance2 + sonar_distance3 + sonar_distance4 + sonar_distance5 + sonar_distance6 + sonar_distance7 + sonar_distance8 + sonar_distance9 + sonar_distance10;
        //sonar_distance_used = sonar_distance_sum / 10;
    }
    else    {sonar_distance_used = 1000;}       //mq, object detection
    
    //sonar_distance_used = 1000;  //testing only
    estimate_distance(); 
    // put your 10Hz code here
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


/*

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

*/
void Copter::estimate_distance()        //mq kalman filter for sideway distance estimation MQ
{
    float system_covariance_Q = 20.0;   //in cm/s
    float measurement_covariance_R = 100.0;  //in cm
    float dt = 0.05 ;//20hz loop for rangefinder

    //kalman filter loop
    float P_k_minus = P_k_plus + system_covariance_Q;
    float K_k = P_k_minus/(P_k_plus+measurement_covariance_R);
    distance_k_plus = (1-K_k)*(distance_k_plus - velocity_for_distance_estimation * dt) + K_k*sonar_distance_used;
    P_k_plus = (1- K_k) * P_k_minus;

    //get roll velocity for next loop
    const Vector3f& vel = inertial_nav.get_velocity();    //from drift mode
    float roll_vel =  vel.y * ahrs.cos_yaw() - vel.x * ahrs.sin_yaw(); // mq, body roll vel in cm/s
    velocity_for_distance_estimation = roll_vel;
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
            //distance_3 = (float)atof(element_buf_T[2]);
            //distance_4 = (float)atof(element_buf_T[3]);
            
            uint8_t sector;
            /*
             ((distance_1 != 0)&& (distance_2 != 0) && (distance_3 != 0)) {

                success = true;

               }
				*/

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
    //tera_distance_right = 10.0;  //testing
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
            //distance_4 = range0 * 0.001;
        }
    else{
           // distance_4 = 100;
    
        }
        if (range1 < 14000 && range1 > 200)
        {
             //tera_distance_right = range1 * 0.1;   //in cm
        }
        else{

            //tera_distance_right = 100.0;
        }
        if (range2 < 14000 && range2 > 200)
        {
             tera_distance_left = range2 * 0.1; //distance_3 = range2 * 0.001;
        }
    else{

           tera_distance_left = 1000.0; //distance_3 = 100;
        }
        if (range3 < 14000 && range3 > 200)
        {
             //distance_1 = range3 * 0.001;
        }
    else{
         //distance_1 = 100;

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
        	tera_distance_right = range7 * 0.1; //for safmc setup
   		 }
    else{
    	tera_distance_right = 1000.0;

    	}


      
      }
      else
      {
      	    tera_distance_right = 1050.0;  //for safety, in case teraranger no value
   			 tera_distance_left = 1050.0;
        //distance_2 = 100;
      }
    }
    else
    {

    	    //tera_distance_right = 600.0;  //for safety, in case teraranger no value
    		//tera_distance_left = 600.0;
        //distance_3 = 100;
      //ROS_DEBUG("[%s] reveived T but did not expect it, reset buffer without evaluating data",
               //ros::this_node::getName().c_str());
    }

    buffer_ctr = 0;  //reset
    memset(&input_buffer, '\0', BUFFER_SIZE);
    input_buffer[buffer_ctr++] = 'T';
  }
  else
  {	
  	uart_T->println("BBB"); //mq trying to correct the problem of not receiving value
    //distance_1 = 124;
    //ROS_DEBUG("[%s] buffer_overflowed without receiving T, reset input_buffer", ros::this_node::getName().c_str());
    tera_distance_right = 1100.0;  //for safety, in case teraranger no value
    tera_distance_left = 1100.0;
  }

 
  // clear struct
  //bzero(&input_buffer, BUFFER_SIZE);

}

  return true;

}  //
