#include "mbed.h"
#include "rtos.h"
#include "MODSERIAL.h"

// ONLINE COMPILER
#include "telemetry.h"
#include "telemetry-mbed.h"

/* LAB COMPUTER COMPILER */
//#include "telemetry/server-cpp/telemetry.h"
//#include "telemetry/server-cpp/telemetry-mbed.h"

MODSERIAL telemetry_serial (PTA2, PTA1);

DigitalIn attempt_1(PTE20);
DigitalIn attempt_2(PTE21);
DigitalIn attempt_3(PTE22);

DigitalOut FUCK_THIS(PTE5);
DigitalOut AWWW_YEAH(PTE4);

DigitalOut led_green(LED_GREEN);
DigitalOut led_red(LED_RED);
DigitalOut led_blue(LED_BLUE);
Ticker flipper;
PwmOut servo(PTA12);
PwmOut motor_left(PTD4); // IN_LS1
PwmOut brake_left(PTA4); // IN_HS1

PwmOut motor_right(PTA5); // IN_LS2 
PwmOut brake_right(PTC9); // IN_HS2

AnalogIn camera1(PTB3);  // First camera - Output from camera
DigitalOut CLK(PTB11);
DigitalOut SI(PTE2);
DigitalIn encoder(PTD5); // Left
//DigitalIn encoder(PTD0); // Right

void servoControl_race(int midpoint);
void servoControl_freescale(int midpoint);
float convert_to_velocity(float period);
void servoControl(int midpoint);
void dynamicPERIOD();

float target_velocity = 2.5f; // average time per phase we are targetting (in ms)
float motor_pwm = 0.1f; // 0.07
uint16_t linescan_buffer[128];
uint16_t track_buffer[128];
uint16_t low_pass_filter[128];
uint16_t high_pass_filter[127];
uint16_t normalized_buffer[128];

float low_pwm = 0.21f;
float high_pwm = 0.24f;
float differtial_turning = .045f;

float k_p = 1.0f;
int turning_threshold = 25;
int threshold = 1200*2;
int servo_update_ms = 16;

int midpoint;
int midpoint_prev1;
int midpoint_prev2;

int min_index_hold;
int max_index_hold;
int min_value_hold;
int max_value_hold;
bool left_edge_hold;
bool right_edge_hold;
bool finish_line = false;

float exposure = 0;
float camera_normalization[128] = {25,35,37,44,51,56,58,60,66,69,
                                   71,73,77,79,81,83,86,87,89,91,
                                   93,93,95,96,97,97,97,98,99,100,
                                   100,100,100,100,100,100,100,100,100,100,
                                   100,100,100,100,100,100,100,100,100,100,
                                   100,100,100,100,100,100,100,100,100,100,
                                   100,100,100,100,100,100,100,100,100,100,
                                   100,100,100,100,100,100,100,100,100,100,
                                   100,100,100,100,100,100,100,100,100,100,
                                   100,100,99,99,98,97,95,94,93,92,
                                   91,90,89,87,85,84,82,80,79,77,
                                   76,75,75,75,74,73,72,70,68,64,
                                   57,53,44,42,37,31,24,22};
const int LEFTMOTOR = 0;
const int RIGHTMOTOR = 1;
const float CENTER = 0.078f;
const float LEFT = 0.096f;
const float RIGHT = 0.06f;
float current_velocity;
int buffer_pointer;
int PERIOD = 1;

void motorControl(int LeftOrRight, float speed)
{
    if(LeftOrRight == LEFTMOTOR) // left
    {
        motor_left.write(speed);
        brake_left.write(0.0f);
    }
    else
    {
        motor_right.write(speed);
        brake_right.write(0.0f);
    }
}

void feedback_loop(float current_average){
    current_velocity = convert_to_velocity(current_average);
    float error = target_velocity - current_velocity;
    error = error / 10 ;
    float k_const = 0.3f;
    motor_pwm = k_const * error + motor_pwm;
    if(motor_pwm >= .3f){
        motor_pwm = .3f;
    }
    else if(motor_pwm < 0.0f){
        motor_pwm = .01f;
    }
    motorControl(LEFTMOTOR, motor_pwm);
    motorControl(RIGHTMOTOR, motor_pwm);
}

float convert_to_velocity(float period) { // s / rev
    float circ = 0.1602f; // in m / rev
    return circ / period;
}

void speed_control(void const *args){
    Timer t;
    float buffer[6]; // = {0,0,0,0,0,0};
    buffer_pointer = 0;
    int state = -1;
    while(1) {
              // if it stopped
        if(t.read_ms() > 200){
            // set speed to be 0.1f if it is completely stopped
            motorControl(LEFTMOTOR, 0.1f);
            motorControl(RIGHTMOTOR, 0.1f);
            t.reset();
        }
        // Initialize state & start timer
        if (state == -1){
            state = encoder;
            t.start();
        }

        else if(encoder != state){
            buffer[buffer_pointer] = t.read_us();
            buffer_pointer++;
            state = encoder;
            t.reset();
        }
        if (buffer_pointer == 6){
            float avg = buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5];
            feedback_loop(avg/1000000); //convert us to seconds.
            buffer_pointer = 0;
            t.reset();
        }
    }
}


void freescale()
{
    PERIOD = 1; // us
    int MAXT = 129;
    int integrating = 0; 
    int i = 0;
    Timer t;
    t.start();
    CLK = 0;
    t.reset();
    int min_value = 65535;
    int max_value = 0;
    int min_index = 0;
    int max_index = 0;
    threshold = 1200*2;
    int bias = 32768;
    bool left_edge = false;
    bool right_edge = false;

    int counter = 0;
    int finish_line_counter = 0;

    int local_min = 0;
    int local_max = 0;
    
    while(1)
    {
        // From checking
        CLK = 0;
        SI = integrating == 0; 
        //wait_us(PERIOD);
        
        CLK = 1;
        //wait_us(PERIOD);
        SI = 0;
        ++integrating;

        min_value = 65535;
        max_value = 0;

        min_index = 0;
        max_index = 0;

        left_edge = false;
        right_edge = false;

        counter = 0;
        
        finish_line_counter = 0;

        local_max = 0;
        local_min = 0;

        // We are integrating
        if(integrating < MAXT)
        {
            linescan_buffer[i] = (uint16_t)((float)camera1 * 65535);
            ++i;
        }
        else if(integrating == MAXT)
        {   
            // Normalize Camera Data 
            for (int j = 0; j < 128; ++j) {
                if((linescan_buffer[j] * (100.0f/camera_normalization[j])) > 65535){
                    normalized_buffer[j] = 65535;
                }
                else{
                    normalized_buffer[j] = linescan_buffer[j] * (100.0f/camera_normalization[j]);
                }
            }
            
            // Low Pass Filter
            for (int k = 0; k < 128; ++k) {
                if(k > 0 && k < 127){
                    low_pass_filter[k] = (1.0f/3.0f) * (normalized_buffer[k-1] + normalized_buffer[k] + normalized_buffer[k+1]);
                }
                else if(k == 0){
                    low_pass_filter[k] = (1.0f/2.0f) * (normalized_buffer[k] + normalized_buffer[k+1]);
                }
                else if(k == 127){
                    low_pass_filter[k] = (1.0f/2.0f) * (normalized_buffer[k-1] + normalized_buffer[k]);
                }
                // if((low_pass_filter[k] > 4768) && (low_pass_filter[k] < 10768)){
                //     counter += 1;
                // }
                // else{
                //     if((counter >= 14) && (counter <= 20)){
                //         finish_line_counter += 1;
                //     }
                //     counter = 0;
                // }
            }

            // if(finish_line_counter == 2){
            //     finish_line = true;
            //     low_pwm = 0.0f;
            //     high_pwm = 0.0f;
            //}
            
            // High Pass Filter
            for (int j = 0; j < 127; ++j) {
                high_pass_filter[j] = ((low_pass_filter[j]/2) - (low_pass_filter[j+1]/2)) + bias;
            }
        
            // After high pass, PIXEL 63 is now center.
            
            for (int j = 6; j < 122; ++j) {
                if (high_pass_filter[j] > max_value){
                    max_value = high_pass_filter[j];
                    max_index = j;
                }
                if (high_pass_filter[j] < min_value){
                    min_value = high_pass_filter[j];
                    min_index = j;
                }
            }

            //     //FINISH LINE CHECKING
            //     if((high_pass_filter[j] > (bias + threshold)) && (high_pass_filter[j] > high_pass_filter[j+1]) && (high_pass_filter[j] > high_pass_filter[j-1])){
            //         local_max += 1;
            //     }
            //     else if((high_pass_filter[j] < (bias - threshold)) && (high_pass_filter[j] < high_pass_filter[j+1]) && (high_pass_filter[j] < high_pass_filter[j-1])){
            //         local_min += 1;
            //     }
            // }

            // if (local_min == 3 && local_max == 3){
            //     finish_line = true;
            //     low_pwm = 0.0f;
            //     high_pwm = 0.0f;
            // }

            // Doesn't account for crossings.
            if (max_value > (bias + threshold)){
                right_edge = true;
            }

            if (min_value < (bias - threshold)){
                left_edge = true;
            }
            
            if (right_edge && left_edge && (min_index > max_index))
            {
                // Found dying edge
                if(min_index - max_index < 6)
                {
                    if(-min_value > max_value)
                    {
                        right_edge = false;
                    }
                    else
                    {
                        left_edge = false;
                    }
                }
                else
                {
                    // Do something more than invalidate the edge
                    if(-min_value > max_value)
                    {
                        right_edge = false;
                    }
                    else
                    {
                        left_edge = false;
                    }
                }                
            }
            if (left_edge && right_edge){
                midpoint = (min_index + max_index)/2;
            }
            else if (left_edge){
                midpoint = (min_index + 54);
            }
            else if (right_edge){
                midpoint = (max_index - 54);
            }
            else{
                midpoint = (midpoint_prev1 - midpoint_prev2) + midpoint_prev1;
            }
                    
            midpoint_prev2 = midpoint_prev1;
            midpoint_prev1 = midpoint;

            min_value_hold = min_value - bias;
            max_value_hold = max_value - bias;
            min_index_hold = min_index;
            max_index_hold = max_index;
            left_edge_hold = left_edge;
            right_edge_hold = right_edge;
            servoControl_freescale(midpoint);
        }
        if(t.read_ms() >= servo_update_ms)//integrating > INTEGRATIONTIME)
        {   
            t.reset();
            integrating = 0;
            i = 0;
        }
    }
}
/*
    There are 128 possible mid points (prefer 100)
    Servo PWM varies from LEFT = 0.09f; RIGHT = 0.06f; -> 0.03
    PWM 0.03 / 128 = 0.00234375
*/
void servoControl_freescale(int midpoint)
{
    // Assume the center is 64 (1 to 127 pixels)
    int center = 64;
    const float UNIT = 0.03f / 127;
    float change;
    if(midpoint < 0)
    {
        midpoint = 0;
    }
    else if (midpoint > 128)
    {
        midpoint = 127;
    }
        
        /* If we are far away from the mid point we can:
        1) Incrase kp
        2) decrease speed
        
        If we are write on the track
        1) kp = 1
        2) increase speed
        
        */
        
        if(midpoint < center)
        {
            // RIGHT TURN
            if(center - midpoint > turning_threshold)
            {
                motor_left.write(low_pwm + differtial_turning); // Assume we are feeding constant velocity
                motor_right.write(low_pwm);
            }
            else
            {
                motor_left.write(high_pwm); // Assume we are feeding constant velocity
                motor_right.write(high_pwm);
            }
        }
        else
        {
            if(midpoint - center > turning_threshold)
            {
                motor_left.write(low_pwm); // Assume we are feeding constant velocity
                motor_right.write(low_pwm + differtial_turning);
            }
            else
            {
                motor_left.write(high_pwm); // Assume we are feeding constant velocity
                motor_right.write(high_pwm);
            }
        }
        
        
        
    if(midpoint < center)
    {
        change = -UNIT * (center - midpoint) * k_p;
    }
    else
    {
        change = -UNIT * (center - midpoint) * k_p;
    }
    
    // Ensure we don't go past servo limits
    if((change + CENTER) > LEFT)
    {
        servo.write(LEFT);
    }
    else if((change + CENTER) < RIGHT)
    {
        servo.write(RIGHT);
    }
    else
    {
        servo.write(CENTER + change);
    }
}


void BlueSMiRF(void const *args)
{
    telemetry_serial.format();
    telemetry_serial.baud(115200);
    Thread::wait(10000);
    
    telemetry_serial.printf("CONNECTED\r\n");
    // while(1)
    // {
    //     if(telemetry_serial.writeable())
    //     {
            
    //     }
        
    //     Thread::wait(100);
    // }
}


void telemetry_thread(void const *args){
    //telemetry_serial.format();
    telemetry_serial.baud(115200);
    telemetry_serial.printf("CONNECTED\r\n");
    
    telemetry::MbedHal telemetry_hal(telemetry_serial);
    telemetry::Telemetry telemetry_obj(telemetry_hal);
    
    telemetry::Numeric<uint32_t> tele_time_ms(telemetry_obj, "time", "Time", "ms", 0);
    telemetry::NumericArray<uint16_t, 128> tele_linescan(telemetry_obj, "linescan", "Linescan", "ADC", 0);
    //telemetry::NumericArray<uint8_t, 128> tele_linescan_2(telemetry_obj, "linescan_2", "Linescan", "ADC", 0);
    // telemetry::NumericArray<uint16_t, 128> tele_normalized(telemetry_obj, "normalized_buffer", "Normalized Linescan", "ADC", 0);
    //telemetry::NumericArray<uint8_t, 128> tele_normalized_2(telemetry_obj, "normalized_buffer_2", "Normalized Linescan 8bit", "ADC", 0);  
    //telemetry::NumericArray<uint16_t, 128> tele_low_pass(telemetry_obj, "low_pass_filter", "Low Pass Filter", "ADC", 0);
    //telemetry::NumericArray<uint16_t, 127> tele_high_pass(telemetry_obj, "high_pass_filter", "High Pass Filter", "ADC", 0);
    //telemetry::Numeric<float> tele_low_pwm(telemetry_obj, "pwm", "Low PWM", "adc", 0);
    // telemetry::Numeric<float> tele_high_pwm(telemetry_obj, "pwm", "High PWM", "adc", 0);
    // telemetry::Numeric<float> tele_k(telemetry_obj, "k", "K Constant", "float", 0);
    // telemetry::Numeric<float> tele_differential(telemetry_obj, "differtial_turning", "Differential Turning", "float", 0);
    // telemetry::Numeric<uint32_t> tele_turning(telemetry_obj, "turning_threshold", "Turning Threshold", "int", 0);
    // telemetry::Numeric<uint32_t> tele_threshold(telemetry_obj, "threshold", "High Pass Threshold", "int", 0);
    //telemetry::Numeric<uint16_t> tele_midpoint(telemetry_obj, "midpoint", "Midpoint", "pixel", 0);
    //telemetry::Numeric<uint16_t> tele_min_index(telemetry_obj, "min_index", "Min Index", "index", 0);
    //telemetry::Numeric<uint16_t> tele_max_index(telemetry_obj, "max_index", "Max Index", "index", 0);

    telemetry_obj.transmit_header();
    
    Timer timer;
    timer.start();
    int start = timer.read_ms();
    
    /* TUNING
    tele_low_pwm = low_pwm;
    tele_high_pwm = high_pwm;
    tele_k = k_p;
    tele_turning = turning_threshold;
    tele_threshold = threshold;
    tele_differential = differtial_turning;
    */

    telemetry_obj.do_io();
    Thread::wait(100);
    
    // Continuously Gather Data
    while(1){
        tele_time_ms = timer.read_ms() - start;
        
        for (int i = 0; i < 128; ++i){
            tele_linescan[i] = linescan_buffer[i];
        }

        /* 8bit linescan
        for (int i = 0; i < 128; ++i){
            tele_linescan_2[i] = (uint8_t)(linescan_buffer[i] * 255);
        }
        
        for (int i = 0; i < 128; ++i){
            if((tele_linescan_2[i] * (100.0f/camera_normalization[i])) > 255){
                tele_normalized_2[i] = 255;
            }
            else{
                tele_normalized_2[i] = tele_linescan_2[i] * (100.0f/camera_normalization[i]);
            }
        }
        */
                
                /*
        for (int i = 0; i < 128; ++i){
            tele_normalized[i] = normalized_buffer[i];
        }
        */

        /*
        for (int i = 0; i < 128; ++i){
            tele_low_pass[i] = low_pass_filter[i];
        } */        

        // for (int i = 0; i < 127; ++i){
        //     tele_high_pass[i] = high_pass_filter[i];
        // } 

        /*
        tele_midpoint = midpoint;
        tele_min_index = min_index;
        tele_max_index = max_index;
        */

        /*
        for (int i = 0; i < 128; ++i){
            tele_normalized[i] = normalized_buffer[i];
        }

        for (int i = 0; i < 128; ++i){
            tele_normalized_2[i] = normalized_buffer_2[i];
        }
        */

        /*
        tele_low_pwm = low_pwm;
        tele_high_pwm = high_pwm;
        tele_turning = turning_threshold;
        tele_threshold = threshold;
        tele_k = k_p;
        tele_differential = differtial_turning;
        */
        
        telemetry_obj.do_io();
        
        /*
        low_pwm = tele_low_pwm;
        high_pwm = tele_high_pwm;
        turning_threshold = tele_turning;
        threshold = tele_threshold;
        k_p = tele_k;
        differtial_turning = tele_differential;
        */
                
        Thread::wait(200);
    }
}

int main() {
    telemetry_serial.printf("--- Team 7 (Quinn, Byung, Frank), EE 192 ---\r\n");
    telemetry_serial.printf("Built " __DATE__ " " __TIME__ "\r\n");  
    
    wait(2);
    FUCK_THIS.write(0);
    AWWW_YEAH.write(1);
    led_blue.write(0);
    led_red.write(0);
    led_green.write(0);

    if(!attempt_1.read()){
        led_green.write(1); // PURPLE LIGHT -- BEST CASE

        /*
        low_pwm = .27f;
        high_pwm = .42f;
        k_p = 1.5f;
        turning_threshold = 25;
        threshold = 2400;
        servo_update_ms = 12;
        */

        low_pwm = .375f;
        high_pwm = .351f;
        differtial_turning = .06f;
        k_p = 1.5f;
        turning_threshold = 24;
        threshold = 2400;
        servo_update_ms = 12;
    }
    else if(!attempt_2.read()){ // YELLOW LIGHT -- SUB-OPTIMAL CASE
        led_blue.write(1); 

        low_pwm = .276f;
        high_pwm = .345f;
        differtial_turning = .048f;
        k_p = 1.5f;
        turning_threshold = 24;
        threshold = 2400;
        servo_update_ms = 12;
    }
    else if(!attempt_3.read()){
        led_red.write(1); // CYAN LIGHT -- SAFETY CASE

        low_pwm = .20f;
        high_pwm = .25f;
        differtial_turning = 0.0f;
        k_p = 1.0f;
        turning_threshold = 25;
        threshold = 2400;
        servo_update_ms = 16;
    }
    else{
        led_green.write(1); // PURPLE LIGHT -- BEST CASE

        low_pwm = .375f;
        high_pwm = .351f;
        differtial_turning = .06f;
        k_p = 1.5f;
        turning_threshold = 24;
        threshold = 2400;
        servo_update_ms = 12;
    }
    
    // Initializing
    servo.period(.02f);
    servo.write(0.075f);    
    motor_left.period(.001f);
    brake_left.period(.001f);
    motor_left.write(low_pwm); // Assume we are feeding constant velocity
    brake_left.write(0.0f);
    motor_right.period(0.001f);
    brake_right.period(0.001f);
    motor_right.write(low_pwm);
    brake_right.write(0.0f);
    //Thread bThread(BlueSMiRF);
    Thread teleThread(telemetry_thread);
    //mainControl();
    
    //Thread speed(speed_control);
    freescale();
}
