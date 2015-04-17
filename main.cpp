#include "mbed.h"
#include "rtos.h"
#include "MODSERIAL.h"

/* ONLINE COMPILER */
#include "telemetry.h"
#include "telemetry-mbed.h"

/* LAB COMPUTER COMPILER */
//#include "telemetry/server-cpp/telemetry.h"
//#include "telemetry/server-cpp/telemetry-mbed.h"

MODSERIAL telemetry_serial (PTA2, PTA1);

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

float target_velocity = 8; // average time per phase we are targetting (in ms)
float motor_pwm = 0.1f; // 0.07
uint16_t linescan_buffer[128];
uint16_t track_buffer[128];
uint16_t moving_average[128];
uint16_t normalized_buffer[128];
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
const float CENTER = 0.075f;
const float LEFT = 0.09f;
const float RIGHT = 0.06f;
float current_velocity;
int midpoint = 55;
int buffer_pointer;
int PERIOD = 1;

int LEFT_EDGE = -1;
int RIGHT_EDGE = -1;

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
    float k_const = 0.03f;
    motor_pwm = k_const * error + motor_pwm;
    if(motor_pwm >= .3f){
        motor_pwm = .3f;
    }
    else if(motor_pwm < 0.0f){
        motor_pwm = .01f;
    }
    
    //motor.write(motor_pwm);
    motorControl(LEFTMOTOR, motor_pwm);
    motorControl(RIGHTMOTOR, motor_pwm);
}

float convert_to_velocity(float period) { // s / rev
    float circ = 0.1602f; // in m / rev
    return circ / period;
}

void speed_control(Timer timer){
    Timer t;
    float buffer[18]; // = {0,0,0,0,0,0};
    buffer_pointer = 0;
    int state = -1;
    //motor.write(motor_pwm);
    //motorControl(LEFTMOTOR, motor_pwm);
    //motorControl(RIGHTMOTOR, motor_pwm);
    current_velocity = -1; //testing
    while(timer.read_ms() < 16) {
        // Initialize state & start timer
        if (state == -1){
            state = encoder;
            t.start();
        }
        // if it stopped
        else if(t.read_ms() > 1602){
            
            t.reset();
        }
        else if(encoder != state){
            buffer[buffer_pointer] = t.read_us();
            buffer_pointer++;
            state = encoder;
            current_velocity = -3;
            t.reset();
        }
        if (buffer_pointer >= 11){
            current_velocity = -2;
            float avg = ((buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5] + buffer[6] + buffer[7] + buffer[8] + buffer[9] + buffer[10] + buffer[11])/2);
            feedback_loop(avg/1000000); //convert us to seconds.
            buffer_pointer = 0;
            t.reset();
        }
    }
}
void mainControl(){
    // Minimum intergartion time will be
    // T = (1/maximum clk) * (n - 18) pixels
    // Ex, for 8MHz, T = 0.125us * ( 128 - 18)  
    PERIOD = 25; // us
    //const int SIZE = 128;
    int MAXT = 129;
    int integrating = 0; 
    int i = 0;
    float new_buffer[128];
    float moving_average[128];
    Timer t;
    t.start();
    CLK = 0;
    
    t.reset();
    while(1)
    {
        // From checking
        float maxValue = 0, minValue = 128;
        int maxIndex = 0, minIndex = 0;
        
        CLK = 0;
        SI = integrating == 0; 
        wait_us(PERIOD);        
        
        CLK = 1;
        wait_us(PERIOD);
        SI = 0;
        integrating++;

        // We are integrating
        if(integrating < MAXT)
        {
            linescan_buffer[i] = camera1;
            i++;
        }
        else if(integrating == MAXT)
        {
            
            for (int k = 1; k < 125; ++k) {
                moving_average[k] = (1.0f/3.0f) * (linescan_buffer[k-1] + linescan_buffer[k] + linescan_buffer[k+1]);
            }
            for (int j = 1; j < 125; ++j) {
                new_buffer[j] = moving_average[j] - moving_average[j-1];
            }
            
            for (int i = 10; i < 120; ++i) {
                if (new_buffer[i] > maxValue) {
                    maxValue = new_buffer[i];
                    maxIndex = i;
                }
            }
            for (int i = maxIndex; i < 120; ++i) {
                if (new_buffer[i] < minValue){
                    minValue = new_buffer[i];
                    minIndex = i;
                }
            }
            
            int change = midpoint - (maxIndex + minIndex)/2;
            change = change > 0? change : -change;
            const int WIDTH = 20;
            
          if(change < 40 && (minIndex - maxIndex) < WIDTH)
            {
                midpoint = (maxIndex + minIndex) / 2;
            }
            else
            {
                // mid is not found
                //midpoint = mid1 + (mid1 - mid2);
                //midpoint = midpoint;
            }
            servoControl(midpoint);
            //Speed control;

            /*
            else if(integrating > MAXT)
            {
                
                TOTALVAL = 0.0f;
                for(int i = 0; i < 128; i++)
                {
                    TOTALVAL += buffer[i];
                }
                exposure = TOTALVAL / 128;
                
                
                //0.18 = nice one in the room with PERIOD = 25
                //0.38 = acceptable camera is still working PERIOD = 25
                //0.5 = a bit saturated. Camera is working but wheel is kinda working PERIOD = 25
                //> after 0.001 it is not working properly.
                
                
                // For the testing purpose, I will adjust period in bluetooth module
            }
            */
        }
        
        if(t.read_ms() >= 16) //integrating > INTEGRATIONTIME
        {
            t.reset();
            integrating = 0;
            i = 0;
        }       
    }
}


void servoControl(int midpoint)
{
    // Assume the cneter is 55
    int center = 55;
    float k_p = 0.9f;
    const float UNIT = 0.03f / 100;
    float change;
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


void race(){
    // Minimum intergartion time will be
    // T = (1/maximum clk) * (n - 18) pixels
    // Ex, for 8MHz, T = 0.125us * ( 128 - 18)  
    PERIOD = 1; // us
    //const int SIZE = 128;
    int MAXT = 129;
    int integrating = 0; 
    int i = 0;
    float new_buffer[128];
    float moving_average[128];
    float TOTALVAL = 0.0f;
    Timer t;
    t.start();
    CLK = 0;
    
    t.reset();
    while(1)
    {
        // From checking
        float maxValue = 0, minValue = 128;
        
        int maxIndex = 0, minIndex = 0;
        
        CLK = 0;
        SI = integrating == 0; 
        wait_us(PERIOD);
        
        CLK = 1;
        wait_us(PERIOD);
        SI = 0;
        integrating++;

        // We are integrating
        if(integrating < MAXT)
        {
            linescan_buffer[i] = camera1;
            TOTALVAL += linescan_buffer[i];
            i++;
        }
        else if(integrating == MAXT)
        {
            
            for (int k = 1; k < 125; ++k) {
                moving_average[k] = (1.0f/3.0f) * (linescan_buffer[k-1] + linescan_buffer[k] + linescan_buffer[k+1]);
            }
            for (int j = 1; j < 125; ++j) {
                new_buffer[j] = moving_average[j] - moving_average[j-1];
            }
            
            for (int i = 10; i < 120; ++i) {
                if (new_buffer[i] > maxValue) {
                    maxValue = new_buffer[i];
                    maxIndex = i;
                }
            }
            for (int i = maxIndex; i < 120; ++i) {
                if (new_buffer[i] < minValue){
                    minValue = new_buffer[i];
                    minIndex = i;
                }
            }
            
            int change = midpoint - (maxIndex + minIndex)/2;
            change = change > 0? change : -change;
            const int WIDTH = 20;
            
          if(change < 40 && (minIndex - maxIndex) < WIDTH)
            {
                midpoint = (maxIndex + minIndex) / 2;
            }
            else
            {
                // mid is not found
            }
            
            exposure = TOTALVAL / 128.0f;
            TOTALVAL = 0.0f;            
            
            integrating = 0;
        }
        if(t.read_ms() >= 16)//integrating > INTEGRATIONTIME)
        {   
            servoControl_race(midpoint);
            t.reset();
        }
    }
}
/*
    There are 128 possible mid points (prefer 100)
    Servo PWM varies from LEFT = 0.09f; RIGHT = 0.06f; -> 0.03
    PWM 0.03 / 128 = 0.00234375
*/
void servoControl_race(int midpoint)
{
    
    // Assume the cneter is 55
    int center = 55;
    float k_p = 0.9f;
    const float UNIT = 0.03f / 100;
    float change;
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



void freescale()
{
    // Minimum intergartion time will be
    // T = (1/maximum clk) * (n - 18) pixels
    // Ex, for 8MHz, T = 0.125us * ( 128 - 18)  
    PERIOD = 1; // us
    //const int SIZE = 128;
    int MAXT = 129;
    int integrating = 0; 
    int i = 0;
    Timer t;
    t.start();
    CLK = 0;
    
    t.reset();
    while(1)
    {
        // From checking
        
        int maxIndex = 0, minIndex = 0;
        
        float highestValue = 0.0f;
        int highestValueIndex = 0;
        
        CLK = 0;
        SI = integrating == 0; 
        wait_us(PERIOD);
        
        CLK = 1;
        wait_us(PERIOD);
        SI = 0;
        ++integrating;

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
                // To find the track
                if(linescan_buffer[k] > highestValue)
                {
                    highestValueIndex = k;
                }
                if(k > 0 && k < 127){
                    moving_average[k] = (1.0f/3.0f) * (normalized_buffer[k-1] + normalized_buffer[k] + normalized_buffer[k+1]);
                }
                else if(k == 0){
                    moving_average[k] = (1.0f/2.0f) * (normalized_buffer[k] + normalized_buffer[k+1]);
                }
                else if(k == 127){
                    moving_average[k] = (1.0f/2.0f) * (normalized_buffer[k-1] + normalized_buffer[k]);
                }
            }
            
            /*
            // High Pass Filter
            for (int j = 0; j < 127; ++j) {
                high_pass_buffer[j] = moving_average[j] - moving_average[j+1];
            }
        
            // After high pass, PIXEL 63 is now center.
            */
            
            /*
            for (int n = 0; n < 128; ++n) {
                if(moving_average[n] < TRACK_COLOR_THRESHOLD){
                    track_buffer[n] = 0;
                }
                else{
                    track_buffer[n] = 1;
                }
            }
            */
            
            /*
            // After highpass, we will have one high pass, and one low value
            for (int i = 10; i < 120; ++i) {
                if (new_buffer[i] > maxValue) {
                    maxValue = new_buffer[i];
                    maxIndex = i;
                }
            }   
            for (int i = 10; i < 120; ++i) {
                if (new_buffer[i] < minValue){
                    minValue = new_buffer[i];
                    minIndex = i;
                }
            }
            */
            
            // This constact MUST be changed.
            int change = midpoint - (maxIndex + minIndex)/2;
            change = change > 0? change : -change;
            const int WIDTH_MIN = 20;
            const int WIDTH_MAX = 100;
            if(change < 40 && ((minIndex - maxIndex) > WIDTH_MIN) && (minIndex - maxIndex <WIDTH_MAX)){
                midpoint = (maxIndex + minIndex) / 2;
            }
            else
            {
                // mid is not found
            }
            servoControl_freescale(midpoint);
        }
        // Period is 2 us
        // We are collecting data 128 times which is total integration time to be 256 us.
        // We can either collecting more data but we are not doing ot right now
        // Anyway, we update the servo at every 256 us * 80 = 20.48 ms to be safe 
        // between 60 ~ 70
        if(t.read_ms() >= 16)//integrating > INTEGRATIONTIME)
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
    // Assume the cneter is 55
    int center = 55;
    float k_p = 0.9f;
    const float UNIT = 0.03f / 100;
    float change;
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


/*
void BlueSMiRF(void const *args)
{
    Thread::wait(10000);
    Timer t;
    float PWM = 0.2f;
    bluetooth.printf("CONNECTED\r\n");
    while(1)
    {
        
        if(bluetooth.writeable())
        {
            //bluetooth.printf("%d\r\n", midpoint);
    
            printf("PROCESSED_LINE::::");
            for(int i = 0; i < 128; ++i){
                bluetooth.printf("%d", track_buffer[i]);
            }
            printf("\r\n");
            
            
            bluetooth.printf("%d\r\n", buffer[65]);
        }
        
        Thread::wait(3000);
    }
}
*/

void telemetry_thread(void const *args){
    //telemetry_serial.format();
    telemetry_serial.baud(115200);
    
    telemetry::MbedHal telemetry_hal(telemetry_serial);
    telemetry::Telemetry telemetry_obj(telemetry_hal);
    
    telemetry::Numeric<uint32_t> tele_time_ms(telemetry_obj, "time", "Time", "ms", 0);
    telemetry::NumericArray<uint16_t, 128> tele_linescan(telemetry_obj, "linescan", "Linescan", "ADC", 0);
    //telemetry::NumericArray<uint8_t, 128> tele_linescan_2(telemetry_obj, "linescan_2", "Linescan", "ADC", 0);
    telemetry::NumericArray<uint16_t, 128> tele_normalized(telemetry_obj, "normalized_buffer", "Normalized Linescan 16bit", "ADC", 0);
    //telemetry::NumericArray<uint8_t, 128> tele_normalized_2(telemetry_obj, "normalized_buffer_2", "Normalized Linescan 8bit", "ADC", 0);  

    telemetry_obj.transmit_header();
    
    Timer timer;
    timer.start();
    int start = timer.read_ms();
    
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

        for (int i = 0; i < 128; ++i){
            tele_normalized[i] = normalized_buffer[i];
        }

        /*
        for (int i = 0; i < 128; ++i){
            tele_normalized[i] = normalized_buffer[i];
        }

        for (int i = 0; i < 128; ++i){
            tele_normalized_2[i] = normalized_buffer_2[i];
        }
        */

        telemetry_obj.do_io();
                
        Thread::wait(200);
    }
}

int main() {
    //telemetry_serial.printf("--- Team 7 (Quinn, Byung, Frank), EE 192 ---\r\n");
    //telemetry_serial.printf("Built " __DATE__ " " __TIME__ "\r\n");  
    
    wait(2);
    
    // Initializing
    //telemetry_serial.format();
    //telemetry_serial.baud(115200);
    servo.period(.02f);
    servo.write(0.075f);    
    motor_left.period(.001f);
    brake_left.period(.001f);
    motor_left.write(0.333f); // Assume we are feeding constant velocity
    brake_left.write(0.0f);
    motor_right.period(0.001f);
    brake_right.period(0.001f);
    motor_right.write(0.333f);
    brake_right.write(0.0f);
    //Thread bThread(BlueSMiRF);
    Thread teleThread(telemetry_thread);
    
    //mainControl();
    freescale();
}
