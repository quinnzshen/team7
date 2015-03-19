#include "mbed.h"
#include "rtos.h"
#include "MODSERIAL.h"
//#include "telemetry/server-cpp/telemetry.h"
//#include "telemetry/server-cpp/telemetry-mbed.h"

Serial serial(USBTX, USBRX);

//MODSERIAL telemetry_serial (PTA2, PTA1) ;
//Serial bluetooth(PTA2, PTA1);

DigitalOut led_green(LED_GREEN);
DigitalOut led_red(LED_RED);
DigitalOut led_blue(LED_BLUE);

PwmOut servo(PTA12);
PwmOut motor(PTD4); // IN_LS1
PwmOut IN_HS1(PTA4);

PwmOut motor2(PTA5); // IN_LS2 
PwmOut IN_HS2(PTC9);

AnalogIn Aout(PTB3);  // First camera - Output from camera
DigitalOut CLK(PTB11);
DigitalOut SI(PTE2);
DigitalIn encoder(PTD5); // Left
//DigitalIn encoder(PTD0); // Right


float convert_to_velocity(float period);
void servoControl(int midpoint);

float target_velocity = 3; // average time per phase we are targetting (in ms)
float motor_pwm = 0.1f; // 0.07
float buffer[128];

const int LEFTMOTOR = 0;
//const int RIGHTMOTOR = 1;
const float CENTER = 0.075f;
const float LEFT = 0.09f;
const float RIGHT = 0.06f;
\
int midpoint = 55;

void motorControl(int LeftOrRight, float speed)
{
	if(speed == 0.0f) // if forwarding
	{
		if(LeftOrRight == LEFTMOTOR) // left
		{
			motor.write(speed);
			IN_HS1.write(0.0f);
		}
		else
		{
			motor2.write(speed);
			IN_HS2.write(0.0f);
		}
	}
	else
	{
		if(LeftOrRight == LEFTMOTOR)
		{
			motor.write(.0f);
			IN_HS1.write(1.0f);
		}
		else
		{
			motor2.write(0.0f);
			IN_HS2.write(1.0f);
		}
	}
}

void feedback_loop(float current_average){
	float current_velocity = convert_to_velocity(current_average);
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
	motor.write(motor_pwm);
	//wait_ms(20);
}

float convert_to_velocity(float period) { // s / rev
	float circ = 0.1602f; // in m / rev
	return circ / period;
}

void speed_control(void const *args){
	Timer t;
	float buffer[18]; // = {0,0,0,0,0,0};
	int buffer_pointer = 0;
	int state = -1;
	motor.write(motor_pwm);
	//wait_ms(20);
	
	while(1) {
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
			t.reset();
		}
		
		if (buffer_pointer >= 11){
			float avg = ((buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5] + buffer[6] + buffer[7] + buffer[8] + buffer[9] + buffer[10] + buffer[11])/2);
			feedback_loop(avg/1000000); //convert us to seconds.
			buffer_pointer = 0;
			t.reset();
		}
	}
}



void mainControl(void const *args){
	
	// Minimum intergartion time will be
	// T = (1/maximum clk) * (n - 18) pixels
	// Ex, for 8MHz, T = 0.125us * ( 128 - 18)  
	const	int PERIOD = 25; // us
	//const int SIZE = 128;
	const int INTEGRATIONTIME = 500;
	int MAXT = 129;
	int integrating = 0; 
	int i = 0;
	
	// From checking
	//float buffer_1[128];
	//float buffer_2[128];
	float new_buffer[128];
	float moving_average[128];
	
	CLK = 0;
	
	while(1)
	{
		//serial.printf("YYYY");
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
			buffer[i] = Aout;
			i++;
		}
		else if(integrating == MAXT)
		{
			
			for (int k = 1; k < 125; ++k) {
				moving_average[k] = (1/3.0) * (buffer[k-1] + buffer[k] + buffer[k+1]);
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
			//motorControl(); Not for this checkpoint
		}
		else if(integrating > INTEGRATIONTIME)
		{
			//serial.printf("MID : %d\r\n", midpoint);
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
void servoControl(int midpoint)
{
	// Assume the cneter is 55
	int center = 55;
	const float UNIT = 0.03f / 100;
	float change;
	if(midpoint < center)
	{
		change = -UNIT * (center - midpoint) * 1.0f;
	}
	else
	{
		change = -UNIT * (center - midpoint) * 1.0f;
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
void telemetry_thread(void const *args)
{
	telemetry::MbedHal telemetry_hal(telemetry_serial);
	telemetry::Telemetry telemetry_obj(telemetry_hal);
	
	// define data
	telemetry::Numeric<uint32_t> tele_time_ms(telemetry_obj,
																						"time", "Time", "ms", 0);
	telemetry::Numeric<float> tele_steer_angle_pct(telemetry_obj,
																						        "steer_angle", "Steer Angle", "Pct.", 0);
	telemetry::Numeric<float> tele_motor_left_duty(telemetry_obj,
																						        "motor_left_duty", "Duty", "%", 0);
	telemetry::Numeric<float> tele_motor_right_duty(telemetry_obj,
																						        "motor_right_duty", "Duty", "%", 0);
	telemetry::NumericArray<uint16_t, 128> tele_linescan(
		telemetry_obj, "linescan", "Linescan", "ADC", 0);
	
	// set limits
	tele_steer_angle_pct.set_limits(-1.1, 1.1);
	tele_motor_left_duty.set_limits(-0.1, 1.1);
	tele_motor_right_duty.set_limits(-0.1, 1.1);
	
	// transmit header once at the beginning
	telemetry_obj.transmit_header();
	
	Timer timer;
	timer.start();
	int start = timer.read_ms();
	
	telemetry_obj.do_io();
	Thread::wait(100);
	
	while (true) {
		tele_time_ms = timer.read_ms() - start;

		for(int i=0; i < 128; ++i) {
			tele_linescan[i] = (int)(65536 * buffer[i]);
		}		
		
		telemetry_obj.do_io();
	}
}
*/
/*

void BlueSMiRF(void const *args)
{
	bluetooth.printf("CONNECTED\r\n");
	int i = 0;
	while(1)
	{
		bluetooth.printf("%d : ", i);
		for(int i = 0; i < 128; i++)
			bluetooth.printf("%f ", buffer[i]);
		bluetooth.printf("%d \r\n", midpoint);
	}
}
*/

int main() {
  serial.printf("--- Team 7 (Quinn, Byung, Frank), EE 192 ---\r\n");
	serial.printf("Built " __DATE__ " " __TIME__ "\r\n");  
	
	wait(3);
	
	//bluetooth.format();
	//bluetooth.baud(115200);
  servo.period(.02f);
	servo.write(0.075f);	
	
	motor.period(.001f);
	IN_HS1.period(.001f);
	motor.write(0.15f); // Assume we are fee8ding constant velocity
	IN_HS1.write(0.0f);
	
	motor2.period(0.001f);
	IN_HS2.period(0.001f);
	motor2.write(0.15f);
	IN_HS2.write(0.0f);
	
	Thread mainThread(mainControl);
	//Thread encoderThread(speed_control);
	
	//Thread bThread(BlueSMiRF);
	Thread::wait(2);
	Thread::wait(osWaitForever);
	
	
}
