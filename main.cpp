#include "mbed.h"
#include "rtos.h"
#include "MODSERIAL.h"
#include "telemetry/server-cpp/telemetry.h"
#include "telemetry/server-cpp/telemetry-mbed.h"

Serial serial(USBTX, USBRX);

MODSERIAL telemetry_serial ( PTA2 , PTA1 ) ;

DigitalOut led_green(LED_GREEN);
DigitalOut led_red(LED_RED);
DigitalOut led_blue(LED_BLUE);

PwmOut servo(PTC9);
PwmOut motor(PTA5);

DigitalOut CLK(PTD4);
DigitalOut SI(PTD0);
AnalogIn Aout(PTB0);

DigitalIn encoder(PTC12);

float target_velocity = 3; // average time per phase we are targetting (in ms)
float motor_pwm = 0.1f; // 0.07
float convert_to_velocity(float period);
float buffer[128];

int ready =0;


void feedback_loop(float current_average){
	float current_velocity = convert_to_velocity(current_average);
	float error = target_velocity - current_velocity;
	
	serial.printf("Current Velocity: %3.4f in m/s \r\n", current_velocity);
	error = error / 10 ;
	float k_const = 0.03f;
		
	motor_pwm = k_const * error + motor_pwm;
	
	serial.printf("PWM %3.4f in pwm \r\n", motor_pwm);
	if(motor_pwm >= .3f){
		serial.printf("TOO BIG.\r\n");
		motor_pwm = .3f;
	}
	else if(motor_pwm < 0.0f){
		serial.printf("NEGATIVE.\r\n");
		motor_pwm = .01f;
	}
	
	motor.write(motor_pwm);
	wait_ms(20);
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
	
	serial.printf("Motor Started\r\n");
	motor.write(motor_pwm);
	wait_ms(20);
	
	while(1) {
		// Initialize state & start timer
		if (state == -1){
			state = encoder;
			t.start();
		}
		else if(t.read_ms() > 1602){
			serial.printf("Current Velocity: %3.4f in m/s \r\n", convert_to_velocity(1602.0f/1000));
			// serial.printf("MOTOR BARELY MOVING. (<.1 m/s)\r\n");
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


void main_control2(void const *args){
	while(1) {
		switch (serial.getc()){
			// Motor Control
			case 'h':
				serial.printf("100%% throttle.\r\n");
				motor.write(1.0f);
				break;
			case 'j':
				serial.printf("50%% throttle.\r\n");
				motor.write(.5f);
				break;
			case 'k':
				serial.printf("30%% throttle.\r\n");
				motor.write(.3f);
				break;
			case 'm':
				serial.printf("5%% throttle.\r\n");
				motor.write(0.05f);
				break;
			case 'l':
				serial.printf("Stopped throttle.\r\n");
				motor.write(0.0f);
				break;
			// Servo Control
			case 'w':
				serial.printf("Centered servo.\r\n");
				servo.write(0.075f);
				break;
			case 'a':
				serial.printf("Full-left servo.\r\n");
				servo.write(0.09f);
				break;
			case 'd':
				serial.printf("Full-right servo.\r\n");
				servo.write(0.06f);
				break;
			default:
				wait_ms(20);
				break;
		}
	}
}

void led_blink_periodic(void const *args) {
  // Toggle the red LED when this function is called.
  led_red = !led_red;
}


void main_control(void const *args){//this is a pointer to a single character in memory  
  // Assume velocity is controlled by other thread and waiting for signal()
	int period = 100;
	
	// Minimum intergartion time will be
	// T = (1/maximum clk) * (n - 18) pixels + 20us
	// Ex, for 8MHz, T = 0.125us * ( 128 - 18) + 20us 
	
	int MAXT = 129;//100*(128-18) + 20;
	int integrating = MAXT; 
	int i = 0;
	
	// From checking
	float new_buffer[128];
	float moving_average[128];
	
	while(1)
	{
		// From checking
		float maxValue = 0;
		float minValue = 128;
		int maxIndex, minIndex;
		maxIndex = 0;
		minIndex = 0;
		int midpoint = 0;
		
		CLK = 0;
		SI = integrating == MAXT ? 1 : 0; // If it is integration time, don't set to 1
		wait_us(period);		
		
		CLK = 1;
		wait_us(period);
		SI = 0;
		integrating = integrating == 0 ? MAXT : integrating-1;
		
		// We are integrating
		if(integrating != MAXT)
		{
			buffer[i] = Aout;
			i++;
			ready++;
		}
		else
		{
			i = 0;
			ready = 0;
			for (int k = 0; k < 125; ++k) {
				moving_average[k] = (1/3.0) * (buffer[k] + buffer[k+1] + buffer[k+2]);
			}
			for (int j = 1; j < 125; ++j) {
				new_buffer[j] = moving_average[j] - moving_average[j-1];
			}
			for (int i = 10; i < 120; ++i) {
				//serial.printf("frame: %d value: %f \r\n", i , new_buffer[i]);
				if (new_buffer[i] < minValue){
					minValue = new_buffer[i];
					minIndex = i;
				}
			}
			for (int i = minIndex; i < 120; ++i) {
				if (new_buffer[i] > maxValue) {
					maxValue = new_buffer[i];
					maxIndex = i;
				}
			}
			midpoint = (maxIndex + minIndex) / 2;
			//serial.printf("midpoint %d \r\n", midpoint);
			//wait(1);
			
			//serial.printf("midpoint %d \r\n", midpoint);
			//serial.printf("Level %d \r\n", (int)((midpoint - 65)/15));
			//servo.write((int)((midpoint - 65)/15) * .006 + .075f);
			
			if(midpoint >= 55 && midpoint < 83)
			{
				//centered;
				serial.printf("center %d \r\n", midpoint);
				servo.write(0.075f);
			}
			else if(midpoint >= 83 && midpoint < 125)
			{
				//right
				serial.printf("right %d \r\n", midpoint);
				servo.write(0.09f);
			}
			else if(midpoint < 55 && midpoint >= 0)
			{
				//left
				serial.printf("left %d \r\n", midpoint);
				servo.write(0.06f);
			}
			
		}
	}
}



void figure8(void const *args)
{
	// wait for 5 sec
	wait(5);
	
	// Top circle
	while(1)
	{
		// Top circle
		wait(6.9);
		motor.write(0.1f); // constant speed 3m/s
		servo.write(0.09f);
		
		wait(6.9);
		// Bottom Circle -- just reverse steering of the top one
		servo.write(0.06f);
		
	}
}

/*
void checking(void const *args)
{
	
	while(1)
	{

		if (ready == 127) {
			
		}
	}
}*/

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
}*/

int main() {
  serial.printf("--- Team 7 (Quinn, Byung, Frank), EE 192 ---\r\n");
	serial.printf("Built " __DATE__ " " __TIME__ "\r\n");  
	
	led_green.write(1);
	
	//motor.period(.002f);
  servo.period(.02f);
	
	servo.write(0.075f);
	//motor.write(0.0f); // Assume we are feeding constant velocity
	
	//CLK.period(0.0001f);
	//CLK.write(0.0f);
	
	Thread mainThread(main_control);
	//Thread checkingThread(checking);
	
	//Thread encoderThread(speed_control);
	//Thread openloopThread(figure8);
	
	
	
	// Set a timer to periodically call led_blink_periodic().
  //RtosTimer ledBlinkTimer(led_blink_periodic);
  //ledBlinkTimer.start(1000);

  // Work is done in the threads, so main() can sleep.
  Thread::wait(osWaitForever);
}
