#include "mbed.h"
#include "rtos.h"
#include "MODSERIAL.h"

Serial serial(USBTX, USBRX);

DigitalOut led_green(LED_GREEN);
DigitalOut led_red(LED_RED);
DigitalOut led_blue(LED_BLUE);

PwmOut servo(PTC9);
PwmOut motor(PTA5);

DigitalIn encoder(PTC12);

float target_velocity = 3; // average time per phase we are targetting (in ms)
float motor_pwm = 0.1f; // 0.07
float convert_to_velocity(float period);

void feedback_loop(float current_average){
	float current_velocity = convert_to_velocity(current_average);
	float error = target_velocity - current_velocity;
	
	serial.printf("Current Velocity: %3.4f in m/s \r\n", current_velocity);
	
	error = error / 10 ;
	float k_const = 0.03f;
		
	motor_pwm = k_const * error + motor_pwm;
	
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

void main_control(void const *args){
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

int main() {
  serial.printf("--- Team 7 (Quinn, Byung, Frank), EE 192 ---\r\n");
	serial.printf("Built " __DATE__ " " __TIME__ "\r\n");  
	
	led_green.write(1);
	
	motor.period(.002f);
	servo.period(.02f);
	
	servo.write(0.075f);
	motor.write(0.0f);
	
	//Thread mainThread(main_control);
	Thread encoderThread(speed_control);
	
	// Set a timer to periodically call led_blink_periodic().
  RtosTimer ledBlinkTimer(led_blink_periodic);
  ledBlinkTimer.start(1000);

  // Work is done in the threads, so main() can sleep.
  Thread::wait(osWaitForever);
}
