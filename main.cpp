#include "mbed.h"
#include "rtos.h"
#include "MODSERIAL.h"

Serial serial(USBTX, USBRX);

DigitalOut led_green(LED_GREEN);
DigitalOut led_red(LED_RED);
DigitalOut led_blue(LED_BLUE);

PwmOut servo(PTC9);
PwmOut motor(PTA5);
	
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
	
	Thread mainThread(main_control);
	
	// Set a timer to periodically call led_blink_periodic().
  RtosTimer ledBlinkTimer(led_blink_periodic);
  ledBlinkTimer.start(1000);

  // Work is done in the threads, so main() can sleep.
  Thread::wait(osWaitForever);
}
