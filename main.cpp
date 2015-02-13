#include "mbed.h"
#include "rtos.h"
#include "MODSERIAL.h"

Serial serial(USBTX, USBRX);

DigitalOut led_green(LED_GREEN);
DigitalOut led_red(LED_RED);
PwmOut led_blue(LED_BLUE);

PwmOut my_led(PTA13);
PwmOut my_led2(PTD5);
PwmOut servo(PTC9);
PwmOut motor(PTA5);

void led_fade_thread(void const *args) {
  // Note that this function doesn't terminate, which is fine since it runs in
  // a thread.
  while (1) {
    // Since the internal LED is active low, inert the duty cycle.
    led_blue.write(1 - 0);
    Thread::wait(250);
    led_blue.write(1 - 0.25);
    Thread::wait(250);
    led_blue.write(1 - 0.5);
    Thread::wait(250);
    led_blue.write(1 - 0.75);
    Thread::wait(250);
  }
}

void blink_my_led(void const *args) {
  while (1) {
    // Since the internal LED is active low, inert the duty cycle.
    my_led2.write(1);
    Thread::wait(750);
    my_led2.write(0);
		Thread::wait(250);
  }
}

void fade_my_led(void const *args) {
	float brightness = 0;
	while (1) {
		for (float i = -1.0f; i < 1.0f; i += .01f)
		{
			if(i <= 0){
				brightness = 1.0f + i;
			}
			else{
				brightness = 1.0f - i;
			}
			my_led.write(brightness);
			Thread::wait(10);
		}
	}
}

/*
void servo_thread(void const *args) {
	while (1) {		
		for (float i = .06f; i <= .09f; i += .002f) {
			servo = i;
			Thread::wait(20);
		}
		for (float i = .09f; i >= .06f; i -= .002f) {
			servo = i;
			Thread::wait(20);
		}
  }
}

void servo_centered(void const *args) {
	while (1) {
		servo2 = .075f;
		Thread::wait(20);
	}
}
*/

void servo_control(void const *args) {
	char keyboard;
	while(1) {
		keyboard = serial.getc();
		if(keyboard == 'w') // center
		{
			servo = 0.075f;
		}
		else if(keyboard == 'a') // left
		{
			servo = .09f;
		}
		else if (keyboard == 'd' ) // Right
		{
			servo = .06f;
		}
		Thread::wait(10);
	}
}
void motor_control(void const *args){
	char keyboard;
	while(1) {
		keyboard = serial.getc();
		if(keyboard == 'h') // 100
		{
			motor = 1.0f;
		}
		else if(keyboard == 'j') // 50
		{
			motor = .5f;
		}
		else if (keyboard == 'k' ) // 30
		{
			motor = .3f;
		}
		else if (keyboard == 'l') // stop
		{
			motor = 0.0f;
		}
		Thread::wait(2);
	}
}
	

void led_blink_periodic(void const *args) {
  // Toggle the red LED when this function is called.
  led_red = !led_red;
}

int main() {
  // It's always nice to know what version is deployed.
  serial.printf("Built " __DATE__ " " __TIME__ "\r\n");
  
  // Quick blink on startup.
  led_green = 0;  // Note that the internal LED is active low.
  wait(0.25);
  led_green = 1;
  wait(0.25);
	
  // Mandatory "Hello, world!".
  serial.printf("Hello, world!\r\n");
	
	/*
	servo = 0.075f; // Center
	servo.period(0.020f); // Servo PWM period
  */
	servo = 0.075f;
	
	//motor.period(0.005f);
	//motor.pulsewidth(); 
	
	// Start a thread running led_fade_thread().
  // Thread ledFadeThread(led_fade_thread);
	// Thread blinkThread(blink_my_led); 
	// Thread fadeThread(fade_my_led);
	Thread servoThread(servo_control);
	Thread motorThread(motor_control);
  // Set a timer to periodically call led_blink_periodic().
  RtosTimer ledBlinkTimer(led_blink_periodic);
  ledBlinkTimer.start(1000);

  // Work is done in the threads, so main() can sleep.
  Thread::wait(osWaitForever);
}
