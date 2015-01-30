#include "mbed.h"
#include "rtos.h"
#include "MODSERIAL.h"

MODSERIAL serial(USBTX, USBRX);

DigitalOut led_green(LED_GREEN);
DigitalOut led_red(LED_RED);
PwmOut led_blue(LED_BLUE);

PwmOut my_led(PTA13);
PwmOut my_led2(PTD5);

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
  // Start a thread running led_fade_thread().
  // Thread ledFadeThread(led_fade_thread);
	Thread blinkThread(blink_my_led); 
	Thread fadeThread(fade_my_led);
  // Set a timer to periodically call led_blink_periodic().
  RtosTimer ledBlinkTimer(led_blink_periodic);
  ledBlinkTimer.start(1000);

  // Work is done in the threads, so main() can sleep.
  Thread::wait(osWaitForever);
}
