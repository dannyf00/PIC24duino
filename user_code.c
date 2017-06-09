#include "picduino.h"						//we use pic-port of the arduino libraries

//global defines
#define LED1					PB12		//LED1 tied to PB11
#define LED2					PB15		//LED2 tied to PB15

//user setup
void setup(void) {
	//user setup code
	pinMode(LED1, OUTPUT);					//LED as output
	
}

void loop(void) {

	digitalWrite(LED1, !digitalRead(LED1));	//flip led1 - for debugging
	delay(100);
	
}
