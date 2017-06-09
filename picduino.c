//PICduino16 code base

#include "picduino_config.h"				//configuration words - for XC16
#include "picduino.h"						//PICduino-related definitions / prototypes

//empty handler
static void empty_handler(void) {
	//default tmr handler - do nothing here
}

//global variables


#if defined(INT02RP)
static void (* _isrptr_INT0)(void)=empty_handler;				//isrptr for INT0, pointing to empty_handler by default
//INT0 isr
void _ISR _INT0Interrupt(void) {
	_INT0IF = 0;						//clear the flag
	_isrptr_INT0();						//run the isr
}
#endif

#if defined(INT12RP)
static void (* _isrptr_INT1)(void)=empty_handler;				//isrptr for INT0, pointing to empty_handler by default
//INT1 isr
void _ISR _INT1Interrupt(void) {
	_INT1IF = 0;						//clear the flag
	_isrptr_INT1();						//run the isr
}
#endif

#if defined(INT22RP)
static void (* _isrptr_INT2)(void)=empty_handler;				//isrptr for INT0, pointing to empty_handler by default
//INT2 isr
void _ISR _INT2Interrupt(void) {
	_INT2IF = 0;						//clear the flag
	_isrptr_INT2();						//run the isr
}
#endif

#if defined(_INT3Interrupt) && defined(INT32RP)
static void (* _isrptr_INT3)(void)=empty_handler;				//isrptr for INT0, pointing to empty_handler by default
//INT3 isr - not always there
void _ISR _INT3Interrupt(void) {
	_INT3IF = 0;						//clear the flag
	_isrptr_INT3();						//run the isr
}
#endif									//use_INT3

#if defined(USE_CNINT)
static void (* _isrptr_CN)(void) = empty_handler;							//isrptr for CN0..31, pointing to empty_handler by default
//cn isr
void _ISR _CNInterrupt(void) {
	_CNIF = 0;							//clear the flag
	_isrptr_CN();						//execute the handler
}
#endif									//pcint

//global definitions
//global variables
//for time base off TIMER1 @ 1:1 prescaler
//volatile uint32_t timer1_millis = 0;
volatile uint32_t timer_ticks = 0;
//static uint16_t timer1_fract = 0;

//timer1 overflow isr
void _ISR _T1Interrupt(void) {
	_T1IF=0;							//clear tmr1 interrupt flag
	timer_ticks+=0x10000ul;				//increment overflow count: 16-bit timer
}

//declare pins
//ALL PINS ARE MAPPED, WHETHER THEY EXIST OR NOT
//SO MAKE SURE THAT THE PINS YOU PICKED ACTUALLY EXIST FOR YOUR PACKAGE
//Pin  0.. 7 -> GPIOA
//Pin  8..15 -> GPIOB
//Pin 16..23 -> GPIOC
//Pin 24..31 -> GPIOD
//Pin 32..39 -> GPIOE
//Pin 40..47 -> GPIOF
//Pin 48..55 -> GPIOG
//Pin 56..63 -> GPIOH
//Pin 64..71 -> GPIOI
const PIN2GPIO GPIO_PinDef[]={
	{GPIOA, 1<<0},						//PICduino Pin  0 = RP0/PB0/CHIP PIN4
	{GPIOA, 1<<1},						//PICduino Pin  1 = RP1/PB1/CHIP PIN5
	{GPIOA, 1<<2},						//PICduino Pin  2 = RP2/PB2/CHIP PIN6
	{GPIOA, 1<<3},						//PICduino Pin  3 = RP3/PB3/CHIP PIN7
	{GPIOA, 1<<4},						//PICduino Pin  4 = RP4/PB4/CHIP PIN11
	{GPIOA, 1<<5},						//PICduino Pin  5 = RP5/PB5/CHIP PIN14
	{GPIOA, 1<<6},						//PICduino Pin  6 = RP6/PB6/CHIP PIN15
	{GPIOA, 1<<7},						//PICduino Pin  7 = RP7/PB7/CHIP PIN16
	{GPIOA, 1<<8},						//PICduino Pin  8 = RP8/PB8/CHIP PIN17
	{GPIOA, 1<<9},						//PICduino Pin  9 = RP9/PB9/CHIP PIN18
	{GPIOA, 1<<10},						//PICduino Pin 10 = RP10/PB10/CHIP PIN21
	{GPIOA, 1<<11},						//PICduino Pin 11 = RP11/PB11/CHIP PIN22
	{GPIOA, 1<<12},						//PICduino Pin 12 = RP12/PB12/CHIP PIN23
	{GPIOA, 1<<13},						//PICduino Pin 13 = RP13/PB13/CHIP PIN24
	{GPIOA, 1<<14},						//PICduino Pin 14 = RP14/PB14/CHIP PIN25
	{GPIOA, 1<<15},						//PICduino Pin 15 = RP15/PB15/CHIP PIN26

	{GPIOB, 1<<0},						//PICduino Pin 16 = RP0/PB0/CHIP PIN4
	{GPIOB, 1<<1},						//PICduino Pin 17 = RP1/PB1/CHIP PIN5
	{GPIOB, 1<<2},						//PICduino Pin 18 = RP2/PB2/CHIP PIN6
	{GPIOB, 1<<3},						//PICduino Pin 19 = RP3/PB3/CHIP PIN7
	{GPIOB, 1<<4},						//PICduino Pin 20 = RP4/PB4/CHIP PIN11
	{GPIOB, 1<<5},						//PICduino Pin 21 = RP5/PB5/CHIP PIN14
	{GPIOB, 1<<6},						//PICduino Pin 22 = RP6/PB6/CHIP PIN15
	{GPIOB, 1<<7},						//PICduino Pin 23 = RP7/PB7/CHIP PIN16
	{GPIOB, 1<<8},						//PICduino Pin 24 = RP8/PB8/CHIP PIN17
	{GPIOB, 1<<9},						//PICduino Pin 25 = RP9/PB9/CHIP PIN18
	{GPIOB, 1<<10},						//PICduino Pin 26 = RP10/PB10/CHIP PIN21
	{GPIOB, 1<<11},						//PICduino Pin 27 = RP11/PB11/CHIP PIN22
	{GPIOB, 1<<12},						//PICduino Pin 28 = RP12/PB12/CHIP PIN23
	{GPIOB, 1<<13},						//PICduino Pin 29 = RP13/PB13/CHIP PIN24
	{GPIOB, 1<<14},						//PICduino Pin 30 = RP14/PB14/CHIP PIN25
	{GPIOB, 1<<15},						//PICduino Pin 31 = RP15/PB15/CHIP PIN26

#if defined(GPIOC)
	{GPIOC, 1<<0},						//PICduino Pin 32 = RP0/PB0/CHIP PIN4
	{GPIOC, 1<<1},						//PICduino Pin 33 = RP1/PB1/CHIP PIN5
	{GPIOC, 1<<2},						//PICduino Pin 34 = RP2/PB2/CHIP PIN6
	{GPIOC, 1<<3},						//PICduino Pin 35 = RP3/PB3/CHIP PIN7
	{GPIOC, 1<<4},						//PICduino Pin 36 = RP4/PB4/CHIP PIN11
	{GPIOC, 1<<5},						//PICduino Pin 37 = RP5/PB5/CHIP PIN14
	{GPIOC, 1<<6},						//PICduino Pin 38 = RP6/PB6/CHIP PIN15
	{GPIOC, 1<<7},						//PICduino Pin 39 = RP7/PB7/CHIP PIN16
	{GPIOC, 1<<8},						//PICduino Pin 40 = RP8/PB8/CHIP PIN17
	{GPIOC, 1<<9},						//PICduino Pin 41 = RP9/PB9/CHIP PIN18
	{GPIOC, 1<<10},						//PICduino Pin 42 = RP10/PB10/CHIP PIN21
	{GPIOC, 1<<11},						//PICduino Pin 43 = RP11/PB11/CHIP PIN22
	{GPIOC, 1<<12},						//PICduino Pin 44 = RP12/PB12/CHIP PIN23
	{GPIOC, 1<<13},						//PICduino Pin 45 = RP13/PB13/CHIP PIN24
	{GPIOC, 1<<14},						//PICduino Pin 46 = RP14/PB14/CHIP PIN25
	{GPIOC, 1<<15},						//PICduino Pin 47 = RP15/PB15/CHIP PIN26
#endif

#if defined(GPIOD)
	{GPIOD, 1<<0},						//PICduino Pin 48 = RP0/PB0/CHIP PIN4
	{GPIOD, 1<<1},						//PICduino Pin 49 = RP1/PB1/CHIP PIN5
	{GPIOD, 1<<2},						//PICduino Pin 50 = RP2/PB2/CHIP PIN6
	{GPIOD, 1<<3},						//PICduino Pin 51 = RP3/PB3/CHIP PIN7
	{GPIOD, 1<<4},						//PICduino Pin 52 = RP4/PB4/CHIP PIN11
	{GPIOD, 1<<5},						//PICduino Pin 53 = RP5/PB5/CHIP PIN14
	{GPIOD, 1<<6},						//PICduino Pin 54 = RP6/PB6/CHIP PIN15
	{GPIOD, 1<<7},						//PICduino Pin 55 = RP7/PB7/CHIP PIN16
	{GPIOD, 1<<8},						//PICduino Pin 56 = RP8/PB8/CHIP PIN17
	{GPIOD, 1<<9},						//PICduino Pin 57 = RP9/PB9/CHIP PIN18
	{GPIOD, 1<<10},						//PICduino Pin 58 = RP10/PB10/CHIP PIN21
	{GPIOD, 1<<11},						//PICduino Pin 59 = RP11/PB11/CHIP PIN22
	{GPIOD, 1<<12},						//PICduino Pin 60 = RP12/PB12/CHIP PIN23
	{GPIOD, 1<<13},						//PICduino Pin 61 = RP13/PB13/CHIP PIN24
	{GPIOD, 1<<14},						//PICduino Pin 62 = RP14/PB14/CHIP PIN25
	{GPIOD, 1<<15},						//PICduino Pin 63 = RP15/PB15/CHIP PIN26
#endif

#if defined(GPIOE)
	{GPIOE, 1<<0},						//PICduino Pin 64 = RP0/PB0/CHIP PIN4
	{GPIOE, 1<<1},						//PICduino Pin 65 = RP1/PB1/CHIP PIN5
	{GPIOE, 1<<2},						//PICduino Pin 66 = RP2/PB2/CHIP PIN6
	{GPIOE, 1<<3},						//PICduino Pin 67 = RP3/PB3/CHIP PIN7
	{GPIOE, 1<<4},						//PICduino Pin 68 = RP4/PB4/CHIP PIN11
	{GPIOE, 1<<5},						//PICduino Pin 69 = RP5/PB5/CHIP PIN14
	{GPIOE, 1<<6},						//PICduino Pin 70 = RP6/PB6/CHIP PIN15
	{GPIOE, 1<<7},						//PICduino Pin 71 = RP7/PB7/CHIP PIN16
	{GPIOE, 1<<8},						//PICduino Pin 72 = RP8/PB8/CHIP PIN17
	{GPIOE, 1<<9},						//PICduino Pin 73 = RP9/PB9/CHIP PIN18
	{GPIOE, 1<<10},						//PICduino Pin 74 = RP10/PB10/CHIP PIN21
	{GPIOE, 1<<11},						//PICduino Pin 75 = RP11/PB11/CHIP PIN22
	{GPIOE, 1<<12},						//PICduino Pin 76 = RP12/PB12/CHIP PIN23
	{GPIOE, 1<<13},						//PICduino Pin 77 = RP13/PB13/CHIP PIN24
	{GPIOE, 1<<14},						//PICduino Pin 78 = RP14/PB14/CHIP PIN25
	{GPIOE, 1<<15},						//PICduino Pin 79 = RP15/PB15/CHIP PIN26
#endif

#if defined(GPIOF)
	{GPIOF, 1<<0},						//PICduino Pin 80 = RP0/PB0/CHIP PIN4
	{GPIOF, 1<<1},						//PICduino Pin 81 = RP1/PB1/CHIP PIN5
	{GPIOF, 1<<2},						//PICduino Pin 82 = RP2/PB2/CHIP PIN6
	{GPIOF, 1<<3},						//PICduino Pin 83 = RP3/PB3/CHIP PIN7
	{GPIOF, 1<<4},						//PICduino Pin 84 = RP4/PB4/CHIP PIN11
	{GPIOF, 1<<5},						//PICduino Pin 85 = RP5/PB5/CHIP PIN14
	{GPIOF, 1<<6},						//PICduino Pin 86 = RP6/PB6/CHIP PIN15
	{GPIOF, 1<<7},						//PICduino Pin 87 = RP7/PB7/CHIP PIN16
	{GPIOF, 1<<8},						//PICduino Pin 88 = RP8/PB8/CHIP PIN17
	{GPIOF, 1<<9},						//PICduino Pin 89 = RP9/PB9/CHIP PIN18
	{GPIOF, 1<<10},						//PICduino Pin 90 = RP10/PB10/CHIP PIN21
	{GPIOF, 1<<11},						//PICduino Pin 91 = RP11/PB11/CHIP PIN22
	{GPIOF, 1<<12},						//PICduino Pin 92 = RP12/PB12/CHIP PIN23
	{GPIOF, 1<<13},						//PICduino Pin 93 = RP13/PB13/CHIP PIN24
	{GPIOF, 1<<14},						//PICduino Pin 94 = RP14/PB14/CHIP PIN25
	{GPIOF, 1<<15},						//PICduino Pin 95 = RP15/PB15/CHIP PIN26
#endif

#if defined(GPIOG)
	{GPIOG, 1<<0},						//PICduino Pin 96 = RP0/PB0/CHIP PIN4
	{GPIOG, 1<<1},						//PICduino Pin 97 = RP1/PB1/CHIP PIN5
	{GPIOG, 1<<2},						//PICduino Pin 98 = RP2/PB2/CHIP PIN6
	{GPIOG, 1<<3},						//PICduino Pin 99 = RP3/PB3/CHIP PIN7
	{GPIOG, 1<<4},						//PICduino Pin 100= RP4/PB4/CHIP PIN11
	{GPIOG, 1<<5},						//PICduino Pin 101= RP5/PB5/CHIP PIN14
	{GPIOG, 1<<6},						//PICduino Pin 102= RP6/PB6/CHIP PIN15
	{GPIOG, 1<<7},						//PICduino Pin 103= RP7/PB7/CHIP PIN16
	{GPIOG, 1<<8},						//PICduino Pin 104= RP8/PB8/CHIP PIN17
	{GPIOG, 1<<9},						//PICduino Pin 105= RP9/PB9/CHIP PIN18
	{GPIOG, 1<<10},						//PICduino Pin 106= RP10/PB10/CHIP PIN21
	{GPIOG, 1<<11},						//PICduino Pin 107= RP11/PB11/CHIP PIN22
	{GPIOG, 1<<12},						//PICduino Pin 108= RP12/PB12/CHIP PIN23
	{GPIOG, 1<<13},						//PICduino Pin 109= RP13/PB13/CHIP PIN24
	{GPIOG, 1<<14},						//PICduino Pin 110= RP14/PB14/CHIP PIN25
	{GPIOG, 1<<15},						//PICduino Pin 111= RP15/PB15/CHIP PIN26
#endif
};


//Arduino Functions: GPIO
//set a pin mode to INPUT or OUTPUT
//no error checking on PIN
inline void pinMode(PIN_TypeDef pin, uint8_t mode) {
	if (mode==INPUT) GIO_IN(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
	else GIO_OUT(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
}

//set / clear a pin
inline void digitalWrite(PIN_TypeDef pin, uint8_t val) {
	if (val==LOW) GIO_CLR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
	else GIO_SET(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
}

//read a pin
inline int digitalRead(PIN_TypeDef pin) {
	return (GIO_GET(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask))?HIGH:LOW;
}
//end GPIO

//Arduino Functions: Time
//return microseconds
uint32_t micros(void) {
	uint32_t m;					//stores overflow count
	uint16_t f;					//return the fractions / TMR1 value
	
	//use double reads
	do {
		m = timer_ticks;
		f = TMR1;
	} while (m != timer_ticks);
	//now m and f are consistent
	return (m | f) / clockCyclesPerMicrosecond() / 1;
}
	
//return milliseconds
//alternatively, = micros()/1000
uint32_t millis(void) {
	uint32_t m;
	uint16_t f;
	
	//use double reads
	do {
		m = timer_ticks;
		f = TMR1;
	} while (m != timer_ticks);
		
	return (m | f) / clockCyclesPerMicrosecond() / 1000;	//or shift 10 positions
}

//return timer ticks
uint32_t ticks(void) {
	uint32_t m;
	uint16_t f;
	
	//use double reads
	do {
		m = timer_ticks;
		f = TMR1;
	} while (m != timer_ticks);
		
	return (m | f); 							//return ticks
}

//delay millisseconds
void delay(uint32_t ms) {
	uint32_t start_time = millis();

	while (millis() - start_time < ms) continue;
}

//delay micros seconds
void delayMicroseconds(uint32_t us) {
	uint32_t start_time = micros();
	
	while (micros() - start_time < us) continue;
}
//end Time


//Arduino Functions: Advanced IO
//shift in - from arduino code base / not optimized
uint8_t shiftIn(PIN_TypeDef dataPin, PIN_TypeDef clockPin, uint8_t bitOrder) {
	uint8_t value = 0;
	uint8_t i;

	for (i = 0; i < 8; ++i) {
		digitalWrite(clockPin, HIGH);
		if (bitOrder == LSBFIRST)
			value |= digitalRead(dataPin) << i;
		else
			value |= digitalRead(dataPin) << (7 - i);
		digitalWrite(clockPin, LOW);
	}
	return value;
}

//shift out - from arduino code base / not optimized
void shiftOut(PIN_TypeDef dataPin, PIN_TypeDef clockPin, uint8_t bitOrder, uint8_t val) {
	uint8_t i;

	for (i = 0; i < 8; i++)  {
		if (bitOrder == LSBFIRST)
			digitalWrite(dataPin, !!(val & (1 << i)));
		else	
			digitalWrite(dataPin, !!(val & (1 << (7 - i))));
			
		digitalWrite(clockPin, HIGH);
		digitalWrite(clockPin, LOW);		
	}
}

//wait for a pulse and return timing
uint32_t pulseIn(PIN_TypeDef pin, uint8_t state) {
	uint32_t tmp;
	state = (state == LOW)?LOW:HIGH;				
	while (digitalRead(pin) == state) continue;		//wait for the pin to opposite
	//now pin is _state
	tmp = micros();
	state = (state == LOW)?HIGH:LOW;				//calculate the state to end the wait
	while (digitalRead(pin) == state) continue;		//wait for the pin to go back to its original state
	tmp = micros() - tmp;							//calculate the pulse width
	return tmp;
}
//end Advanced IO

//install external interrupt handler
//mode 0: falling edge, 1: rising edge
void attachInterrupt(uint8_t intx, void (*isrptr) (void), uint8_t mode) {
	switch (intx) {
#if defined(INT02RP)
		case 0:									//for int0
			_isrptr_INT0 = isrptr;				//install user handler
			_INT0IF = 0;						//clear int0 flag
			_INT0EP = (mode==RISING)?0:1;		//0=triggered on the rising edge. 1 = falling edge
			_INT0IE = 1;						//enable int0 interrupt
			break;
#endif											//int0
#if defined(INT12RP)
		case 1:									//for int1
			_isrptr_INT1 = isrptr;				//install user handler
			_INT1IF = 0;						//clear int1 flag
			_INT1EP = (mode==RISING)?0:1;		//0=triggered on the rising edge. 1 = falling edge
			_INT1IE = 1;						//enable int1 interrupt
			break;
#endif											//int1
#if defined(INT22RP)
		case 2:									//for int2
			_isrptr_INT2 = isrptr;				//install user handler
			_INT2IF = 0;						//clear int2 flag
			_INT2EP = (mode==RISING)?0:1;		//0=triggered on the rising edge. 1 = falling edge
			_INT2IE = 1;						//enable int2 interrupt
			break;
#endif											//int2
#if defined(_INT3Interrupt) &&  defined(INT32RP)	//int3 not present on all chips
		case 3:									//for int3
			_isrptr_INT3 = isrptr;				//install user handler
			_INT3IF = 0;						//clear int3 flag
			_INT3EP = (mode==RISING)?0:1;		//0=triggered on the rising edge. 1 = falling edge
			_INT3IE = 1;						//enable int3 interrupt
			break;
#endif											//int3
		default: break;							//do nothing
	}		
}
	
//disable external interrupt
void detachInterrupt(uint8_t intx) {
	uint8_t mode = FALLING;						//default mode -> falling edge
	switch (intx) {
#if defined(INT02RP)
		case 0:									//for int0
			_isrptr_INT0 = empty_handler;		//install user handler
			_INT0IF = 0;						//clear int0 flag
			_INT0EP = (mode==RISING)?0:1;		//0=triggered on the rising edge. 1 = falling edge
			_INT0IE = 0;						//disable int0 interrupt
			break;
#endif
#if defined(INT12RP)
		case 1:									//for int1
			_isrptr_INT1 = empty_handler;		//install user handler
			_INT1IF = 0;						//clear int1 flag
			_INT1EP = (mode==RISING)?0:1;		//0=triggered on the rising edge. 1 = falling edge
			_INT1IE = 0;						//disable int1 interrupt
			break;
#endif
#if defined(INT22RP)
		case 2:									//for int2
			_isrptr_INT2 = empty_handler;		//install user handler
			_INT2IF = 0;						//clear int2 flag
			_INT2EP = (mode==RISING)?0:1;		//0=triggered on the rising edge. 1 = falling edge
			_INT2IE = 0;						//disable int2 interrupt
			break;
#endif
#if defined(_INT3Interrupt) && defined(INT32RP)
		case 3:									//for int3
			_isrptr_INT3 = empty_handler;		//install user handler
			_INT3IF = 0;						//clear int3 flag
			_INT3EP = (mode==RISING)?0:1;		//0=triggered on the rising edge. 1 = falling edge
			_INT3IE = 0;						//disable int3 interrupt
			break;
#endif
		default: break;							//do nothing
	}		
}

#if defined(USE_CNINT)
//install user CN interrupt handler
//cnx = 0..31
void attachCNInterrupt(void (*isrptr) (void)) {
	//uint32_t tmp = 1ul << cnx;
	_CNIF = 0;									//clear the flag
	_isrptr_CN = isrptr;						//install user handler
	_CNIE = 1;									//enable interrupt
	//CNEN1 |= tmp; CNPU1 |= (pue==0)?0:tmp;					//enable CN interrupts
	//CNEN2 |= (tmp >> 16); CNPU2 |= (pue==0)?0:(tmp >> 16);	//enable or disable PUE
}

//cnx = 0..31
void detachCNInterrupt(void) {
	//uint32_t tmp = 1ul << cnx;
	_CNIF = 0;									//clear the flag
	_isrptr_CN = empty_handler;						//install user handler
	_CNIE = 0;									//disable interrupt
	//CNEN1 |= tmp; CNPU1 |= (pue==0)?0:tmp;					//enable CN interrupts
	//CNEN2 |= (tmp >> 16); CNPU2 |= (pue==0)?0:(tmp >> 16);	//enable or disable PUE
}

//cnx = 0..31
//activate a CN channel
void activateCNInterrupt(uint8_t cnx, CNWPU_TypeDef pue) {
	uint32_t tmp = 1ul << cnx;
	_CNIF = 0;									//clear the flag
	//_isrptr_CN = isrptr;						//install user handler
	CNEN1 |= tmp; CNPU1 |= (pue==CN_FLOAT)?0:tmp;					//enable CN interrupts
	CNEN2 |= (tmp >> 16); CNPU2 |= (pue==CN_FLOAT)?0:(tmp >> 16);	//enable or disable PUE
}

//disable user CN interrupt handler
void deactivateCNInterrupt(uint8_t cnx) {
	uint32_t tmp = 1ul << cnx;
	_CNIF = 0;									//clear the flag
	//_isrptr_CN = isrptr;						//install user handler
	CNEN1 &=~tmp; CNPU1 &=~tmp;					//disable CN interrupts
	CNEN2 &=~(tmp >> 16); CNPU2 &=~(tmp >> 16);	//disable or disable PUE
}
#endif										//use_PCINT


#if defined(PWM12RP) || defined(PWM22RP) || defined(PWM32RP) || defined(PWM42RP) || defined(PWM52RP)
//pwm output
//dc = 0x00..max 0xffff for pwm1/2/3/4/5
//RP4=PWM1, RP12=PWM2, RP13=PWM3, RP14=PWM4, RP15=PWM5
void analogWrite(uint8_t rpin, uint16_t dc) {
	dc = (dc > PWM_PR)?PWM_PR:dc;						//bound the range
	switch (rpin) {
		case 1: 										//configure pwm to RP4/OC1
			OC1RS= dc;									//load the period register
			break;
		case 2: 										//pin 12 not on GB002 -> had to map to pin 11
			OC2RS= dc;									//load the period register
			break;
		case 3: 										//configure pwm to RP13/OC3
			OC3RS= dc;									//load the period register
			break;
		case 4: 										//configure pwm to RP14/OC4
			OC4RS= dc;									//load the period register
			break;
		case 5: 										//configure pwm to RP15/OC5
			OC5RS= dc;									//load the period register
			break;
		default: 	break; //do nothing
	}
	
}
#endif										//use_pwm	

#if defined(USE_ADC1)
//read adc, 10-bit, right aligned
//ch = 0..15 (13=VddCore, 14 = Vbandgap / 2, 15 = Vbandgap)
uint16_t analogRead(AIN_TypeDef ch) {
	ch = ch & 0x0f;											//4 bits
	//configure ch as analog
	AD1PCFG &=~(1<<ch);										//0->analog mode, 1->digital
	AD1CHSbits.CH0SA = ch;									//set the adc channel bits
	AD1CON1bits.SAMP = 1;									//start the conversion
	while (AD1CON1bits.DONE==0) continue;					//wait for conversion to end - conversion complete if DONE = 1	
	return ADC1BUF0;										//return adc results
}

//set Vref
//Vref sources: 0->Vref = AVdd-AVss, 1->Vref+-AVss, 2->AVdd-Vref-, 3->Vref+ - Vref-
void analogReference(uint8_t Vref) {
	AD1CON2bits.VCFG = Vref & 0x07;							//select the source -> default is Vdd-Vss
}

#endif										//use_adc1

#if defined(SPI1SDO2RP)
//write data to spi
//order(LSBFIRST/MSBFIRST) is not used -> device is alwasy MSBFIRST
uint8_t SPI1Write(uint8_t order, uint8_t dat) {
	while (SPI1STAT & (1<<1)) continue;						//'1'->buffer full, '0'->buffer not full
	SPI1BUF = dat;											//load data the transmission starts
	while (!_SPI1IF) continue;								//wait for transmission to end
	_SPI1IF = 0;											//clear the flag
	return SPI1BUF;
}

//write a string of data to spi
uint8_t SPI1Writes(uint8_t order, uint8_t *dat, uint16_t length) {
	while (length--) {
		while (SPI1STAT & (1<<1)) continue;					//'1'->buffer full, '0'->buffer not full
		SPI1BUF = *dat++;									//load data the transmission starts
	}
	while (!_SPI1IF) continue;								//wait for transmission to end
	_SPI1IF = 0;											//clear the flag
	return SPI1BUF;
}
#endif
#if defined(SPI1SDI2RP)
//reads from spi
uint8_t SPI1Read(uint8_t order, uint8_t dat) {
	while (!_SPI1IF) continue;								//wait for transmission to end
	_SPI1IF = 0;											//clear the flag
	return SPI1BUF;
}
#endif														//spi1

#if defined(SPI2SDO2RP)
uint8_t SPI2Write(uint8_t order, uint8_t dat) {
	while (SPI2STAT & (1<<1)) continue;						//'1'->buffer full, '0'->buffer not full
	SPI2BUF = dat;											//load data the transmission starts
	while (!_SPI2IF) continue;								//wait for transmission to end
	_SPI2IF = 0;											//clear the flag
	return SPI2BUF;
}

//write a string of data to spi
uint8_t SPI2Writes(uint8_t order, uint8_t *dat, uint16_t length) {
	while (length--) {
		while (SPI2STAT & (1<<1)) continue;					//'1'->buffer full, '0'->buffer not full
		SPI2BUF = *dat++;									//load data the transmission starts
	}
	while (!_SPI2IF) continue;								//wait for transmission to end
	_SPI2IF = 0;											//clear the flag
	return SPI2BUF;
}
#endif
#if defined(SPI2SDI2RP)
//reads from spi
uint8_t SPI2Read(uint8_t order, uint8_t dat) {
	while (!_SPI2IF) continue;								//wait for transmission to end
	_SPI2IF = 0;											//clear the flag
	return SPI2BUF;
}
#endif														//use spi2

#if defined(USE_I2C1)
//send a start condition
uint8_t I2C1Start(void) {
	I2C1CONbits.SEN = 1;									//send a start condition
	while (I2C1CONbits.SEN) continue;
	return I2C1CONbits.SEN;
}

//senda  restart condition
uint8_t I2C1Restart(void) {
	I2C1CONbits.RSEN = 1;									//send a restart condition
	while (I2C1CONbits.RSEN) continue;
	return I2C1CONbits.RSEN;
}

//send a stop condition
uint8_t I2C1Stop(void) {
	I2C1CONbits.PEN = 1;									//send a stop condition
	while (I2C1CONbits.PEN) continue;
	return I2C1CONbits.PEN;
}

//write a char
uint8_t I2C1Write(uint8_t ch) {
	I2C1TRN = ch;											//send a char
	while (I2C1STATbits.TRSTAT) continue;
	return I2C1STATbits.ACKSTAT;
}

//read a char
uint8_t I2C1Read(uint8_t ack) {
	I2C1CONbits.RCEN = 1;									//send a master receive
	while (I2C1CONbits.RCEN) continue;
	I2C1STATbits.I2COV = 0;									//clear the flag
	if (ack==I2C_ACK) I2C1CONbits.ACKDT = 0;				//send ack
	else I2C1CONbits.ACKDT = 1;								//send nack
	I2C1CONbits.ACKEN=1;									//initialize an ack sequence
	while (I2C1CONbits.ACKEN) continue;						//wait for the ack to be sent
	return I2C1RCV;
}
#endif

#if defined(USE_I2C2)
//send a start condition
uint8_t I2C2Start(void) {
	I2C2CONbits.SEN = 1;									//send a start condition
	while (I2C2CONbits.SEN) continue;
	return I2C2CONbits.SEN;
}

//senda  restart condition
uint8_t I2C2Restart(void) {
	I2C2CONbits.RSEN = 1;									//send a restart condition
	while (I2C2CONbits.RSEN) continue;
	return I2C2CONbits.RSEN;
}

//send a stop condition
uint8_t I2C2Stop(void) {
	I2C2CONbits.PEN = 1;									//send a stop condition
	while (I2C2CONbits.PEN) continue;
	return I2C2CONbits.PEN;
}

//write a char
uint8_t I2C2Write(uint8_t ch) {
	I2C2TRN = ch;											//send a char
	while (I2C2STATbits.TRSTAT) continue;
	return I2C2STATbits.ACKSTAT;
}

//read a char
uint8_t I2C2Read(uint8_t ack) {
	I2C2CONbits.RCEN = 1;									//send a master receive
	while (I2C2CONbits.RCEN) continue;
	I2C2STATbits.I2COV = 0;									//clear the flag
	if (ack==I2C_ACK) I2C2CONbits.ACKDT = 0;				//send ack
	else I2C2CONbits.ACKDT = 1;								//send nack
	I2C2CONbits.ACKEN=1;									//initialize an ack sequence
	while (I2C2CONbits.ACKEN) continue;						//wait for the ack to be sent
	return I2C2RCV;
}
#endif

#if defined(U1TX2RP) || defined(U1TX2RP)
//initialize uart1
void serial1Begin(uint32_t bps) {
	//remap the TX/RX pins
	IO_UNLOCK(); 
#if defined(U1TX2RP)
	U1TX2RP();
#endif
#if defined(U1RX2RP)
	U1RX2RP();
#endif
	IO_LOCK();

	
	//enable power to periphearl
	PMD1bits.U1MD = 0;										//'0'->enable power, '1'->disable power
	U1MODE &=~(1<<15);										//'0'->disable module, '1'->enable module
	U1MODE = 	(0<<15) |										//'0'->disable module, '1'->enable module
				(0<<14) |									//'0'->continues to work in idle mode, '1'->discontinue in idle mode
				(0<<12) |									//'0'->disable irda, '1'->enable irda
				(0<<11) |									//'0'->rts in flow control, '1'->rts in simplex mode
				(0<< 8) |									//0->tx/rx pins are used, 1->tx/rx/rts pins are used, 2->tx/rx/rts/cts pins are used, 3->tx/rx/blk pins are used
				(0<< 7) |									//'0'->disable wake-up, '1'->enable wake-up
				(0<< 6) |									//'0'->disable loopback, '1'->enable loopback
				(0<< 5) |									//'0'->disable auto baud rate, '1'->enable auto baud rate
				(0<< 4) |									//'0'->rx not invented, '1'->rx invented
				(1<< 3) |									//'0'->high baud rate disabled, '1'->high baud rate enabled
				(0<< 1) |									//0->8-bit, no parity, 1->8-bit, even parity, 2->8-bit, odd parity, 3->9 bit, noparity
				(0<< 0) |									//'0'->1 stop bit, '1'->two stop bit
				0x00;
	U1BRG = SystemCoreClock / bps / ((U1MODEbits.BRGH==1)?4:16);				//genearte baud rate, based on UxMODE.BRG bit ('1'->high baud (/4), 0->low baud (/15))
	U1BRG = (U1BRG==0)?0:(U1BRG-1);							//lowest baud rate is 0
	U1STA = 0;												//reset value
	U1STAbits.UTXEN = 1;									//TX enabled
	
	U1MODE |= (1<<15);										//enable UART1
}

//write a char
void serial1Write(unsigned char ch) {
	while (U1STAbits.UTXBF == 1) continue;					//wait for transmission buffer to free up (UTXBF = 0)
	U1TXREG = ch;											//start the transmission
	//optional
	//while (U1STAbits.TRMT == 1) continue;					//wait for transmission to end (TRMT=1)

}
	
//write a string
void serial1Print(unsigned char *str) {

	do {
		while (U1STAbits.UTXBF == 1) continue;					//wait for transmission buffer to free up (UTXBF = 0)
		U1TXREG = *str;											//start the transmission
	} while (*str++);
	//optional
	//while (U1STAbits.TRMT == 1) continue;						//wait for transmission to end (TRMT=1)
}

//write a string with return
void serial1Println(unsigned char *str) {
	serial1Print(str);
	serial1Print((unsigned char *)"\n\r");
}

//read off uart
unsigned char serial1Read(void) {
	while (U1STAbits.TRMT == 1) continue;						//wait for transmission to end (TRMT=1)
	IFS0bits.U1RXIF = 0;
	return U1RXREG;
}

//test if number of data in the buffer
uint8_t serial1Available(void) {
	uint8_t tmp=IFS0bits.U1RXIF;
	IFS0bits.U1RXIF = 0;
	return (tmp)?1:0;
}
#endif														//use_uart1

#if defined(U2TX2RP) || defined(U2TX2RP)
//initialize uart2
void serial2Begin(uint32_t bps) {
	//remap the TX/RX pins
	IO_UNLOCK(); 
#if defined(U2TX2RP)
	U2TX2RP();
#endif
#if defined(U2RX2RP)
	U2RX2RP();
#endif
	IO_LOCK();

	//enable power to periphearl
	PMD1bits.U2MD = 0;										//'0'->enable power, '1'->disable power
	U2MODE &=~(1<<15);										//'0'->disable module, '1'->enable module
	U2MODE = 	(0<<15) |										//'0'->disable module, '1'->enable module
				(0<<14) |									//'0'->continues to work in idle mode, '1'->discontinue in idle mode
				(0<<12) |									//'0'->disable irda, '1'->enable irda
				(0<<11) |									//'0'->rts in flow control, '1'->rts in simplex mode
				(0<< 8) |									//0->tx/rx pins are used, 1->tx/rx/rts pins are used, 2->tx/rx/rts/cts pins are used, 3->tx/rx/blk pins are used
				(0<< 7) |									//'0'->disable wake-up, '1'->enable wake-up
				(0<< 6) |									//'0'->disable loopback, '1'->enable loopback
				(0<< 5) |									//'0'->disable auto baud rate, '1'->enable auto baud rate
				(0<< 4) |									//'0'->rx not invented, '1'->rx invented
				(1<< 3) |									//'0'->high baud rate disabled, '1'->high baud rate enabled
				(0<< 1) |									//0->8-bit, no parity, 1->8-bit, even parity, 2->8-bit, odd parity, 3->9 bit, noparity
				(0<< 0) |									//'0'->1 stop bit, '1'->two stop bit
				0x00;
	U2BRG = SystemCoreClock / bps / ((U2MODEbits.BRGH==1)?4:16) - 1*0;				//genearte baud rate, based on UxMODE.BRG bit ('1'->high baud (/4), 0->low baud (/15))
	U2BRG = (U2BRG==0)?0:(U2BRG-1);							//lowest baud rate is 0
	U2STA = 0;												//reset value
	U2STAbits.UTXEN = 1;									//TX enabled
	
	U2MODE |= (1<<15);										//enable UART1
}

//write a char
void serial2Write(unsigned char ch) {
	while (U2STAbits.UTXBF == 1) continue;					//wait for transmission buffer to free up (UTXBF = 0)
	U2TXREG = ch;											//start the transmission
	//optional
	//while (U2STAbits.TRMT == 1) continue;					//wait for transmission to end (TRMT=1)

}
	
//write a string
void serial2Print(unsigned char *str) {

	do {
		while (U2STAbits.UTXBF == 1) continue;					//wait for transmission buffer to free up (UTXBF = 0)
		U2TXREG = *str;											//start the transmission
	} while (*str++);
	//optional
	//while (U2STAbits.TRMT == 1) continue;						//wait for transmission to end (TRMT=1)
}

//write a string with return
void serial2Println(unsigned char *str) {
	serial2Print(str);
	serial2Print((unsigned char *)"\n\r");
}

//read off uart
unsigned char serial2Read(void) {
	while (U2STAbits.TRMT == 1) continue;						//wait for transmission to end (TRMT=1)
	IFS1bits.U2RXIF = 0;
	return U2RXREG;
}

//test if number of data in the buffer
uint8_t serial2Available(void) {
	uint8_t tmp=IFS1bits.U2RXIF;
	IFS1bits.U2RXIF = 0;
	return (tmp)?1:0;
}
#endif														//use_uart1

//reset the mcu
//start timer1 at 1:1 prescaler
void mcu_init(void) {
	//turn off all peripherals
	PMD1=0xffff; 
	PMD2=0xffff;
	PMD3=0xffff;
//#if defined(PMD4)		- only on GA102/GB002
#if 	defined(__PIC24FJ64GA102__) | defined(__PIC24FJ32GA102__) | \
		defined(__PIC24FJ64GA104__) | defined(__PIC24FJ32GA104__) | \
	 	defined(__PIC24FJ32GB002__) | defined (__PIC24FJ64GB002__) | \
	 	defined(__PIC24FJ32GB004__) | defined (__PIC24FJ64GB004__)
	PMD4=0xffff;
#endif

	//all pins digital
//#if defined(AD1PCFG)
	AD1PCFG = 0xffff;						//1->all pins digital
//#endif
	//or AD1PCFGH
//#if defined(AD1PCFGH)
	//AD1PCFGH = 0xffff;					//all pins digital
//#endif
	//or AD1PCFGL
//#if defined(AD1PCFGL)
	//AD1PCFGL = 0xffff;					//all pins digital
//#endif

	//initialize timer1 for time base
	//initialize timer_ticks
	timer_ticks=0;
	
	//configure timer1 as time base
	PMD1bits.T1MD = 0;						//enable power to tmr1
	T1CONbits.TON = 0;						//turn off rtc1
	PR1 = 0xffff;							//PR1=period-1;						//minimum rtc resolution is 1ms
	T1CONbits.TCS = 0;						//use internal clock = Fosc / 2
	T1CONbits.TCKPS=0x00;					//2-bit prescaler = 0b00 -> 1:1			//ps & TMR_PS_MASK;			//set prescaler to 1:1
	T1CONbits.TGATE = 0;					//rtc1 gate disabled
	//T1CONbits.T32 = 0;						//16-bit mode
	_T1IF = 0;								//reset the flag
	_T1IE = 1;								//tmr1 interrupt on
	TMR1 = 0;								//reset the timer/counter
	T1CONbits.TON = 1;						//turn on tmr1
	
	IO_UNLOCK(); 
#if defined(INT02RP)
	INT02RP();
#endif
#if defined(INT12RP)
	INT12RP();
#endif
#if defined(INT22RP)
	INT22RP();
#endif
#if defined(INT32RP)
	INT32RP();
#endif
#if defined(INT42RP)
	INT42RP();
#endif
	IO_LOCK();

#if defined(PWM12RP) || defined(PWM22RP) || defined(PWM32RP) || defined(PWM42RP) || defined(PWM52RP)
	//configure TMR2 as time base for PWM generation
	//initialize tmr2 for pwm
	PMD1bits.T2MD = 0;						//enable power to tmr2
	T2CONbits.TON = 0;						//turn off tmr2
	PR2 = PWM_PR;							//set the top of powm
	T2CONbits.TCS = 0;						//user internal clock = Fosc / 2
	T2CONbits.TCKPS = (PWMOUT_PS) & 0x03;	//2-bit prescaler = 0b00 -> 1:1, 1->8x, 2->64x, 3->256x
	T2CONbits.TGATE = 0;					//tmr2 gate disabled
	T2CONbits.T32 = 0;						//16-bit mode
	_T2IF = 0;								//reset the flag
	_T2IE = 0;								//tmr2 interrupt off
	TMR2 = 0;								//reset the timer/counter
	T2CONbits.TON = 1;						//turn on tmr2

//initialize the output compre / pwm module
#if 	defined(__PIC24FJ64GA002__) | defined (__PIC24FJ64GA004__) | \
		defined(__PIC24FJ48GA002__) | defined (__PIC24FJ48GA004__) | \
		defined(__PIC24FJ32GA002__) | defined (__PIC24FJ32GA004__) | \
		defined(__PIC24FJ16GA002__) | defined (__PIC24FJ16GA004__)
	PMD2bits.OC1MD = PMD2bits.OC2MD = PMD2bits.OC3MD = PMD2bits.OC4MD = PMD2bits.OC5MD = 0;		//'0'->enable the owm/pwm module
	OC1CONbits.OCTSEL = OC2CONbits.OCTSEL = OC3CONbits.OCTSEL = OC4CONbits.OCTSEL = OC5CONbits.OCTSEL = 0 ;					//'0'->TMR2 as the time base, '1'->TMR1 as the time base
	OC1CONbits.OCM = OC2CONbits.OCM = OC3CONbits.OCM = OC4CONbits.OCM = OC5CONbits.OCM = 0b110;	//'0'->turn the output compare module off, 0b110->PWM mode, OCFx disabled
	OC1RS = OC2RS = OC3RS = OC4RS = OC5RS = 1;
#else	//ga102+gb002
	PMD2bits.OC1MD = PMD2bits.OC2MD = PMD2bits.OC3MD = PMD2bits.OC4MD = PMD2bits.OC5MD = 0;		//'0'->enable the owm/pwm module
	OC1CON1bits.OCTSEL = OC2CON1bits.OCTSEL = OC3CON1bits.OCTSEL = OC4CON1bits.OCTSEL = OC5CON1bits.OCTSEL = 0 ;					//'0'->TMR2 as the time base, '1'->TMR1 as the time base
	OC1CON1bits.OCM = OC2CON1bits.OCM = OC3CON1bits.OCM = OC4CON1bits.OCM = OC5CON1bits.OCM = 0b110;	//'0'->turn the output compare module off, 0b110->PWM mode, OCFx disabled
	OC1CON2 = OC2CON2 = OC3CON2 = OC4CON2 = OC5CON2 = 0;										//OCx connected to peripheral
	OC1RS = OC2RS = OC3RS = OC4RS = OC5RS = 1;
#endif										//ga002
#endif										//use_pwm
	//configure pwm output pins
#if defined(PWM12RP)
	IO_UNLOCK(); PWM12RP(); IO_LOCK();
#endif
#if defined(PWM22RP)
	IO_UNLOCK(); PWM22RP(); IO_LOCK();
#endif
#if defined(PWM32RP)
	IO_UNLOCK(); PWM32RP(); IO_LOCK();
#endif
#if defined(PWM42RP)
	IO_UNLOCK(); PWM42RP(); IO_LOCK();
#endif
#if defined(PWM52RP)
	IO_UNLOCK(); PWM52RP(); IO_LOCK();
#endif

			
#if defined(USE_ADC1)
	//initialize the adc module
	PMD1bits.ADC1MD = 0;					//0->enable ad1
	AD1CON1bits.ADON = 0;					//0->adc is off, 1->adc is on
	AD1CON1bits.FORM = 0;					//0->adc results right aligned
	AD1CON1bits.SSRC = 0b111;				//0b111->internal sampling starts conversion, 0->clearing SAMP bit to start conversion
	AD1CON1bits.ASAM = 0;					//0->use SAMP to start adc conversion, 1->auto set SAMP bit
	
	AD1CON2bits.VCFG = 0;					//0->Vref = AVdd-AVss, 1->Vref+-AVss, 2->AVdd-Vref-, 3->Vref+ - Vref-
	AD1CON2bits.CSCNA= 0;					//0->single conversion, 1->scan conversion
	AD1CON2bits.SMPI = 0;					//0->interrupts at end of each conversion
	AD1CON2bits.BUFM = 0;					//16-bit word as buffer
	AD1CON2bits.ALTS = 0;					//always use mux A as input
	
	AD1CON3bits.ADRC = 0;					//ADC driven by systems clock
	AD1CON3bits.SAMC = 31;					//auto-sample time bits: 1..31 Tad
	AD1CON3bits.ADCS = ((ADC1_CS)>64)?64:(ADC1_CS);				//conversion clock: # of Tcy. 0..64

	AD1CSSL = 0;							//scanning disabled

	//AD1CON1bits.ADON = 1;					//0->adc is off, 1->adc is on
	
	//only mux A is used -> see AD1CON2bits.ALTS bit
	AD1CHSbits.CH0NA = 0;					//negative to Vref-
	//configure AD1CHSbits.CH0SA in adc routine
	
	AD1CON1bits.ADON = 1;					//1->turn on adc
	
#endif

#if defined(USE_PCINT)	
	//disable CN/PUE
	CNEN1 = CNEN2 = CNPU1 = CNPU2 = 0;
#endif

#if defined(SPI1SCK2RP) || defined(SPI1SDO2RP) || defined(SPI1SDI2RP)
	//configure the pins
	IO_UNLOCK();									//start the unlock sequence
#if defined(SPI1SCK2RP)
	SPI1SCK2RP();
#endif
#if defined(SPI1SDO2RP)
	SPI1SDO2RP();
#endif
#if defined(SPI1SDI2RP)
	SPI1SDI2RP();
#endif
	IO_LOCK();
	
	//turn on power to peripheral
	PMD1bits.SPI1MD = 0;							//'0'->enable power to peripherals
	//configure peripheral
	//turn off peripheral
	SPI1STAT &=~(1<<15);							//'0'->disable peripheral, '1'->enable peripheral
	SPI1CON1  = (0<<12) |							//'0'->enable SCLK, '1'->disable SCLK
				(0<<11) |							//'0'->SDOx enabled, '1'->SDOx disabled
				(0<<10) |							//'0'->8-bit spi, '1'->16-bit spi
				(1<< 9) |							//'0'->data sampled at the middle of the slock, '1'->data sampled at the end of the clock
				(0<< 8) |							//'0'->data captured on first sclk transition, '1'->data captured on 2nd sclk transition
				(0<< 7) |							//'0'->slave select pin disable, '1'->slave select pin enabled
				(0<< 6) |							//'0'->sclk idles low, '1'->sclk idles high
				(1<< 5) |							//'0'->slave mode, '1'->master mode
				(6<< 2) |							//3-bit spi 2ndary prescaler, 111->1x, 110->2x (default), ..., 000->8x
				(((SPI1_PS) & 0x03) << 0) |			//2-bit spi primary prescaler, 0->1x, 2->4x, 3->16x, 4->64x
				0x00;
	SPI1CON2  =	0x0001;								//use default value
	
	//clean the flag and disable interrupts
	SPI1STAT  = 0x00;								//clear all flag / interrupts
	
	//assign pins to peripheral
	
	//turn on peripheral
	SPI1STAT |= (1<<15);							//turn on the peripheral

#endif


#if defined(SPI2SCK2RP) || defined(SPI2SDO2RP) || defined(SPI2SDI2RP)
	//configure the pins
	IO_UNLOCK();									//start the unlock sequence
#if defined(SPI2SCK2RP)
	SPI2SCK2RP();
#endif
#if defined(SPI2SDO2RP)
	SPI2SDO2RP();
#endif
#if defined(SPI2SDI2RP)
	SPI2SDI2RP();
#endif
	IO_LOCK();

	//turn on power to peripheral
	PMD1bits.SPI2MD = 0;							//'0'->enable power to peripherals
	//configure peripheral
	//turn off peripheral
	SPI2STAT &=~(1<<15);							//'0'->disable peripheral, '1'->enable peripheral
	SPI2CON1  = (0<<12) |							//'0'->enable SCLK, '1'->disable SCLK
				(0<<11) |							//'0'->SDOx enabled, '1'->SDOx disabled
				(0<<10) |							//'0'->8-bit spi, '1'->16-bit spi
				(1<< 9) |							//'0'->data sampled at the middle of the slock, '1'->data sampled at the end of the clock
				(0<< 8) |							//'0'->data captured on first sclk transition, '1'->data captured on 2nd sclk transition
				(0<< 7) |							//'0'->slave select pin disable, '1'->slave select pin enabled
				(0<< 6) |							//'0'->sclk idles low, '1'->sclk idles high
				(1<< 5) |							//'0'->slave mode, '1'->master mode
				(6<< 2) |							//3-bit spi 2ndary prescaler, 111->1x, 110->2x (default), ..., 000->8x
				(((SPI2_PS) & 0x03) << 0) |			//2-bit spi primary prescaler, 0->1x, 2->4x, 3->16x, 4->64x
				0x00;
	SPI2CON2  =	0x0001;								//use default value
	
	//clean the flag and disable interrupts
	SPI2STAT  = 0x00;								//clear all flag / interrupts
	
	//assign pins to peripheral
	
	//turn on peripheral
	SPI2STAT |= (1<<15);							//turn on the peripheral
#endif
	
#if defined(USE_I2C1)
	//turn on power to peripheral
	PMD1bits.I2C1MD = 0;							//'0'->enable power to peripherals
	//configure peripheral
	//turn off peripheral
	I2C1CON &=~(1<<15);								//'0'->disable periopheral, '1'->enable peripheral
	I2C1CON = 	(0<<15) |							//'0'->disable periopheral, '1'->enable peripheral
				(0<<13) |							//'0'->operates in idle, '1'-> not operate in idle
				(1<<12) |							//'0'->hold sck low (in slave mode), '1'->do not hold sck low
				(0<<11) |							//'0'->disable intelligent peripheral management , '1'->enable it
				(0<<10) |							//'0'->7-bit slave address, '1'->10 bit slave address
				(0<< 9) |							//'0'->enable slew rate control, '1'->disable slew rate control
				(0<< 8) |							//'0'->disable smbus input thresholds, '1'->enable smbus input thresholds
				(0<< 7) |							//'0'->disable general call, '1'->enable general call
				(0<< 6) |							//'0'->disable clock stretching, '1'->enable clock stretch
				(0<< 5) |							//'0'->send ack during acknowledge, '1'->sends NACK during acknowledge
				(0<< 4) |							//'0'->ack sequence not in progress, '1'->initiates acknowledge sequence
				(0<< 3) |							//'0'->receives sequence not in progress, '1'->enable receive mode for i2c
				(0<< 2) |							//'0'->stop condition not in progress, '1'->initiates stop condition
				(0<< 1) |							//'0'->restart not in progress, '1'->initializes restart
				(0<< 0) |							//'0'->start condition not in progress, '1'->initializes start condition
				0x00;
	I2C1STAT = 0x0000;								//default value.
	I2C1MSK = 0x0000;								//default value
	I2C1BRG = F_CPU / F_I2C1 + 2*0;					//minimum of 2
	I2C1BRG = (I2C1BRG<=2)?2:(I2C1BRG-1);
	//clean the flag and disable interrupts
	//turn on peripheral
	I2C1CON |= (1<<15);								//turn on the peripheral
#endif

#if defined(USE_I2C2)
	//turn on power to peripheral
	PMD3bits.I2C2MD = 0;							//'0'->enable power to peripherals
	//configure peripheral
	//turn off peripheral
	I2C2CON &=~(1<<15);								//'0'->disable periopheral, '1'->enable peripheral
	I2C2CON = 	(0<<15) |							//'0'->disable periopheral, '1'->enable peripheral
				(0<<13) |							//'0'->operates in idle, '1'-> not operate in idle
				(1<<12) |							//'0'->hold sck low (in slave mode), '1'->do not hold sck low
				(0<<11) |							//'0'->disable intelligent peripheral management , '1'->enable it
				(0<<10) |							//'0'->7-bit slave address, '1'->10 bit slave address
				(0<< 9) |							//'0'->enable slew rate control, '1'->disable slew rate control
				(0<< 8) |							//'0'->disable smbus input thresholds, '1'->enable smbus input thresholds
				(0<< 7) |							//'0'->disable general call, '1'->enable general call
				(0<< 6) |							//'0'->disable clock stretching, '1'->enable clock stretch
				(0<< 5) |							//'0'->send ack during acknowledge, '1'->sends NACK during acknowledge
				(0<< 4) |							//'0'->ack sequence not in progress, '1'->initiates acknowledge sequence
				(0<< 3) |							//'0'->receives sequence not in progress, '1'->enable receive mode for i2c
				(0<< 2) |							//'0'->stop condition not in progress, '1'->initiates stop condition
				(0<< 1) |							//'0'->restart not in progress, '1'->initializes restart
				(0<< 0) |							//'0'->start condition not in progress, '1'->initializes start condition
				0x00;
	I2C2STAT = 0x0000;								//default value.
	I2C2MSK = 0x0000;								//default value
	I2C2BRG = F_CPU / F_I2C2 + 2*0;					//minimum of 2
	I2C2BRG = (I2C2BRG<=2)?2:(I2C2BRG-1);
	//clean the flag and disable interrupts
	//turn on peripheral
	I2C2CON |= (1<<15);								//turn on the peripheral
#endif

	//enable all interrupts
	interrupts();
}

//templated code for main()
int main(void) {
	mcu_init();							//reset the chip
	setup();							//user-set up the code
	while (1) {
		loop();							//user specified loop
	}
	return 0;
}
		