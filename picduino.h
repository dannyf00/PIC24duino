//PICduino GPIO functions for PIC24-family chips
//compiler: XC16 (developed and tested under v1.11)
//
//version: 0.14b@7/5/2017
//fixed a bug on cyclesPerMillisecond() and cyclesPerMicrosecond() - they are reversely in the original code
//
//version: 0.14a@ 6/5/2017
//i2c1/2 implemented
//added ticks() to return raw timer ticks
//
//version: 0.14 $ 6/4/2017
//uart1/2 implemented
//spi1/2 implemented
//remappable pins implemented
//module customization implemented
//
//version: 0.13 @ 5/29/2017
//attachInterrupt()/detachInterrupt(), and attachCNInterrupt()/detachCNInterrupt() implemented
//
//version: 0.12a @ 5/28/2017
//deleted support for PIC24F/FV, dsPIC33 chips. Total supported chips = 16 chips
//supported devices:
//PIC24FJ16/32/48/64GA002
//PIC24FJ16/32/48/64GA004
//PIC24FJ32/64GA102
//PIC24FJ32/64GA104
//PIC24FJ32GB002/004
//PIC24FJ64GB002/004
//
//pulseIn(), analogWrite() and analogRead() implemented
//
//version: 0.12
//5/21/2017: added support for the following chip, total of 26 + 14 = 40 chips
//PIC24FV08KM102/202/204, PIC24FV16KM102/104/202/204
//PIC24F08KM102/202/204, PIC24F16KM102/104/202/204
//
//version: 0.11
//5/20/2017: added support for the following chip, total of 12+14=26 chips
//dsPIC33FJ32GP302/304,
//dsPIC33FJ64GPX02/X04, AND
//dsPIC33FJ128GPX02/X04
//PIC24FJ32GB002/004
//PIC24FJ64GB002/004
//
//version: 0.10
//5/20/2017: initial release
//supported device list: total of 12
//PIC24FJ16/32/48/64GA002
//PIC24FJ16/32/48/64GA004
//PIC24FJ32/64GA102
//PIC24FJ32/64GA104
//
//arduino functions supported:
//GPIO: pinMode(), digitalWrite(), digitalRead()
//Timing: millis(), micros(), delay(), delayMicroseconds()
//Advanced IO: shiftOut(), shiftIn()
//Math: min(), max(), abs(), constrain(), map(), pow(), sqrt()
//Trigonometry: sin(), cos(), tan()
//Bits and Bytes: lowByte(), highByte(), bitRead(), bitWrite(), bitSet(), bitClear(), bit()
//Interrupts: interrupts(), noInterrupts()
//Random Numbers: randomSeed(), random(max). random(min, max) mapped to random2(min, max)
//Characters: isAlphaNumeric(), isAlpha(), isAscii(), isWhitespace(), isControl(), isDigit(), isGraph(), isLowerCase(), isPrintable(), isPunct(), isSpace(), isUpperCase(), isHexadeimalDigit()
//
//not yet implemented
//External Interrupts: attachInterrupt(), detachInterrupt()
//Advanced IO: tone(), noTone(), pulseIn()
//AnalogIO: analogReference(), analogRead(), analogWrite()

#ifndef __PICDUINO_H
#define __PICDUINO_H

//#include <p24fxxxx.h>								//we use pic24f - C30
#include <xc.h>										//we use XC16
#include <stdint.h>									//standard types
#include <stdlib.h>									//we use rand()
#include <stdio.h>									//we use sprintf

//PIC24duino module configuration
//GPIO is always available
#define USE_ADC1									//comment out if ADC1 is not used
#define USE_CNINT									//comment out if CN interrupts are not used
#define USE_I2C1									//comment out if I2C1 is not used
#define USE_I2C2									//comment out if I2C2 is not used

//PIC24duino remappable pin configuration
//#define PWM12RP()		PPS_OC1_TO_RP(4)			//PWM1 output mapped to RPn
//#define PWM22RP()		PPS_OC2_TO_RP(12)			//PWM2 output mapped to RPn
//#define PWM32RP()		PPS_OC3_TO_RP(13)			//PWM3 output mapped to RPn
//#define PWM42RP()		PPS_OC4_TO_RP(14)			//PWM4 output mapped to RPn
//#define PWM52RP()		PPS_OC5_TO_RP(15)			//PWM5 output mapped to RPn
//#define INT02RP()		PPS_INT0_TO_RP(1)			//INT0 input mapped to RPn
//#define INT12RP()		PPS_INT1_TO_RP(1)			//INT1 input mapped to RPn
//#define INT22RP()		PPS_INT2_TO_RP(1)			//INT2 input mapped to RPn
//#define INT32RP()		PPS_INT3_TO_RP(1)			//INT3 input mapped to RPn
//#define SPI1SCK2RP()	PPS_SCK1OUT_TO_RP(5)		//SPI1 SCK1 output to RPn
//#define SPI1SDO2RP()	PPS_SDO1_TO_RP(6)			//SPI1 SDO output to RPn
//#define SPI1SDI2RP()	PPS_SDI1_TO_RP(7)			//SPI1 SDI output to RPn
//#define SPI2SCK2RP()	PPS_SCK2OUT_TO_RP(8)		//SPI2 SCK1 output to RPn
//#define SPI2SDO2RP()	PPS_SDO2_TO_RP(9)			//SPI2 SDO output to RPn
//#define SPI2SDI2RP()	PPS_SDI2_TO_RP(10)			//SPI2 SDI output to RPn
#define U1TX2RP()		PPS_U1TX_TO_RP(0)			//u1tx to rp0
//#define U1RX2RP()		PPS_U1RX_TO_RP(1)			//u1rx to rp1
#define U2TX2RP()		PPS_U2TX_TO_RP(2)			//u2tx to rp0
//#define U2RX2RP()		PPS_U2RX_TO_RP(3)			//u2rx to rp1
//end PIC24duino remappable pin configuration

//PIC24duino hardware configuration
#define PWMOUT_BITs		15							//in bits. PWM output / analogWrite() resolution. 8-14 suggested
#define PWMOUT_PS		0							//2-bit prescaler for pwm time base. 0->1x, 1->8x, 2->64x, 3->256x
#define ADC1_CS			64							//adc conversion clock. [0..64]
#define SPI1_PS			2							//2-bit spi prescaler. 0->1x, 1->4x, 2->16x, 3->64x
#define SPI2_PS			3							//2-bit spi prescaler. 0->1x, 1->4x, 2->16x, 3->64x
#define F_TONE			1000						//beep frequency, in Hz. 500 - 8000. 
#define F_I2C1			400000ul					//I2C1 frequency, in Hz
#define F_I2C2			100000ul					//I2C2 frequency, in Hz
//end PIC24duino configuration

//global definitions
//pin definitions. need to match GPIO_PinDef[]
typedef enum {
	PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,
	PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,
#if defined(GPIOC)
	PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15,
#endif
#if defined(GPIOD)
	PD0, PD1, PD2, PD3, PD4, PD5, PD6, PD7, PD8, PD9, PD10, PD11, PD12, PD13, PD14, PD15,
#endif
#if defined(GPIOE)
	PE0, PE1, PE2, PE3, PE4, PE5, PE6, PE7, PE8, PE9, PE10, PE11, PE12, PE13, PE14, PE15,
#endif
#if defined(GPIOF)
	PF0, PF1, PF2, PF3, PF4, PF5, PF6, PF7, PF8, PF9, PF10, PF11, PF12, PF13, PF14, PF15,
#endif
#if defined(GPIOG)
	PG0, PG1, PG2, PG3, PG4, PG5, PG6, PG7, PG8, PG9, PG10, PG11, PG12, PG13, PG14, PG15,
#endif
} PIN_TypeDef;

//analog input channel.
typedef enum {
	A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16,
} AIN_TypeDef;

//cn weak pull-up
typedef enum {
	CN_FLOAT,
	CN_WPUEN
} CNWPU_TypeDef;
	
#ifndef F_CPU										//allows user specified F_CPU in the IDE
#define F_CPU				(16000000ul/2)			//cpu runs at 8Mhz/2 -> F_xtal = 8Mhz (internal RC)
#endif
#define SystemCoreClock		F_CPU

//Period for PWM for TMR2/3/4/5. TMR1 PR is fixed at 0xffff
#define PWM_PR				((1ul<<PWMOUT_BITs) - 1)			//12-bit pwm -> F_PWM = F_CPU / 2/ PWM_PR
#define I2C_ACK				0			//send i2c ack
#define I2C_NACK			(!I2C_ACK)	//send i2c nack
#define I2C_READ			0x01		//i2c command for read
#define I2C_WRITE			0x00		//i2c command for write

//for C30 only
#ifndef uint8_t 
typedef unsigned char uint8_t;
#endif
#ifndef int8_t
typedef signed char int8_t;
#endif
#ifndef uint16_t
typedef unsigned short uint16_t;
#endif
#ifndef int16_t
typedef signed short int16_t;
#endif
#ifndef uint32_t
typedef unsigned long uint32_t;
#endif
#ifndef int32_t
typedef signed long int32_t;
#endif

//port manipulation macros for PIC.
//port manipulation macros - register-based
#define IO_SET(port, bits)	port |= (bits)			//set bits on port
#define IO_CLR(port, bits)	port &=~(bits)			//clear bits on port
#define IO_FLP(port, bits)	port ^= (bits)			//flip bits on port
#define IO_GET(port, bits)	((port) & (bits))		//return bits on port
#define IO_OUT(ddr, bits)	ddr &=~(bits)			//set bits as output
#define IO_IN(ddr, bits)	ddr |= (bits)			//set bits as input

//gpio definitions - GPIO-based
#define GIO_SET(gpio, bits)	IO_SET(gpio->LAT, bits)		//gpio->LAT |= (bits)			//set bits on gpio
#define GIO_CLR(gpio, bits)	IO_CLR(gpio->LAT, bits)		//gpio->LAT &=~(bits)			//clear bits on gpio
#define GIO_FLP(gpio, bits)	IO_FLP(gpio->LAT, bits)		//gpio->LAT ^= (bits)			//flip bits on gpio
#define GIO_GET(gpio, bits)	IO_GET(gpio->PORT, bits)	//((gpio->PORT) & (bits))		//return bits on gpio
#define GIO_OUT(gpio, bits)	IO_OUT(gpio->TRIS, bits)	//ddr->TRIS &=~(bits)			//set bits as output
#define GIO_IN(gpio, bits)	IO_IN(gpio->TRIS, bits)		//ddr->TRIS |= (bits)			//set bits as input


#define NOP()				Nop()						//asm("nop")					//nop()
#define NOP2()				{NOP(); NOP();}
#define NOP4()				{NOP2(); NOP2();}
#define NOP8()				{NOP4(); NOP4();}
#define NOP16()				{NOP8(); NOP8();}
#define NOP16()				{NOP8(); NOP8();}
#define NOP24()				{NOP16(); NOP8();}
#define NOP32()				{NOP16(); NOP16();}
#define NOP40()				{NOP32(); NOP8();}

//interrupts
#ifndef sei
#define sei()				//ei()						//for arduino compatability
#endif

#ifndef cli
#define cli()				//di()
#endif

//reset the mcu
void mcu_init();									//reset the mcu


//gpio definitions
typedef struct {
	volatile unsigned short TRIS;		//direction register - offset 0x0000
	volatile unsigned short PORT;		//input data register
	volatile unsigned short LAT;		//output data register
	volatile unsigned short ODC;		//open drain register
} GPIO_TypeDef;							//gpio type definitions

#define GPIOA				((GPIO_TypeDef *) &TRISA)
#define GPIOB				((GPIO_TypeDef *) &TRISB)
#if defined(TRISC)
#define GPIOC            	((GPIO_TypeDef *) &TRISC)
#endif
#if defined(TRISD)
#define GPIOD				((GPIO_TypeDef *) &TRISD)
#endif
#if defined(TRISE)
#define GPIOE				((GPIO_TypeDef *) &TRISE)
#endif
#if defined(TRISF)
#define GPIOF				((GPIO_TypeDef *) &TRISF)
#endif
#if defined(TRISG)
#define GPIOG				((GPIO_TypeDef *) &TRISG)
#endif

typedef struct {
	GPIO_TypeDef *gpio;					//gpio for a pin
	uint16_t mask;						//pin mask
} PIN2GPIO;
	
#define INPUT				0
#define OUTPUT				1			//(!INPUT)
#define INPUT_PULLUP		2

#define LOW					0
#define HIGH				1			//(!LOW)

#define PI 					3.1415926535897932384626433832795
#define HALF_PI 			(PI / 2)							//1.5707963267948966192313216916398
#define TWO_PI 				(PI + PI)							//6.283185307179586476925286766559
#define DEG_TO_RAD 			(TWO_PI / 360)						//0.017453292519943295769236907684886
#define RAD_TO_DEG 			(360 / TWO_PI)						//57.295779513082320876798154814105
#define EULER 				2.718281828459045235360287471352	//Euler's number

#define SERIAL  			0x0
#define DISPLAY 			0x1

#define LSBFIRST 			0
#define MSBFIRST 			1									//(!LSBFIRST)							//1

#define CHANGE 				1
#define FALLING 			2
#define RISING 				3

#ifndef min
#define min(a,b) 			((a)<(b)?(a):(b))
#endif
#ifndef max
#define max(a,b) 			((a)>(b)?(a):(b))
#endif
#define abs(x) 				((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     		((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) 		((deg)*DEG_TO_RAD)
#define degrees(rad) 		((rad)*RAD_TO_DEG)
#define sq(x) 				((x)*(x))

#define interrupts() 		sei()
#define noInterrupts() 		cli()

#define clockCyclesPerMicrosecond() 	( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) 	( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) 	( (a) * clockCyclesPerMicrosecond() )

#define lowByte(w) 			((uint8_t) ((w) & 0xff))
#define highByte(w) 		((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) 	((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#define bit(n)				(1ul<<(n))

#define false				0
#define true				(!false)

//characters
#define isAlphaNumeric(c)	isalnum(c)
#define isAlpha(c)			isalpha(c)
#define isAscii(c)			isascii(c)
#define isWhitespace(c)		isblank(c)
#define isControl(c)		iscntrl(c)
#define isDigit(c)			isdigit(c)
#define isGraph(c)			isgraph(c)
#define isLowerCase(c)		islower(c)
#define isPrintable(c)		isprint(c)
#define isPunct(c)			ispunct(c)
#define isSpace(c)			isspace(c)
#define isUpperCase(c)		isupper(c)
#define isHexadecimalDigit(c)	isxdigit(c)

//external setup/loop - defined by user
extern void setup(void);
extern void loop(void);

//random number
#define randomSeed(seed)	srand(seed)
#define random(max)			random2(0, max)
#define random2(min, max)	((min) + (int32_t) ((max) - (min)) * rand() / 32768)

//GPIO
void pinMode(PIN_TypeDef pin, uint8_t mode);
void digitalWrite(PIN_TypeDef pin, uint8_t mode);
int digitalRead(PIN_TypeDef pin);

//time base
uint32_t ticks(void);
uint32_t millis(void);
uint32_t micros(void);
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);
#define cyclesPerMicrosecond()			(F_CPU / 1000000ul)
#define cyclesPerMillisecond()			(F_CPU / 1000ul)

//advanced IO
void tone(void);									//tone frequency specified by F_TONE in STM8Sduino.h
void noTone(void);
//shiftin/out: bitOrder = MSBFIRST or LSBFIRST
uint8_t shiftIn(PIN_TypeDef dataPin, PIN_TypeDef clockPin, uint8_t bitOrder);
void shiftOut(PIN_TypeDef dataPin, PIN_TypeDef clockPin, uint8_t bitOrder, uint8_t val);
uint32_t pulseIn(PIN_TypeDef pin, uint8_t state);		//wait for a pulse and return timing

//pwm output
//dc = 0x00..0x0fff for pwm2/3/4/5, 0x00..0xffff for pwm1
//RP4=PWM1, RP12=PWM2, RP13=PWM3, RP14=PWM4, RP15=PWM5
void analogWrite(uint8_t rpin, uint16_t dc);

//analog read on ADC1
//read DRL first for right aligned results
uint16_t analogRead(AIN_TypeDef ain);

//analog reference - default to AVdd-AVss
//Vref sources: 0->Vref = AVdd-AVss, 1->Vref+-AVss, 2->AVdd-Vref-, 3->Vref+ - Vref-
void analogReference(uint8_t Vref);

//interrupts
//install external interrupt handler
//mode 0: falling edge, 1: rising edge
void attachInterrupt(uint8_t intx, void (*isrptr) (void), uint8_t mode);
void detachInterrupt(uint8_t intx);

//change notification interrupts
//install user CN interrupt handler
void attachCNInterrupt(void (*isrptr) (void));
void detachCNInterrupt(void);
void activateCNInterrupt(uint8_t cnx, CNWPU_TypeDef pue);
void deactivateCNInterrupt(uint8_t cnx);

//SPI1
uint8_t SPI1Write(uint8_t order, uint8_t dat);
uint8_t SPI1Writes(uint8_t order, uint8_t *dat, uint16_t length);
uint8_t SPI1Read(uint8_t order, uint8_t dat);

//SPI2
uint8_t SPI2Write(uint8_t order, uint8_t dat);
uint8_t SPI2Writes(uint8_t order, uint8_t *dat, uint16_t length);
uint8_t SPI2Read(uint8_t order, uint8_t dat);

//i2c1
uint8_t I2C1Start(void);
uint8_t I2C1Restart(void);
uint8_t I2C1Stop(void);
uint8_t I2C1Write(uint8_t dat);
uint8_t I2C1Read(uint8_t ack);

//i2c2
uint8_t I2C2Start(void);
uint8_t I2C2Restart(void);
uint8_t I2C2Stop(void);
uint8_t I2C2Write(uint8_t dat);
uint8_t I2C2Read(uint8_t ack);

//serial
//uart1
void serial1Begin(uint32_t bps);
void serial1Write(unsigned char ch);
void serial1Print(unsigned char *str);
void serial1Println(unsigned char *str);
uint8_t serial1Available(void);

//uart2
void serial2Begin(uint32_t bps);
void serial2Write(unsigned char ch);
void serial2Print(unsigned char *str);
void serial2Println(unsigned char *str);
uint8_t serial2Available(void);

//timing
uint32_t ticks(void);

//macros for remappable pins
//iolock/unlock sequence
//unlock IOLOCK
#define IO_UNLOCK()	{asm volatile ( "MOV #OSCCON, w1 \n" \
					"MOV #0x46, w2 \n" \
					"MOV #0x57, w3 \n" \
					"MOV.b w2, [w1] \n" \
					"MOV.b w3, [w1] \n" \
					"BCLR OSCCON,#6"); \
					}

//lock IOLOCK
#define IO_LOCK()	{asm volatile ( "MOV #OSCCON, w1 \n" \
					"MOV #0x46, w2 \n" \
					"MOV #0x57, w3 \n" \
					"MOV.b w2, [w1] \n" \
					"MOV.b w3, [w1] \n" \
					"BSET OSCCON, #6" ); \
					}

//for peripheral pin select (PPS)
#if defined(_INT1R)
/// Maps INT1 to a remappable pin;
/// see PPS_xxx_TO_RP(pin) for more informatino.
#define PPS_INT1_TO_RP(pin) _INT1R = pin
#else
#define PPS_INT1_TO_RP(pin)
#endif


#if defined(_INT2R)
#define PPS_INT2_TO_RP(pin) _INT2R = pin
#else
#define PPS_INT2_TO_RP(pin)
#endif

#if defined(_T2CKR)
#define PPS_T2CK_TO_RP(pin) _T2CKR = pin
#else
#define PPS_T2CK_TO_RP(pin)
#endif

#if defined(_T3CKR)
#define PPS_T3CK_TO_RP(pin) _T3CKR = pin
#else
#define PPS_T3CK_TO_RP(pin)
#endif

#if defined(_T4CKR)
#define PPS_T4CK_TO_RP(pin) _T4CKR = pin
#else
#define PPS_T4CK_TO_RP(pin)
#endif

#if defined(_T5CKR)
#define PPS_T5CK_TO_RP(pin) _T5CKR = pin
#else
#define PPS_T5CK_TO_RP(pin)
#endif

#if defined(_IC1R)
#define PPS_IC1_TO_RP(pin) _IC1R = pin
#else
#define PPS_IC1_TO_RP(pin)
#endif

#if defined(_IC2R)
#define PPS_IC2_TO_RP(pin) _IC2R = pin
#else
#define PPS_IC2_TO_RP(pin)
#endif

#if defined(_IC3R)
#define PPS_IC3_TO_RP(pin) _IC3R = pin
#else
#define PPS_IC3_TO_RP(pin)
#endif

#if defined(_IC4R)
#define PPS_IC4_TO_RP(pin) _IC4R = pin
#else
#define PPS_IC4_TO_RP(pin)
#endif

#if defined(_IC5R)
#define PPS_IC5_TO_RP(pin) _IC5R = pin
#else
#define PPS_IC5_TO_RP(pin)
#endif

#if defined(_IC6R)
#define PPS_IC6_TO_RP(pin) _IC6R = pin
#else
#define PPS_IC6_TO_RP(pin)
#endif

#if defined(_IC7R)
#define PPS_IC7_TO_RP(pin) _IC7R = pin
#else
#define PPS_IC7_TO_RP(pin)
#endif

#if defined(_IC8R)
#define PPS_IC8_TO_RP(pin) _IC8R = pin
#else
#define PPS_IC8_TO_RP(pin)
#endif

#if defined(_OCFAR)
#define PPS_OCFA_TO_RP(pin) _OCFAR = pin
#else
#define PPS_OCFA_TO_RP(pin)
#endif

#if defined(_OCFBR)
#define PPS_OCFB_TO_RP(pin) _OCFBR = pin
#else
#define PPS_OCFB_TO_RP(pin)
#endif

#if defined(_U1RXR)
#define PPS_U1RX_TO_RP(pin) _U1RXR = pin
#else
#define PPS_U1RX_TO_RP(pin)
#endif

#if defined(_U1CTSR)
#define PPS_U1CTS_TO_RP(pin) _U1CTSR = pin
#else
#define PPS_U1CTS_TO_RP(pin)
#endif

#if defined(_U2RXR)
#define PPS_U2RX_TO_RP(pin) _U2RXR = pin
#else
#define PPS_U2RX_TO_RP(pin)
#endif

#if defined(_U2CTSR)
#define PPS_U2CTS_TO_RP(pin) _U2CTSR = pin
#else
#define PPS_U2CTS_TO_RP(pin)
#endif

#if defined(_SDI1R)
#define PPS_SDI1_TO_RP(pin) _SDI1R = pin
#else
#define PPS_SDI1_TO_RP(pin)
#endif

#if defined(_SCK1R)
#define PPS_SCK1IN_TO_RP(pin) _SCK1R = pin
#else
#define PPS_SCK1IN_TO_RP(pin)
#endif

#if defined(_SS1R)
#define PPS_SS1IN_TO_RP(pin) _SS1R = pin
#else
#define PPS_SS1IN_TO_RP(pin)
#endif

#if defined(_SDI2R)
#define PPS_SDI2_TO_RP(pin) _SDI2R = pin
#else
#define PPS_SDI2_TO_RP(pin)
#endif

#if defined(_SCK2R)
#define PPS_SCK2IN_TO_RP(pin) _SCK2R = pin
#else
#define PPS_SCK2IN_TO_RP(pin)
#endif

#if defined(_SS2R)
#define PPS_SS2IN_TO_RP(pin) _SS2R = pin
#else
#define PPS_SS2IN_TO_RP(pin)
#endif

#if defined(_C1RXR)
#define PPS_C1RXR_TO_RP(pin) _C1RXR = pin
#else
#define PPS_C1RXR_TO_RP(pin)
#endif

#if defined(_C2RXR)
#define PPS_C2RXR_TO_RP(pin) _C2RXR = pin
#else
#define PPS_C2RXR_TO_RP(pin)
#endif
//@}


//end RP input mapping
//Your device may not have all of these peripherals!

//start RP output mapping


#if defined(_RP0R)
/// Maps C1OUT to a remappable pin;
/// see PPS_yyy_TO_RP(pin) for more informatino.
#define PPS_C1OUT_TO_RP(pin) _RP##pin##R = 1
#else
#define PPS_C1OUT_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_C2OUT_TO_RP(pin) _RP##pin##R = 2
#else
#define PPS_C2OUT_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_U1TX_TO_RP(pin) _RP##pin##R = 3
#else
#define PPS_U1TX_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_U1RTS_TO_RP(pin) _RP##pin##R = 4
#else
#define PPS_U1RTS_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_U2TX_TO_RP(pin) _RP##pin##R = 5
#else
#define PPS_U2TX_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_U2RTS_TO_RP(pin) _RP##pin##R = 6
#else
#define PPS_U2RTS_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_SDO1_TO_RP(pin) _RP##pin##R = 7
#else
#define PPS_SDO1_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_SCK1OUT_TO_RP(pin) _RP##pin##R = 8
#else
#define PPS_SCK1OUT_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_SS1OUT_TO_RP(pin) _RP##pin##R = 9
#else
#define PPS_SS1OUT_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_SDO2_TO_RP(pin) _RP##pin##R = 10
#else
#define PPS_SDO2_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_SCK2OUT_TO_RP(pin) _RP##pin##R = 11
#else
#define PPS_SCK2OUT_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_SS2OUT_TO_RP(pin) _RP##pin##R = 12
#else
#define PPS_SS2OUT_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_C1TX_TO_RP(pin) _RP##pin##R = 16
#else
#define PPS_C1TX_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_OC1_TO_RP(pin) _RP##pin##R = 18
#else
#define PPS_OC1_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_OC2_TO_RP(pin) _RP##pin##R = 19
#else
#define PPS_OC2_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_OC3_TO_RP(pin) _RP##pin##R = 20
#else
#define PPS_OC3_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_OC4_TO_RP(pin) _RP##pin##R = 21
#else
#define PPS_OC4_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_OC5_TO_RP(pin) _RP##pin##R = 22
#else
#define PPS_OC5_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_CTPLS_TO_RP(pin) _RP##pin##R = 29
#else
#define PPS_CTPLS_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_C3OUT_TO_RP(pin) _RP##pin##R = 30
#else
#define PPS_C3OUT_TO_RP(pin)
#endif

#endif //gpio_h_
