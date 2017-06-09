#ifndef _PICDUINO_CONFIG_H
#define _PICDUINO_CONFIG_H
//configuration bits for PICduino16

#if 	defined(__PIC24FJ64GA002__) | defined (__PIC24FJ64GA004__) | \
		defined(__PIC24FJ48GA002__) | defined (__PIC24FJ48GA004__) | \
		defined(__PIC24FJ32GA002__) | defined (__PIC24FJ32GA004__) | \
		defined(__PIC24FJ16GA002__) | defined (__PIC24FJ16GA004__)

/*config fues settings for everything else - need customization*/
//CONFIG1
#pragma config JTAGEN = OFF
#pragma config GCP = OFF
#pragma config GWRP = OFF
#pragma config BKBUG = OFF
#pragma config COE = OFF
#pragma config ICS = PGx2
#pragma config FWDTEN = OFF
#pragma config WINDIS = OFF
#pragma config FWPSA = PR128
#pragma config WDTPS = PS32768

//CONFIG2
#pragma config IESO = OFF
#pragma config WUTSEL = LEG
#pragma config SOSCSEL = SOSC
#pragma config FNOSC = PRI					//FRC, FRCPLL, PRI, PRIPLL, SOSC, LPRC, FRCDIV
#pragma config FCKSM = CSDCMD
#pragma config OSCIOFNC = OFF
#pragma config IOL1WAY = OFF
#pragma config I2C1SEL = PRI
#pragma config POSCMOD = HS					//EC, XT, HS, NONE

#elif 	defined(__PIC24FJ64GA102__) | defined (__PIC24FJ64GA104__) | \
		defined(__PIC24FJ32GA102__) | defined (__PIC24FJ32GA104__)

//config fues settings for 24fj64ga102
///config fues settings
//CONFIG1
#pragma config JTAGEN = OFF
#pragma config GCP = OFF
#pragma config GWRP = OFF
#pragma config ICS = PGx2
#pragma config FWDTEN = OFF
#pragma config WINDIS = OFF
#pragma config FWPSA = PR128
#pragma config WDTPS = PS32768

//CONFIG2
#pragma config IESO = OFF
#pragma config FNOSC = PRI					//FRC, FRCPLL, PRI, PRIPLL, SOSC, LPRC, FRCDIV
#pragma config FCKSM = CSDCMD
#pragma config OSCIOFNC = OFF
#pragma config IOL1WAY = OFF
#pragma config POSCMOD = HS					//EC, XT, HS, NONE
#pragma config I2C1SEL = PRI

//CONFIG3
#pragma config WPFP = WPFP0
#pragma config SOSCSEL = IO
#pragma config WUTSEL = LEG
#pragma config WPDIS = WPDIS
#pragma config WPCFG = WPCFGDIS
#pragma config WPEND = WPENDMEM

//CONFIG4
#pragma config DSWDTPS = DSWDTPSF
#pragma config DSWDTOSC = LPRC
#pragma config RTCOSC = LPRC
#pragma config DSBOREN = OFF
#pragma config DSWDTEN = OFF

//config bits for dsPIC33FJ32GP302/304, dsPIC33FJ64GPX02/X04, AND dsPIC33FJ128GPX02/X04
#elif 	defined(__dsPIC33FJ32GP302__) | defined (__dsPIC33FJ64GP202__) | defined (__dsPIC33FJ64GP802__) | defined (__dsPIC33FJ128GP202__) | defined (__dsPIC33FJ128GP802__) | \
	 	defined(__dsPIC33FJ32GP304__) | defined (__dsPIC33FJ64GP204__) | defined (__dsPIC33FJ64GP804__) | defined (__dsPIC33FJ128GP204__) | defined (__dsPIC33FJ128GP804__)
#pragma config BWRP = WRPROTECT_OFF
#pragma config BSS = NO_BOOT_CODE
#pragma config RBS = NO_BOOT_RAM
#pragma config SWRP = WRPROTECT_OFF
#pragma config SSS = NO_SEC_CODE
#pragma config RSS = NO_SEC_RAM
#pragma config GWRP = OFF
#pragma config GSS = OFF
#pragma config FNOSC = PRI			//PRI, PRIPLL, FRC, FRCPLL, FRCDIV16, FRCDIVN, LPRC, SOSC
#pragma config IESO = OFF
#pragma config POSCMD = HS			//EC, HS, XT, NONE
#pragma config OSCIOFNC = OFF
#pragma config IOL1WAY = OFF
#pragma config FCKSM = CSDCMD
#pragma config WDTPOST = PS32768
#pragma config WDTPRE = PR128
#pragma config WINDIS = OFF
#pragma config FWDTEN = OFF
#pragma config FPWRT = PWR128
#pragma config ALTI2C = OFF
#pragma config ICS = PGD2
#pragma config JTAGEN = OFF

//config bits for PIC24FJ32GB002/004, PIC24FJ64GB002/004
#elif 	defined(__PIC24FJ32GB002__) | defined (__PIC24FJ64GB002__) | \
	 	defined(__PIC24FJ32GB004__) | defined (__PIC24FJ64GB004__)
//CONFIG1
#pragma config JTAGEN = OFF
#pragma config GCP = OFF
#pragma config GWRP = OFF
#pragma config ICS = PGx2
#pragma config FWDTEN = OFF
#pragma config WINDIS = OFF
#pragma config FWPSA = PR128
#pragma config WDTPS = PS32768

//CONFIG2
#pragma config IESO = OFF
#pragma config PLLDIV = DIV12
#pragma config PLL96MHZ = ON
#pragma config FNOSC = PRI					//PRI, PRIPLL, FRC, FRCPLL, FRCDIV, LPRC, SOSC
#pragma config FCKSM = CSDCMD
#pragma config OSCIOFNC = OFF
#pragma config IOL1WAY = OFF
#pragma config POSCMOD = HS					//EC, HS, XT, NONE
#pragma config I2C1SEL = PRI

//CONFIG3
#pragma config WPFP = WPFP0
#pragma config SOSCSEL = IO
#pragma config WUTSEL = LEG
#pragma config WPDIS = WPDIS
#pragma config WPCFG = WPCFGDIS
#pragma config WPEND = WPENDMEM

//CONFIG4
#pragma config DSWDTPS = DSWDTPSF
#pragma config DSWDTOSC = LPRC
#pragma config RTCOSC = LPRC
#pragma config DSBOREN = OFF
#pragma config DSWDTEN = OFF

//config bits for //PIC24FV08KM102/202/204, PIC24FV16KM102/104/202/204, PIC24F08KM102/202/204, PIC24F16KM102/104/202/204
//chip used on microstick I/II
#elif 	defined(__PIC24FV08KM102__) | defined(__PIC24FV08KM202__) | defined(__PIC24FV16KM102__) | defined(__PIC24FV16KM202__) | defined(__PIC24F08KM102__)  | defined (__PIC24F08KM202__)  | defined(__PIC24F16KM102__)  | defined (__PIC24F16KM202__) | \
		defined(__PIC24FV08KM204__) | defined(__PIC24FV16KM104__) | defined(__PIC24FV16KM204__) | defined(__PIC24F08KM204__)  | defined(__PIC24F16KM104__) | defined(__PIC24F16KM204__)

//CONFIG1
#pragma config BWRP = OFF
#pragma config BSS = OFF
#pragma config GWRP = OFF
#pragma config GCP = OFF
#pragma config FNOSC = PRI			//PRI, PRIPLL, FRC, FRCPLL, SOSC, LPRC, LPFRC, FRCDIV
#pragma config SOSCSRC = DIG
#pragma config LPRCSEL = HP
#pragma config IESO = OFF
#pragma config POSCMOD = HS			//EC, HS, XT, NONE
#pragma config OSCIOFNC = IO
#pragma config POSCFREQ = HS			//HS (>8Mhz), MS, LS (<100K)
#pragma config SOSCSEL = SOSCLP
#pragma config FCKSM = CSDCMD
#pragma config WDTPS = PS32768
#pragma config FWPSA = PR128
#pragma config FWDTEN = OFF
#pragma config WINDIS = OFF
#pragma config BOREN = BOR0
#pragma config RETCFG = OFF
#pragma config PWRTEN = OFF
#pragma config I2C1SEL = PRI
#pragma config BORV = V18
#pragma config MCLRE = OFF
#pragma config ICS = PGx1			//PGx1, PGx2, PGx3

#else
#warning "no configuration setting for this device"
#endif
#endif

