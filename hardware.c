/*
 * communication.c
 *
 * Created: 4/19/2011 2:38:26 PM
 *  Author: Milan Romic *  Author: robert kovacs */
#include "Headers/avr_compiler.h"
#include "Headers/usart_driver.h"
#include "Headers/TC_driver.h"
#include "Headers/globals.h"
#include "Headers/mechanism.h"
#include "Headers/hardware.h"
#include "Headers/port_driver.h"
#include "Headers/adc_driver.h"


void Podesi_Parametre_Robota(void)
{
	//mehanicke karakteristike
	metar = 57075;		//57000 joxo-treba da je u redu!	 57100 kada su tockvi glibavi				 //75000; //broj inkremenata za 1m - eksperiment!      /39035*2 izracunata vrednost 88,5
	krug360 = 53250;	//54000	joxo-valja pogledati jos jednom											//49650 - eksperiment 1;  //66250 - matematika;	//broj inkremenata za jedan krug - eksperiment!		//13653
	
	scale_factor_for_mm = metar / 1000;
	krug180 = krug360 >> 1;	
	krug90 =  krug360 >> 2;
	krug45 =  krug360 >> 3;
	krug180_PI = krug180 / M_PI;	
	double flag_krug2 = atan2((double)(-500), (double)(-500));
	flag_krug = (signed long)(flag_krug2 *krug180_PI);

	smer_zadati = 1;						//1-napred, 2-nazad, 0-sam bira smer
	zeljena_pravolinijska_brzina = 450;		//brzina kojom se pravo krece robot
	zeljena_brzina_okretanja = 300; //brzina kojom se okrece robot
	max_brzina_motora = 800;				//eksperimentalno utvrdjena max brzina motora [impuls/vreme_odabiranja(3ms)] (max je oko 1000)
	
	modifikovana_zeljena_pravolinijska_brzina = zeljena_pravolinijska_brzina;
	rezervni_ugao = krug45/45;		//vrednost ugaone greske preko koje se radi reorijentacija robota  
	PWM_perioda = 800;			//PWM tajmer broji do 800 - frekvenicja 20KHz
}

void Podesi_PID_Pojacanja(void)
{
	//PID parametri
	//Regulacija pravolinijskog kretanja
	Kp_pravolinijski = 5;						//5
	Ki_pravolinijski = 1;						//1
	Kd_pravolinijski = 150;						//0.2
	Kp_teta_pravolinijski = 20;					//20	isto kao Kp_teta
	
	//Regulacija ugaonog zakretanja
	Kp_teta = 10;	//20
	Ki_teta = 0.5;	//1.2
	Kd_teta = 20;	//20
	Kp_teta_okretanje = 1.5;	//ne koristi se nigde u kodu :)
		
	//Regulacija brzine
	Kp_brzina = 0.35;	//Ko menja Kp_brzina ovde treba da promeni i u mechanism.c, ne znam koja linija koda jer jox nema linije na svom kompu
	Ki_brzina = 0;
	Kd_brzina = 0;

	//Ubrzavanje po rampi
	Accel_PID_pos = 2;	//bilo 2
}

void Podesi_QDEC(void)
{
	//enkoderski ulazi
	PORT_ConfigurePins( &PORTD, 0xFF, false, false, PORT_OPC_PULLUP_gc, PORT_ISC_FALLING_gc);
	PORT_SetPinsAsInput( &PORTD, 0xFF );
	
	//Levi pasivni PORTD6,7
	PORTD.PIN6CTRL |= 0b01000000;	//invertuje se pin, da enkoder broji u drugom smeru
	EVSYS.CH0MUX = 0b01101110;		// PORTD6 mux input -> MOTOR_QDEC.LEFT
	EVSYS.CH0CTRL = 0b00001001;		//enable QDEC, filtering 2x
	TCD1.CTRLA = 0b00000001;		//clock source
	TCD1.CTRLD = 0b01101000;  		//event action=QDEC, event source=ch0 [01101000]
	TCD1.INTCTRLA = ( TCD1.INTCTRLA & ~TC1_OVFINTLVL_gm ) | 1;	//interapt na owerflov

	//Desni pasivni	PORTD4,5
	//PORTD.PIN4CTRL |= 0b01000000;	//invertuje se pin, da enkoder broji u drugom smeru
	EVSYS.CH2MUX = 0b01101100;			//PORTD4 mux input -> PASIVNI_DESNI
	EVSYS.CH2CTRL = 0b00001001;			//enable QDEC, filtering 2x
	TCD0.CTRLA = 0b00000001;			//clock source
	TCD0.CTRLD = 0b01101010;  			//event action=QDEC, event source=ch2
	TCD0.INTCTRLA = ( TCD0.INTCTRLA & ~TC0_OVFINTLVL_gm ) | 1;
}

void PodesiADC(void)
{
	//ADCB.CTRLA = 0b00000101; //ch1 single conversion EN, ADC EN
	//ADCB.CTRLB = 0b00000100; //8 bitna rez
	//ADCB.REFCTRL = 0b00000010; //bandgap
	//ADCB.EVCTRL = 0b00000000;	//
	

	/* Move stored calibration values to ADC B. */
	ADC_CalibrationValues_Set(&ADCA);

	/* Get offset value for ADC A. */
 	offset = ADC_Offset_Get(&ADCA);

	/* Set up ADC B to have signed conversion mode and 12 bit resolution. */
  	ADC_ConvMode_and_Resolution_Config(&ADCA, false, ADC_RESOLUTION_8BIT_gc);

	/* Sample rate is CPUFREQ/8. Allow time for storing data. */
	ADC_Prescaler_Config(&ADCA, ADC_PRESCALER_DIV4_gc);

	/* Set referance voltage on ADC B to be VCC-0.6 V.*/
	ADC_Referance_Config(&ADCA, ADC_REFSEL_INT1V_gc);

	/* Setup channel 0 to have single ended input. */
	ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,
	                                 ADC_CH_INPUTMODE_SINGLEENDED_gc,
	                                 ADC_CH_GAIN_1X_gc);
	/* Setup channel 1 to have single ended input. */
	ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH1,
										ADC_CH_INPUTMODE_SINGLEENDED_gc,
										ADC_CH_GAIN_1X_gc);

	/* Set input to the channel in ADC B to be PIN 1. */	//Levi motor pin0
	ADC_Ch_InputMux_Config(&ADCA.CH0, ADC_CH_MUXPOS_PIN0_gc, ADC_CH_MUXNEG_PIN0_gc);
	/* Set input to the channel in ADC B to be PIN 1. */	//Desni motor pin1
	ADC_Ch_InputMux_Config(&ADCA.CH1, ADC_CH_MUXPOS_PIN1_gc, ADC_CH_MUXNEG_PIN0_gc);
	
	/* Setup sweep of all four virtual channels. */
	ADC_SweepChannels_Config(&ADCA, ADC_SWEEP_01_gc);

	/* Enable ADC B with free running mode, Vcc reference and unsigned conversion.*/
	ADC_Enable(&ADCA);

	/* Wait until common mode voltage is stable. Default clk is 2MHz and
	 * therefore below the maximum frequency to use this function. */
	ADC_Wait_32MHz(&ADCA);

	/* Enable free running mode. */
	ADC_FreeRunning_Enable(&ADCA);

}

void Podesi_Interapt(void)
{
	// Enable high level interrupts in the PMIC. 
	PMIC.CTRL |= PMIC_HILVLEN_bm;
	// Enable medium level interrupts in the PMIC. 
	PMIC.CTRL |= PMIC_MEDLVLEN_bm;
	// Enable low level interrupts in the PMIC. 
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
	//round-robin algoritam EN
	PMIC.CTRL |= PMIC_RREN_bm;
	
	sei(); // enable all inerrupts
}

void Podesi_Oscilator(void)
{
	//Startovanje internog oscilatora od 32MHz
	OSC.CTRL = OSC_RC32MEN_bm;

	do {
		//Sacekaj dok se stabilizuje oscilator
	} while ( ( OSC.STATUS & OSC_RC32MRDY_bm ) == 0 );
	
	//Ukljuci preskalere B i C
	CCP = CCP_IOREG_gc;
	CLK.PSCTRL = CLK_PSBCDIV_2_2_gc;

	//Postavi 32MHz kao glavni oscilator
	CCP = CCP_IOREG_gc;
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;	
}

void Podesi_USART_Komunikaciju(void)
{
	//USART_E1 - BT - 115200
	PORTE.DIR |= PIN7_bm;//PE7 (TXE1) - izlaz  
	PORTE.DIR  &= ~PIN6_bm;//PE6 (RXE1) - ulaz
	USART_InterruptDriver_Initialize(&USART_E1_data, &USARTE1, USART_DREINTLVL_LO_gc);//Koriscenje USARTE1 (definisano u globals.h) i inicijalizacija buffer-a
	USART_Format_Set(USART_E1_data.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);//USARTE1, 8 Data bits, No Parity, 1 Stop bit.
	USART_RxdInterruptLevel_Set(USART_E1_data.usart, USART_RXCINTLVL_LO_gc);//Aktiviranje RXC interrupt-a
	USART_Baudrate_Set(&USARTE1,107, -5  );//Podesavanje Baud rate	//9600
	USART_Rx_Enable(USART_E1_data.usart);	//Ukljucivanje RX i TX
	USART_Tx_Enable(USART_E1_data.usart);
	
	//USART_E0 BT_RS232 - Salje na LOGIKU - 19200
	PORTE.DIR |= PIN3_bm;//PE3 (TXE0) - izlaz
	PORTE.DIR  &= ~PIN2_bm;//PE2 (RXE0) - ulaz
	USART_InterruptDriver_Initialize(&USART_E0_data, &USARTE0, USART_DREINTLVL_LO_gc);//Koriscenje USARTE0 i inicijalizacija buffer-a
	USART_Format_Set(USART_E0_data.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);//USARTE0, 8 Data bits, No Parity, 1 Stop bit.
	USART_RxdInterruptLevel_Set(USART_E0_data.usart, USART_RXCINTLVL_LO_gc);//Aktiviranje RXC interrupt-a
	USART_Baudrate_Set(&USARTE0,12,1);  //9600---> 3269, -6      //107, -5---->115200 //12,1 ----> 19200
	USART_Rx_Enable(USART_E0_data.usart);//Ukljucivanje RX i TX
	USART_Tx_Enable(USART_E0_data.usart);
	

	//USART_C0 - Xmega_USB - 115200
	PORTC.DIR &= PIN3_bm;//PE3 (TXE0) - izlaz
	PORTC.DIR  |= ~PIN2_bm;	//PE2 (RXE0) - ulaz
	USART_InterruptDriver_Initialize(&USART_C0_data, &USARTC0, USART_DREINTLVL_LO_gc);//Koriscenje USARTE0 i inicijalizacija buffer-a
	USART_Format_Set(USART_C0_data.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false); 	//USARTE0, 8 Data bits, No Parity, 1 Stop bit.
	USART_RxdInterruptLevel_Set(USART_C0_data.usart, USART_RXCINTLVL_LO_gc);//Aktiviranje RXC interrupt-a
	USART_Baudrate_Set(&USARTC0, 107, -5 );	//Podesavanje Baud rate//115200
	USART_Rx_Enable(USART_C0_data.usart);//Ukljucivanje RX i TX
	USART_Tx_Enable(USART_C0_data.usart);

}

void Podesi_Tajmere(void)
{
	//SPISAK TAJMERA:
	//TCC0 - SLOBODAN (FET izlazi)
	//TCC1 - SLOBODAN (FET izlazi)
	//TCD0 - QDEC enkoder (motor_levi)
	//TCD1 - QDEC enkoder (motor_desni)
	//TCE0 - QDEC enkoder (pasivni_levi)
	//TCE1 - system tajmer (3ms)
	//TCF0 - servo signal generator
	//TCF1 - motor PWM (levi, desni)
	
	//System tajmer za uzorkovanje enkodera i PID regulaciju
	/* Set period ( TOP value ). */
	TC_SetPeriod( &TCE1, 0x002F ); //0x00BF = 12ms //0x5F = 6ms //0x2F = 3ms <- Mirko //Nasa -> //0x5DC0
	/* Enable overflow interrupt at low level */
	TC1_SetOverflowIntLevel( &TCE1, TC_OVFINTLVL_MED_gc );
	/* Start Timer/Counter. */
	TC1_ConfigClockSource( &TCE1, TC_CLKSEL_DIV256_gc );
	
	////Tajmer za servo - TCF0
	//PORT_SetPinsAsOutput( &PORTF, 0b00001111 );
	///* Set period ( TOP value ). */
	//TCF0.PER = 0x09C4;	//2500	~ 20ms perioda 
	//TCF0.CTRLB |= 0b11110011; //CCDEN, CCCEN, CCBEN, CCAEN, X, WGMMODE2, WGMODE1, WGMODE0
	///* Start Timer/Counter. */
	//TC0_ConfigClockSource( &TCF0, TC_CLKSEL_DIV64_gc );
}

void Podesi_PWM(void)
{	
	HIRESF_CTRLA = 0b00000010; //enable hi-res za TCF1
	// Configure timer 
	TCF1.PER = PWM_perioda;
	TCF1.CTRLB = TC1_CCBEN_bm | TC1_CCAEN_bm | TC_WGMODE_DS_T_gc;
	TCF1.CTRLA = TC_CLKSEL_DIV1_gc;
	
	PORT_SetPinsAsOutput( &PORTF, 0b00110000 ); //PF5 - PWM_L, PF4 - PWM_D
}

void Podesi_Pinove(void)
{
	//PORTB - digitalni ulazi 
	
	PORT_SetPinsAsInput( &PORTB, 0xFF );
	PORT_ConfigurePins( &PORTB,
						0xFF,
						false,
						false,
						PORT_OPC_PULLUP_gc,
						PORT_ISC_FALLING_gc);
						
	//PORTC - digitalni izlazi
						
	PORT_SetPinsAsOutput(&PORTC,0xFF);
	PORT_ConfigurePins(&PORTC,
	0xFF,
	false,
	false,
	PORT_OPC_PULLDOWN_gc,
	PORT_ISC_BOTHEDGES_gc);
	PORT_ClearPins(&PORTC, 0xFF);
	
	//podesavanje interrupt0 za PORTB.0 - ISR(PORTB_INT0_vect)
	PORT_ConfigureInterrupt0( &PORTB, PORT_INT0LVL_LO_gc, 0x01 );
	
	//PORTC - FET izlazi 
	//PORT_SetPinsAsOutput(&PORTC, 0xFF);
//  	PORT_MapVirtualPort0( PORTCFG_VP0MAP_PORTC_gc );	//mapiranje virtualnog porta 0 na PORTC
//  	PORT_SetDirection( &VPORT0, 0xFF );
//		VPORT0.OUT = 0x00;	//clear
	
	//pinovi za upravljanje moorima
	PORT_SetPinsAsOutput(&PORTH, 0xFF); //PH0-IN_A1, PH1-EN_A1, PH2-EN_B1, PH3-IN_B1, PH4-IN_A2, PH5-EN_A2, PH6-EN_B2, PH7-IN_B2
	PORT_ClearPins(&PORTH, 0xFF);
	PORT_SetPins(&PORTH, 0b01100110); // EN ALL	

	//servo izlazi
	PORT_SetPinsAsOutput(&PORTF, 0x0F);
	PORT_ClearPins(&PORTF, 0x0F);
	
}

