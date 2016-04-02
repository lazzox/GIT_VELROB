/*
 * funkcije.c
 *
 * Created: 16/11/15 14:36:10
 *  Author: marko
 */ 

#include <avr/io.h>
#include "Headers/funkcije.h"
#include "Headers/avr_compiler.h"
#include "Headers/usart_driver.h"
#include "Headers/port_driver.h"
#include "Headers/adc_driver.h"
#include "math.h"
#include "Headers/globals.h"
#include "Headers/mechanism.h"
#include "Headers/hardware.h"

static char step1 = 0;
static char flag1 = 0;

volatile signed long
X_pos,
Y_pos,
X_cilj,
Y_cilj,
teta,
teta_cilj,
teta_cilj_final;



void nuliraj_poziciju_robota(void)
{
	X_pos = X_cilj = 0;
	Y_pos = Y_cilj = 0;
	teta = teta_cilj = teta_cilj_final = 0;
	smer_zadati = smer_trenutni = 2; //Idi pravo
	TCD0.CNT = 0;			//Desni enkoder
	TCD1.CNT = 0;			//Levi enkoder
	count_L = 0;
	count_R = 0;
	last_count_R = 0;
	last_count_L = 0;
}

void zadaj_X_Y_teta(signed long x, signed long y, signed long teta_des, unsigned char dir)
{
	X_cilj = x * scale_factor_for_mm;
	Y_cilj = y * scale_factor_for_mm;
	teta_cilj_final = (teta_des * krug360) / 360;
	smer_zadati = dir;
}

void zadaj_X_Y(signed long x, signed long y, unsigned char dir)
{
	X_cilj = x * scale_factor_for_mm;
	Y_cilj = y * scale_factor_for_mm;
	smer_zadati = dir;
}

void zadaj_teta(signed long teta_des, unsigned char dir)
{
	teta_cilj_final = (teta_des * krug360) / 360;
	smer_zadati = dir;
}

void idi_pravo(signed long x, signed long y, unsigned long ugao)
{
	//if (meca == 1){
		//zadaj_teta(45,0);
	//}
	//else{
		//zadaj_teta(90,0);
	//}
	//if(meca==1){
		//if (sys_time>2666)
		//{
			//X_cilj = x * scale_factor_for_mm;
			//Y_cilj = y * scale_factor_for_mm;
			//teta_cilj_final = (ugao * krug360) / 360;
			//smer_zadati = 1;
			//sys_time=0;
		//}	
	//}	
	//else if(meca==0 && neca==1 ){
		//X_cilj = x * scale_factor_for_mm;
		//Y_cilj = y * scale_factor_for_mm;
		//teta_cilj_final = (ugao * krug360) / 360;
		//smer_zadati = 1;
	//}
	
	X_cilj = x * scale_factor_for_mm;
	Y_cilj = y * scale_factor_for_mm;
	
	teta_cilj_final = (ugao * krug360) / 360;
	smer_zadati = 1;

	//X_cilj = x * scale_factor_for_mm;
	//Y_cilj = y * scale_factor_for_mm;
	//teta_cilj_final = (ugao * krug360) / 360;
	//smer_zadati = 1;
	
}




void idi_unazad(signed long x, signed long y, unsigned long ugao)
{
	//zadaj_X_Y_teta(0,0,0,1);
	
	X_cilj = x * scale_factor_for_mm;
	Y_cilj = y * scale_factor_for_mm;
	teta_cilj_final = (ugao * krug360) / 360;
	smer_zadati = 2;
}






void sendMsg(char *poruka)
{
	while(*poruka != '\0'){
		sendChar(*poruka);
		poruka++;
	}
}

void sendChar(char c)
{
	USARTE0.DATA = c;
	while(!(USARTE0.STATUS & (1 << 5)));
}

void inicijalizuj_servo_tajmer_20ms()
{
	PORTF.DIR |= (1 << 0);	//servo 1
	
	//Clock source = 32/4 MHz = 8 MHz
	TCF0.CTRLA |= (1 << 2 | 1 << 0);						//Set presclaer to 64, PER 2500 = 20 ms
	TCF0.CTRLB |= (0x0F << 4 | 0x03 << 0);					//Enable Capture/compare A,B,C,D and select single slope PWM
	TCF0.INTCTRLA |= (1 << 0);								//Enable low level overflow interrupt
	TCF0.INTCTRLB |= (1 << 0 | 1 << 2 | 1 << 4 | 1 << 6);	//Enable Capture/compare low level interrupts
	TCF0.PER = 2500;
}

void pomeri_servo_1(uint16_t deg)
{
	uint16_t res = (uint16_t)(deg*(250/180));	//250 cycles for 180 degree turn
	if(res <= 0)
		res = 125;								//125 cycles for 0 degree turn
	else if(res > 250)
		res = 250;
	TCF0.CCA = res;
}

ISR(TCF0_CCA_vect)
{
	PORTF.OUT |= (1 << 0);
}

ISR(TCF0_OVF_vect)
{
	PORTF.OUT &= ~(1 << 0);
}

 void demo_1(void)
 {
	switch(step1)
	{
		case 0:
			if(flag1 == 0)
			{
			
				stigao_flag0 = 0;
				flag1 = 1;
				//zadaj_teta(45,0);
				idi_pravo(700,0,180);
			
				//sendChar('0');
			}
			else if(stigao_flag0 == 1)
			{
				step1++;
				flag1 = 0;
				sys_time=0;
			}
			break;
		
 		case 1:
 			
 				if(flag1 == 0)
 				{
 					stigao_flag0 = 0;
 					flag1 = 1;
 					//idi_pravo(500,0,0);
 					idi_pravo(700,300,0);
 			
 					//sendChar('0');
 				}
 				else if(stigao_flag0 == 1)
 				{
 					step1++;
 					flag1 = 0;
 					sys_time=0;
 				}
 			
 			break;
 			
//  			case 2:
//  			if(sys_time>1533)
//  			{
//  				if(flag1 == 0)
//  				{
//  					stigao_flag0 = 0;
//  					flag1 = 1;
//  					//idi_pravo(500,0,0);
//  					idi_pravo(2500,0,0);
//  					
//  					//sendChar('0');
//  				}
//  				else if(stigao_flag0 == 1)
//  				{
//  					step1++;
//  					flag1 = 0;
//  					sys_time=0;
//  				}
//  			}
//  			break;
// 			
// 			case 3:
// 			if(sys_time>1533)
// 			{
// 				if(flag1 == 0)
// 				{
// 					stigao_flag0 = 0;
// 					flag1 = 1;
// 					//idi_pravo(500,0,0);
// 					idi_unazad(0,0,0);
// 					
// 					//sendChar('0');
// 				}
// 				else if(stigao_flag0 == 1)
// 				{
// 					step1++;
// 					flag1 = 0;
// 					sys_time=0;
// 				}
// 			}
// 			break;
// 		
		default:
		break;
		
 	}
 }

void proba (void){
	switch(step1)
	{
		case 0:
		if(flag1 == 0){
			stigao_flag0 = 0;
			flag1 = 1;
			zadaj_teta(90,0);
			//idi_pravo(-500,-500,0);
			// zadaj_X_Y(-500,0,2);
			sendChar('0');
		}
		else if(stigao_flag0 == 1){
			step1++;
			flag1 = 0;
		}
		break;
		
		//case 1:
		//if(flag1 == 0){
			//stigao_flag0 = 0;
			//flag1 = 1;
			//idi_unazad(0,0,0);
			//// zadaj_X_Y(-500,0,2);
			//sendChar('1');
		//}
		//else if(stigao_flag0 == 1){
			//step1++;
			//flag1 = 0;
		//}
		//break;
		
		default:
		break;
	}
}

void kocka(void)
{
	
	switch(step1)
	{
		case 0:
		if(flag1 == 0){
			stigao_flag0 = 0;
			flag1 = 1;
			idi_pravo(400,0,0);
			// zadaj_X_Y(-500,0,2);
			sendChar('0');
		}
		else if(stigao_flag0 == 1){
			step1++;
			flag1 = 0;
			sys_time=0;
		}
		break;
		
		case 1:
		if(flag1 == 0){
			stigao_flag0 = 0;
			flag1 = 1;
			idi_pravo(400,400,0);
			// zadaj_X_Y(-500,0,2);
			sendChar('1');
		}
		else if(stigao_flag0 == 1){
			step1++;
			flag1 = 0;
			sys_time=0;
			
		}
		break;
		
		case 2:
		if(flag1 == 0){
			stigao_flag0 = 0;
			flag1 = 1;
			idi_pravo(0,400,0);
			// zadaj_X_Y(-500,0,2);
			sendChar('2');
		}
		else if(stigao_flag0 == 1){
			step1++;
			flag1 = 0;
			sys_time=0;
			
		}
		break;
		
		case 3:
		if(flag1 == 0){
			stigao_flag0 = 0;
			flag1 = 1;
			idi_pravo(0,0,0);
			// zadaj_X_Y(-500,0,2);
			sendChar('3');
		}
		else if(stigao_flag0 == 1){
			step1++;
			flag1 = 0;
			sys_time=0;
			
		}
		break;
		
		default:
		break;
	}
}