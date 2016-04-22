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

volatile char step1 = 0;
volatile char flag1 = 0;






void nuliraj_poziciju_robota(void)
{
	X_pos = 0;
	Y_pos = 0;
	teta = 0;
	X_cilj = 0;
	Y_cilj = 0;
	teta_cilj = teta_cilj_final = 0;
	smer_zadati = smer_trenutni = 2; //Idi pravo
	TCD0.CNT = 0;			//Desni enkoder
	TCD1.CNT = 0;			//Levi enkoder
	count_L = 0;
	count_R = 0;
	last_count_R = 0;
	last_count_L = 0;
}

void postavi_sistem(signed long x, signed long y, signed long ugao)
{
	X_pos = x * scale_factor_for_mm;
	X_cilj = X_pos;
	X_cilj_stari = X_pos;
	
	//Y_pos
	//Y_pos = 0;
	Y_pos = y * scale_factor_for_mm;
	Y_cilj = Y_pos;
	Y_cilj_stari = Y_pos;
	
	
 	teta = (ugao * krug360) / 360;
 	teta_cilj = teta;
 	teta_cilj_final = 0xFFFFFFFF;
 	teta_greska=0;

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

void idi_pravo(signed long x, signed long y, signed long ugao)
{
	//modifikovana_zeljena_pravolinijska_brzina=zeljena_pravolinijska_brzina;
	X_cilj = x * scale_factor_for_mm;
	Y_cilj = y * scale_factor_for_mm;
	
	teta_cilj_final = (ugao * krug360) / 360;
	smer_zadati = 1;
}

void idi_pravo2(signed long x, signed long y)
{
	//modifikovana_zeljena_pravolinijska_brzina=zeljena_pravolinijska_brzina;
	X_cilj = x * scale_factor_for_mm;
	Y_cilj = y * scale_factor_for_mm;
	ugao_timer=0;
	pravo2_flag=1;
	smer_zadati = 2;
}

void idi_unazad(signed long x, signed long y, signed long ugao)
{
	//zadaj_X_Y_teta(0,0,0,1);
	
	//modifikovana_zeljena_pravolinijska_brzina=zeljena_pravolinijska_brzina;
	X_cilj = x * scale_factor_for_mm;
	Y_cilj = y * scale_factor_for_mm;
	
	teta_cilj_final = (ugao * krug360) / 360;
	smer_zadati = 2;
}

void zaustavi_se_u_mestu(void)
{
	modifikovana_zeljena_pravolinijska_brzina=0;
	X_cilj=X_pos;
	Y_cilj=Y_pos;
	teta_cilj_final=teta;
	//set_direct_out=1;
	
	//zeljena_brzina_okretanja=0;
	//stigao_flag=0;
		
}

void SendChar_USB(char c)
{
	USARTC0.DATA = c;
	while(!(USARTC0.STATUS & (1 << 5)));
}

void sendMsg(char *poruka)
{
	while(*poruka != '\0'){
		SendChar(*poruka);
		poruka++;
	}
}

//E0 je komunikacija sa logika plocicom
void SendChar(char c)
{
	USARTE0.DATA = c;
	while(!(USARTE0.STATUS & (1 << 5)));
}

void inicijalizuj_servo_tajmer_20ms()
{
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

void pomeri_servo_2(uint16_t deg)
{
	uint16_t res = (uint16_t)(deg*(250/180));	//250 cycles for 180 degree turn
	if(res <= 0)
	res = 125;								//125 cycles for 0 degree turn
	else if(res > 250)
	res = 250;
	TCF0.CCB = res;
}

void pomeri_servo_3(uint16_t deg)
{
	uint16_t res = (uint16_t)(deg*(250/180));	//250 cycles for 180 degree turn
	if(res <= 0)
	res = 125;								//125 cycles for 0 degree turn
	else if(res > 250)
	res = 250;
	TCF0.CCC = res;
}

void pomeri_servo_4(uint16_t deg)
{
	uint16_t res = (uint16_t)(deg*(250/180));	//250 cycles for 180 degree turn
	if(res <= 0)
	res = 125;								//125 cycles for 0 degree turn
	else if(res > 250)
	res = 250;
	TCF0.CCD = res;
}





 void demo_1(void)
 {
	switch(step1)
	{
		case 0:
			if(flag1 == 0)
			{
			
				stigao_flag = 0;
				flag1 = 1;
				//zadaj_teta(45,0);
				idi_pravo(700,0,180);
			
				//sendChar('0');
			}
			else if(stigao_flag == 1)
			{
				step1++;
				flag1 = 0;
				sys_time=0;
			}
			break;
		
 		case 1:
 			
 				if(flag1 == 0)
 				{
 					stigao_flag = 0;
 					flag1 = 1;
 					//idi_pravo(500,0,0);
 					idi_pravo(700,300,0);
 			
 					//sendChar('0');
 				}
 				else if(stigao_flag == 1)
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
			//SendChar_USB('C');
			//SendChar_USB(flag1);
			if(flag1 == 0)
			{
				SendChar_USB('C');
				stigao_flag = 0;
				flag1 = 1;
				idi_pravo(1000,0,0);
				SendChar_USB(8);
			}
			else if(stigao_flag)
			{
				SendChar_USB('S');
				step1++;
				flag1 = 0;
				sys_time=0;
			}
		break;
		
// 		case 1:
// 		SendChar_USB('C');
// 		SendChar_USB('3');
// 			if(flag1 == 0){
// 				stigao_flag = 0;
// 				flag1 = 1;
// 				idi_unazad(0,0,0);
// 				SendChar_USB('9');
// 				// zadaj_X_Y(-500,0,2);
// 				//sendChar('1');
// 			}
// 			else if(stigao_flag == 1){
// 				SendChar_USB('T');
// 				step1++;
// 				flag1 = 0;
// 				sys_time=0;
// 			}
// 		break;
		
		//case 2:
		//if(sys_time>1500){
			//if(flag1 == 0){
				//stigao_flag = 0;
				//flag1 = 1;
				//idi_unazad(900,0,0);
				//// zadaj_X_Y(-500,0,2);
				////sendChar('1');
			//}
			//else if(stigao_flag == 1){
				//step1++;
				//flag1 = 0;
				//sys_time=0;
			//}
		//}
		//break;
		//
		//case 3:
		//if(sys_time>1500){
			//if(flag1 == 0){
				//stigao_flag = 0;
				//flag1 = 1;
				//idi_unazad(0,0,0);
				//// zadaj_X_Y(-500,0,2);
				////sendChar('1');
			//}
			//else if(stigao_flag == 1){
				//step1++;
				//flag1 = 0;
				//sys_time=0;
			//}
		//}
		//break;
		
		default:
			break;
	}
}

void kocka_poy(void){
	switch(step1)
	{
		case 0:
		if(flag1 == 0){
			stigao_flag = 0;
			flag1 = 1;
			idi_pravo(400,0,0);
			stigao_flag_sigurnosni = 1;
			// zadaj_X_Y(-500,0,2);
			//sendChar('0');
		}
		else if(stigao_flag == 1){
			step1++;
			flag1 = 0;
			sys_time=0;
			stigao_flag=0;
			stigao_flag_sigurnosni = 0;
		}
		break;
		
		case 1:
		if(flag1 == 0){
			stigao_flag = 0;
			flag1 = 1;
			idi_pravo(400,400,0);
			stigao_flag_sigurnosni = 1;
			// zadaj_X_Y(-500,0,2);
			//sendChar('1');
		}
		else if(stigao_flag == 1){
			step1++;
			flag1 = 0;
			sys_time=0;
			stigao_flag=0;
			stigao_flag_sigurnosni = 0;
			
		}
		break;
		
		case 2:
		if(flag1 == 0){
			stigao_flag = 0;
			flag1 = 1;
			idi_pravo(0,400,0);
			stigao_flag_sigurnosni = 1;
		}
		else if(stigao_flag == 1){
			step1++;
			flag1 = 0;
			sys_time=0;
			stigao_flag=0;
			stigao_flag_sigurnosni = 0;
		}
		break;
		
		case 3:
		if(flag1 == 0){
			stigao_flag = 0;
			flag1 = 1;
			idi_unazad(0,0,0);
			stigao_flag_sigurnosni = 1;
			// zadaj_X_Y(-500,0,2);
			//sendChar('3');
		}
		else if(stigao_flag == 1){
			step1++;
			flag1 = 0;
			sys_time=0;
			stigao_flag=0;
			stigao_flag_sigurnosni = 0;
			
		}
		break;
		
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
			stigao_flag = 0;
			flag1 = 1;
			idi_pravo(400,0,0);
			stigao_flag_sigurnosni = 1;
			// zadaj_X_Y(-500,0,2);
			//sendChar('0');
		}
		else if(stigao_flag == 1){
			step1++;
			flag1 = 0;
			sys_time=0;
			stigao_flag=0;
			stigao_flag_sigurnosni = 0;
		}
		break;
		
		case 1:
		if(flag1 == 0){
			stigao_flag = 0;
			flag1 = 1;
			idi_pravo(400,-400,0);
			stigao_flag_sigurnosni = 1;
			// zadaj_X_Y(-500,0,2);
			//sendChar('1');
		}
		else if(stigao_flag == 1){
			step1++;
			flag1 = 0;
			sys_time=0;
			stigao_flag=0;
			stigao_flag_sigurnosni = 0;
			
		}
		break;
		
		case 2:
		if(flag1 == 0){
			stigao_flag = 0;
			flag1 = 1;
			idi_pravo(0,-400,0);
			stigao_flag_sigurnosni = 1;
		}
		else if(stigao_flag == 1){
			step1++;
			flag1 = 0;
			sys_time=0;
			stigao_flag=0;
			stigao_flag_sigurnosni = 0;
		}
		break;
		
		case 3:
		if(flag1 == 0){
			stigao_flag = 0;
			flag1 = 1;
			idi_pravo(0,0,0);
			stigao_flag_sigurnosni = 1;
			// zadaj_X_Y(-500,0,2);
			//sendChar('3');
		}
		else if(stigao_flag == 1){
			step1++;
			flag1 = 0;
			sys_time=0;
			stigao_flag=0;
			stigao_flag_sigurnosni = 0;
			
		}
		break;
		
		default:
		break;
	}
}