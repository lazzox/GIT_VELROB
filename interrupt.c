/*
 * interrupt.c
 *
 * Created: 4/19/2011 2:15:28 PM
 *  Author: robert kovacs
 */ 
#include <avr/io.h>
#include "Headers/avr_compiler.h"
#include "Headers/adc_driver.h"
#include "Headers/usart_driver.h"
#include "Headers/TC_driver.h"
#include "Headers/globals.h"
#include "Headers/mechanism.h"
#include "Headers/port_driver.h"
#include "math.h"

//Tajmer za rad drajvera
ISR(TCE1_OVF_vect)	//3ms
{	
	vreme_cekanja_tete++;
	vreme_pozicioniranja++;
	sys_time++;
	
	sample_counter_niz_1++;
	sample_counter_niz_2++;
	sample_counter_niz_3++;
	
	//Sample time za PID brzinski
	PID_brzinski();
	//Sample time za uzimanje trenutnog stanja enkodera i racunaje pozicije
	Rac_tren_poz_sample_counter++;
	//Sample time za pracenje pravca
	Pracenje_Pravca_sample_counter++;
	//Sample time za pozicioni PID
	PID_pozicioni_sample_counter++;
	
	if(timeout)
	{
		timeout--;
	}
	
	//provera vreme primanja
	if (proveri_vreme_primanja > 0)
	{
		proveri_vreme_primanja++;
		if (proveri_vreme_primanja > 100)
		{
			RX_i_E0 = 0;
			RX_i_E1 = 0;
			RX_i_C0 = 0;
			proveri_vreme_primanja = 0;
		}
	}
	
	//snimanje niz_1
	if ((sample_counter_niz_1 > sample_time_niz_1) && (niz_counter_niz_1 < 127))
	{
		if (velicina_niz_1 == 2)
		{
			niz_1[niz_counter_niz_1] = mmio32(adresa_niz_1 + 1);
			niz_1[niz_counter_niz_1] = mmio32(adresa_niz_1);
		}
		else
		niz_1[niz_counter_niz_1] = mmio32(adresa_niz_1);
		
		niz_counter_niz_1++;
		sample_counter_niz_1 = 0;
	}
}
//Serijska komunikacija - USART_E0 - BT_RS232 - MCU
ISR(USARTE0_RXC_vect)
{
	int i;
	USART_RXComplete(&USART_E0_data);
	receiveArray[RX_i_E0] = USART_RXBuffer_GetByte(&USART_E0_data);
	//USART_TXBuffer_PutByte(&USART_E0_data, receiveArray[RX_i_E0]);	//echo
	RX_i_E0++;
	
	//vremenska zastita
	if (RX_i_E0 >= 1)
 		proveri_vreme_primanja = 1;
	 
	//CITANJE PARAMETARA - 1
	if(receiveArray[0] == 1)						//provera funkcijskog bajta >> 1-citanje osnovnih parametara
	{
		if(RX_i_E0 == 1)								//stigla je cela poruka (2)
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			
			//if(receiveArray[1] == 1)				//CHC ok
			{
				//Slanje		
				sendArray[0] = receiveArray[0];										//vracamo funkcijski broj
				sendArray[1] = (X_pos / scale_factor_for_mm) >> 8;					//Absolutna X pozicija HI
				sendArray[2] = X_pos / scale_factor_for_mm;							//Absolutna X pozicija LO
				sendArray[3] = (Y_pos / scale_factor_for_mm) >> 8;					//Absolutna Y pozicija HI
				sendArray[4] = Y_pos / scale_factor_for_mm;							//Absolutna Y pozicija LO
				sendArray[5] = ((teta * 360) / krug360) >> 8;						//Teta HI
				sendArray[6] = ((teta * 360) / krug360);							//Teta LO	
// 				sendArray[7] = (rastojanje_cilj_temp / scale_factor_for_mm) >> 8;	//Rastojanje od zadate tacke HI
// 				sendArray[8] = (rastojanje_cilj_temp / scale_factor_for_mm);		//Rastojanje od zadate tacke LO
// 				sendArray[9] = stigao_flag;											//stigao flag
// 				sendArray[10] = sample_L16;											//trenutna brzina leva
// 				sendArray[11] = sample_R16;											//trenutna brzina desna
// 				sendArray[12] =	ADC_ResultCh_GetWord(&ADCA.CH0, offset);			//struja motora 1
// 				sendArray[13] =	ADC_ResultCh_GetWord(&ADCA.CH1, offset);			//struja motora 2
// 				sendArray[14] =	PORTB.IN;											//digitalni ulazi
				
				//sendArray[15] = 0;													//nuliramo CHC
				i = 0;
				while (i <= 14) 
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_E0_data, sendArray[i]);
					//sendArray[15] ^= sendArray[i];	//CHC
					if(byteToBuffer)
						i++;
				}
				RX_i_E0 = 0;
			}
			RX_i_E0 = 0;		//ako ne valja CHC ponistava se komanda				
		}		
	}
	//ZADAVANJE X,Y KOORDINATA - 2
	else if(receiveArray[0] == 2)					//provera funkcijskog bajta >> 2-upis x,y koordinate
	{
		if(RX_i_E0 >= 5)							//stigla je cela poruka	(5 bajtova)
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			//ENABLE
			stop_PID_desni = 0;
			stop_PID_levi = 0;
			set_direct_out = 0;
			
			X_cilj = 0;
			X_cilj |= ((int)receiveArray[1]) << 8;
			X_cilj |= (int)receiveArray[2];
			X_cilj = (X_cilj * scale_factor_for_mm);
			//Y_cilj
			Y_cilj = 0;
			Y_cilj |= ((int)receiveArray[3]) << 8;
			Y_cilj |= (int)receiveArray[4];
			Y_cilj = Y_cilj * scale_factor_for_mm;
			
			//slanje odgovora
			sendArray[0] = receiveArray[0];	
			i = 0;
			while (i < 1)
			{
				bool byteToBuffer;
				byteToBuffer = USART_TXBuffer_PutByte(&USART_E0_data, sendArray[i]);
				if(byteToBuffer)
				{
					i++;
				}
			}
			RX_i_E0 = 0;
		}
	}	
	//ZADAVANJE X,Y KOORDINATA I PARAMETRE KRETANJA - 3
	else if(receiveArray[0] == 3)					//provera funkcijskog bajta >> 3 - X,Y koordinate sa svim parametrima kretanja
	{
		if(RX_i_E0 >= 11)							//stigla je cela poruka	(11)
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
// 			i = 0;
// 			CHC = 0;
// 			for(i=0; i<= 4; i++)					//racunanje CHC
// 				CHC ^= receiveArray[i];

			//if(receiveArray[x] == CHC)				//CHC ok
			{
				//x_cilj
				if(!(receiveArray[1] == 0xFF && receiveArray[2] == 0xFF))	//FF se koristi ako ne zelimo da menjamo parametar
				{
					//ENABLE
					stop_PID_desni = 0;
					stop_PID_levi = 0;
					set_direct_out = 0;
					
					X_cilj = 0;
					X_cilj |= ((int)receiveArray[1]) << 8;
					X_cilj |= (int)receiveArray[2];
					X_cilj = (X_cilj * scale_factor_for_mm);	
				}	
				//Y_cilj
				if(!(receiveArray[3] == 0xFF && receiveArray[4] == 0xFF))	//FF se koristi ako ne zelimo da menjamo parametar
				{		
					stop_PID_desni = 0;
					stop_PID_levi = 0;	
					set_direct_out = 0;
					Y_cilj = 0;
					Y_cilj |= ((int)receiveArray[3]) << 8;
					Y_cilj |= (int)receiveArray[4];
					Y_cilj = Y_cilj * scale_factor_for_mm;
				}	
				
				//teta_cilj_final_absolute
				if(!(receiveArray[5] == 0xFF && receiveArray[6] == 0xFF))	//FF se koristi ako ne zelimo da menjamo parametar
				{
					teta_cilj_final = 0;	//i ovde ako se zadaje 0xFF nece se uvazavati ovaj parametar
					teta_cilj_final |= ((int)receiveArray[5]) << 8;
					teta_cilj_final |= (int)receiveArray[6];
					teta_cilj_final = (teta_cilj_final * krug360) / 360;
				}	
				//teta_cilj_final_relative
				else if(!(receiveArray[7] == 0xFF && receiveArray[8] == 0xFF))	//FF se koristi ako ne zelimo da menjamo parametar
				{
					teta_cilj_final = 0;
					teta_cilj_final |= ((int)receiveArray[7]) << 8;
					teta_cilj_final |= (int)receiveArray[8];
					teta_cilj_final = teta + (teta_cilj_final * krug360) / 360;
				}
				else
					teta_cilj_final = 0xFFFFFFFF;				
				//bzina
				if(receiveArray[9] != 0xFF)	// ako zadajemo 0xFF ne menja se brzina
				{
					zeljena_pravolinijska_brzina = receiveArray[9] * 3;	//podesiti faktor!
					zeljena_brzina_okretanja = zeljena_pravolinijska_brzina / 2;
				}				
				//smer
				if(receiveArray[10] != 0xFF)	// ako zadajemo 0xFF ne menja se smer
					smer_zadati = receiveArray[10];	
				
				//pokretanje snimanja u nizove
				sample_counter_niz_1 = 0;
				niz_counter_niz_1 = 0;	
				sample_counter_niz_2 = 0;
				niz_counter_niz_2 = 0;
				sample_counter_niz_3 = 0;
				niz_counter_niz_3 = 0;		
	
				//slanje odgovora
				i = 0;
				sendArray[0] = receiveArray[0];	
				while (i < 1)
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_E0_data, sendArray[i]);
					//sendArray[1] ^= sendArray[i];	//CHC
					if(byteToBuffer)
					{
						i++;
					}
				}
				RX_i_E0 = 0;
			}
			RX_i_E0 = 0;		//ako ne valja CHC ponistava se komanda	
		}					
	}
	//RELATIVNA DISTANCA I UGAO - 4
	else if(receiveArray[0] == 4)					//provera funkcijskog bajta
	{
		if(RX_i_E0 >= 5)								//stigla je cela poruka
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			{
				//ENABLE
				stop_PID_desni = 0;
				stop_PID_levi = 0;
				set_direct_out = 0;
				
				rel_distanca = 0;
				rel_distanca |= ((int)receiveArray[1]) << 8;
				rel_distanca |= (int)receiveArray[2];
				rel_distanca = (rel_distanca * scale_factor_for_mm);

				rel_ugao = 0;
				rel_ugao |= ((int)receiveArray[3]) << 8;
				rel_ugao |= (int)receiveArray[4];
				rel_ugao = (rel_ugao * krug360) / 360;
		
				//racunanje koordinate
				double X_pos_cos, Y_pos_sin;
				X_pos_cos = (double)(teta + rel_ugao) / krug180;
				Y_pos_sin = (double)(teta + rel_ugao) / krug180;
				X_pos_cos = cos(X_pos_cos * M_PI);
				Y_pos_sin = sin(Y_pos_sin * M_PI);
				X_pos_cos = rel_distanca * X_pos_cos;
				Y_pos_sin = rel_distanca * Y_pos_sin;
		
				X_cilj = X_pos + (signed long)(X_pos_cos);
				Y_cilj = Y_pos + (signed long)(Y_pos_sin);
				
				//slanje odgovora
				i = 0;
				sendArray[0] = receiveArray[0];
				while (i < 1)
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_E0_data, sendArray[i]);
					//sendArray[1] ^= sendArray[i];	//CHC
					if(byteToBuffer)
					{
						i++;
					}
				}
				RX_i_E0 = 0;
			}
			RX_i_E0 = 0;		//ako ne valja CHC ponistava se komanda
		}
	}
	//SET DIRECT OUT - 5
	else if(receiveArray[0] == 5)					//provera funkcijskog bajta
	{
		if(RX_i_E0 >= 3)							//stigla je cela poruka
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			{
				//ENABLE
				set_direct_out = 1;
				
				if(receiveArray[1] >= 128)
					PID_brzina_L = (receiveArray[1] - 128) * 5;	//podesiti faktor!
				else
					PID_brzina_L = (128 - receiveArray[1]) * (-5);
					
				if(receiveArray[2] >= 128)
					PID_brzina_R = (receiveArray[2] - 128) * 5;	//podesiti faktor!
				else
					PID_brzina_R = (128 - receiveArray[2]) * (-5);
				
				//slanje odgovora
				i = 0;
				sendArray[0] = receiveArray[0];
				while (i < 1)
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_E0_data, sendArray[i]);
					//sendArray[1] ^= sendArray[i];	//CHC
					if(byteToBuffer)
					{
						i++;
					}
				}
				RX_i_E0 = 0;
			}
			RX_i_E0 = 0;		//ako ne valja CHC ponistava se komanda
		}
	}			
	//PODESAVANJE FET izlaza i servoa - 6
	else if(receiveArray[0] == 6)					//provera funkcijskog bajta 
	{
		if(RX_i_E0 >= 7)							//stigla je cela poruka	(11)
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			{
				PORTC.OUT |= receiveArray[1] & receiveArray[2];	//izlazi + maska
				PORTC.OUT &= ~(receiveArray[1] ^ receiveArray[2]);	//izlazi + maska
				
				//120 - nulti polozaj, 280 - krajnji polozaj
				TCF0.CCA = receiveArray[3] + 120;
				TCF0.CCB = receiveArray[4] + 120;
				TCF0.CCC = receiveArray[5] + 120;
				TCF0.CCD = receiveArray[6] + 120;
			
				//slanje odgovora
				i = 0;
				sendArray[0] = receiveArray[0];
				while (i < 1)
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_E0_data, sendArray[i]);
					//sendArray[1] ^= sendArray[i];	//CHC
					if(byteToBuffer)
					{
						i++;
					}
				}
				RX_i_E0 = 0;
			}
			RX_i_E0 = 0;	//ako ne valja CHC ponistava se komanda
		}		
	}		
	//Upis trenutne pozicije - 7
	else if(receiveArray[0] == 7)					//provera funkcijskog bajta
	{
		if(RX_i_E0 >= 7)							//stigla je cela poruka	(11)
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			{
				//x_pos
				if(!(receiveArray[1] == 0xFF && receiveArray[2] == 0xFF))	//FF se koristi ako ne zelimo da menjamo parametar
				{
					X_pos = 0;
					X_pos |= ((int)receiveArray[1]) << 8;
					X_pos |= (int)receiveArray[2];
					X_pos = (X_pos * scale_factor_for_mm);
					X_cilj = X_pos;
					X_cilj_stari = X_pos;
				}
				//Y_pos
				if(!(receiveArray[3] == 0xFF && receiveArray[4] == 0xFF))	//FF se koristi ako ne zelimo da menjamo parametar
				{
					Y_pos = 0;
					Y_pos |= ((int)receiveArray[3]) << 8;
					Y_pos |= (int)receiveArray[4];
					Y_pos = Y_pos * scale_factor_for_mm;
					Y_cilj = Y_pos;
					Y_cilj_stari = Y_pos;
				}
				
				//teta
				if(!(receiveArray[5] == 0xFF && receiveArray[6] == 0xFF))	//FF se koristi ako ne zelimo da menjamo parametar
				{
					teta = 0;	//i ovde ako se zadaje 0xFF nece se uvazavati ovaj parametar
					teta |= ((int)receiveArray[5]) << 8;
					teta |= (int)receiveArray[6];
					teta = (teta * krug360) / 360;
					teta_cilj = teta;
				}
				
				//slanje odgovora
				i = 0;
				sendArray[0] = receiveArray[0];
				while (i < 1)
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_E0_data, sendArray[i]);
					//sendArray[1] ^= sendArray[i];	//CHC
					if(byteToBuffer)
					{
						i++;
					}
				}
				RX_i_E0 = 0;
			}
			RX_i_E0 = 0;	//ako ne valja CHC ponistava se komanda
		}
	}
	//Total Stop - 8
	else if(receiveArray[0] == 8)					//provera funkcijskog bajta
	{
		if(RX_i_E0 >= 3)							//stigla je cela poruka
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			{
				
				if(receiveArray[1] == 0x01)
				{
					stop_PID_levi = 1;
					PID_brzina_L = 0;
				}				
				else
					stop_PID_levi = 0;
					
				if(receiveArray[2] == 0x01)
				{
					stop_PID_desni = 1;
					PID_brzina_R = 0;
				}					
				else
					stop_PID_desni = 0;
				
				//slanje odgovora
				i = 0;
				sendArray[0] = receiveArray[0];
				while (i < 1)
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_E0_data, sendArray[i]);
					//sendArray[1] ^= sendArray[i];	//CHC
					if(byteToBuffer)
					{
						i++;
					}
				}
				RX_i_E0 = 0;
			}
			RX_i_E0 = 0;		//ako ne valja CHC ponistava se komanda
		}
	}
	//citanje X koordinate
	else if(receiveArray[0] == 101)						//provera funkcijskog bajta >> 101-citanje X pozicije
	{
		if(RX_i_E0 == 1)								//stigla je cela poruka (2)
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			
			{
				//Slanje		
				sendArray[0] = receiveArray[0];										//vracamo funkcijski broj
				sendArray[1] = (X_pos / scale_factor_for_mm) >> 8;					//Absolutna X pozicija HI
				sendArray[2] = X_pos / scale_factor_for_mm;							//Absolutna X pozicija LO
				
				i = 0;
				while (i < 3) 
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_E0_data, sendArray[i]);
					//sendArray[15] ^= sendArray[i];	//CHC
					if(byteToBuffer)
						i++;
				}
				RX_i_E0 = 0;
			}
			RX_i_E0 = 0;		//ako ne valja CHC ponistava se komanda				
		}		
	}
	//citanje Y koordinate
	else if(receiveArray[0] == 102)						//provera funkcijskog bajta >> 1-citanje osnovnih parametara
	{
		if(RX_i_E0 == 1)								//stigla je cela poruka (2)
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			
			//if(receiveArray[1] == 1)				//CHC ok
			{
				//Slanje		
				sendArray[0] = receiveArray[0];										//vracamo funkcijski broj
				sendArray[1] = (Y_pos / scale_factor_for_mm) >> 8;					//Absolutna X pozicija HI
				sendArray[2] = Y_pos / scale_factor_for_mm;							//Absolutna X pozicija LO
				
				i = 0;
				while (i < 3) 
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_E0_data, sendArray[i]);
					//sendArray[15] ^= sendArray[i];	//CHC
					if(byteToBuffer)
						i++;
				}
				RX_i_E0 = 0;
			}
			RX_i_E0 = 0;		//ako ne valja CHC ponistava se komanda				
		}		
	}
	//citanje teta abs
	else if(receiveArray[0] == 103)						//provera funkcijskog bajta >> 1-citanje osnovnih parametara
	{
		if(RX_i_E0 == 1)								//stigla je cela poruka (2)
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			
			//if(receiveArray[1] == 1)				//CHC ok
			{
				//Slanje		
				sendArray[0] = receiveArray[0];										//vracamo funkcijski broj
				sendArray[1] = ((teta * 360) / krug360) >> 8;						//Teta HI
				sendArray[2] = ((teta * 360) / krug360);							//Teta LOW
				
				i = 0;
				while (i < 3) 
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_E0_data, sendArray[i]);
					//sendArray[15] ^= sendArray[i];	//CHC
					if(byteToBuffer)
						i++;
				}
				RX_i_E0 = 0;
			}
			RX_i_E0 = 0;		//ako ne valja CHC ponistava se komanda				
		}		
	}
}
//Serijska komunikacija USART_E1 - BT - bluetooth
ISR(USARTE1_RXC_vect)
{	
	int i;
	USART_RXComplete(&USART_E1_data);
	receiveArray[RX_i_E1] = USART_RXBuffer_GetByte(&USART_E1_data);
	//USART_TXBuffer_PutByte(&USART_E1_data, receiveArray[RX_i_E1]);	//echo
	RX_i_E1++;
	
	//vremenska zastita
	if (RX_i_E1 >= 1)
 		proveri_vreme_primanja = 1;
	 
	//CITANJE PARAMETARA - 1
	if(receiveArray[0] == 1)						//provera funkcijskog bajta >> 1-citanje osnovnih parametara
	{
		if(RX_i_E1 == 1)								//stigla je cela poruka (2)
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			
			//if(receiveArray[1] == 1)				//CHC ok
			{
				//Slanje		
				sendArray[0] = receiveArray[0];										//vracamo funkcijski broj
				sendArray[1] = (X_pos / scale_factor_for_mm) >> 8;					//Absolutna X pozicija HI
				sendArray[2] = X_pos / scale_factor_for_mm;							//Absolutna X pozicija LO
				sendArray[3] = (Y_pos / scale_factor_for_mm) >> 8;					//Absolutna Y pozicija HI
				sendArray[4] = Y_pos / scale_factor_for_mm;							//Absolutna Y pozicija LO
				sendArray[5] = ((teta * 360) / krug360) >> 8;						//Teta HI
				sendArray[6] = ((teta * 360) / krug360);							//Teta LO	
				sendArray[7] = (rastojanje_cilj_temp / scale_factor_for_mm) >> 8;	//Rastojanje od zadate tacke HI
				sendArray[8] = (rastojanje_cilj_temp / scale_factor_for_mm);		//Rastojanje od zadate tacke LO
				sendArray[9] = stigao_flag;											//stigao flag
				sendArray[10] = sample_L16;											//trenutna brzina leva
				sendArray[11] = sample_R16;											//trenutna brzina desna
				sendArray[12] =	ADC_ResultCh_GetWord(&ADCA.CH0, offset);			//struja motora 1
				sendArray[13] =	ADC_ResultCh_GetWord(&ADCA.CH1, offset);			//struja motora 2
				sendArray[14] =	PORTB.IN;											//digitalni ulazi
				
				//sendArray[15] = 0;													//nuliramo CHC
				i = 0;
				while (i <= 14) 
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_E1_data, sendArray[i]);
					//sendArray[15] ^= sendArray[i];	//CHC
					if(byteToBuffer)
						i++;
				}
				RX_i_E1 = 0;
			}
			RX_i_E1 = 0;		//ako ne valja CHC ponistava se komanda				
		}		
	}
	//ZADAVANJE X,Y KOORDINATA - 2
	else if(receiveArray[0] == 2)					//provera funkcijskog bajta >> 2-upis x,y koordinate
	{
		if(RX_i_E1 >= 5)							//stigla je cela poruka	(5 bajtova)
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			//ENABLE
			stop_PID_desni = 0;
			stop_PID_levi = 0;
			set_direct_out = 0;
			
			X_cilj = 0;
			X_cilj |= (int)receiveArray[1] << 8;
			X_cilj |= (int)receiveArray[2];
			X_cilj = (X_cilj * scale_factor_for_mm);
			//Y_cilj
			Y_cilj = 0;
			Y_cilj |= (int)receiveArray[3] << 8;
			Y_cilj |= (int)receiveArray[4];
			Y_cilj = Y_cilj * scale_factor_for_mm;
			
			//slanje odgovora
			sendArray[0] = receiveArray[0];	
			i = 0;
			while (i < 1)
			{
				bool byteToBuffer;
				byteToBuffer = USART_TXBuffer_PutByte(&USART_E1_data, sendArray[i]);
				if(byteToBuffer)
				{
					i++;
				}
			}
			RX_i_E1 = 0;
		}
	}	
	//ZADAVANJE X,Y KOORDINATA I PARAMETRE KRETANJA - 3
	else if(receiveArray[0] == 3)					//provera funkcijskog bajta >> 3 - X,Y koordinate sa svim parametrima kretanja
	{
		if(RX_i_E1 >= 11)							//stigla je cela poruka	(11)
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			{
				//x_cilj
				if(!(receiveArray[1] == 0xFF && receiveArray[2] == 0xFF))	//FF se koristi ako ne zelimo da menjamo parametar
				{
					//ENABLE
					stop_PID_desni = 0;
					stop_PID_levi = 0;
					set_direct_out = 0;
					
					X_cilj = 0;
					X_cilj |= (int)receiveArray[1] << 8;
					X_cilj |= (int)receiveArray[2];
					X_cilj = (X_cilj * scale_factor_for_mm);	
				}	
				//Y_cilj
				if(!(receiveArray[3] == 0xFF && receiveArray[4] == 0xFF))	//FF se koristi ako ne zelimo da menjamo parametar
				{		
					stop_PID_desni = 0;
					stop_PID_levi = 0;	
					set_direct_out = 0;
					Y_cilj = 0;
					Y_cilj |= (int)receiveArray[3] << 8;
					Y_cilj |= (int)receiveArray[4];
					Y_cilj = Y_cilj * scale_factor_for_mm;
				}	
				
				//teta_cilj_final_absolute
				if(!(receiveArray[5] == 0xFF && receiveArray[6] == 0xFF))	//FF se koristi ako ne zelimo da menjamo parametar
				{
					teta_cilj_final = 0;	//i ovde ako se zadaje 0xFF nece se uvazavati ovaj parametar
					teta_cilj_final |= (int)receiveArray[5] << 8;
					teta_cilj_final |= (int)receiveArray[6];
					teta_cilj_final = (teta_cilj_final * krug360) / 360;
				}	
				//teta_cilj_final_relative
				else if(!(receiveArray[7] == 0xFF && receiveArray[8] == 0xFF))	//FF se koristi ako ne zelimo da menjamo parametar
				{
					teta_cilj_final = 0;
					teta_cilj_final |= (int)receiveArray[7] << 8;
					teta_cilj_final |= (int)receiveArray[8];
					teta_cilj_final = teta + (teta_cilj_final * krug360) / 360;
				}
				else
					teta_cilj_final = 0xFFFFFFFF;				
				//bzina
				if(receiveArray[9] != 0xFF)	// ako zadajemo 0xFF ne menja se brzina
				{
					zeljena_pravolinijska_brzina = receiveArray[9] * 3;	//podesiti faktor!
					zeljena_brzina_okretanja = zeljena_pravolinijska_brzina / 2;
				}				
				//smer
				if(receiveArray[10] != 0xFF)	// ako zadajemo 0xFF ne menja se smer
					smer_zadati = receiveArray[10];	
				
				//pokretanje snimanja u nizove
				sample_counter_niz_1 = 0;
				niz_counter_niz_1 = 0;	
				sample_counter_niz_2 = 0;
				niz_counter_niz_2 = 0;
				sample_counter_niz_3 = 0;
				niz_counter_niz_3 = 0;		
	
				//slanje odgovora
				i = 0;
				sendArray[0] = receiveArray[0];	
				while (i < 1)
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_E1_data, sendArray[i]);
					//sendArray[1] ^= sendArray[i];	//CHC
					if(byteToBuffer)
					{
						i++;
					}
				}
				RX_i_E1 = 0;
			}
			RX_i_E1 = 0;		//ako ne valja CHC ponistava se komanda	
		}					
	}
	//RELATIVNA DISTANCA I UGAO - 4
	else if(receiveArray[0] == 4)					//provera funkcijskog bajta
	{
		if(RX_i_E1 >= 5)								//stigla je cela poruka
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			{
				//ENABLE
				stop_PID_desni = 0;
				stop_PID_levi = 0;
				set_direct_out = 0;
				
				rel_distanca = 0;
				rel_distanca |= (int)receiveArray[1] << 8;
				rel_distanca |= (int)receiveArray[2];
				rel_distanca = (rel_distanca * scale_factor_for_mm);

				rel_ugao = 0;
				rel_ugao |= (int)receiveArray[3] << 8;
				rel_ugao |= (int)receiveArray[4];
				rel_ugao = (rel_ugao * krug360) / 360;
		
				//racunanje koordinate
				double X_pos_cos, Y_pos_sin;
				X_pos_cos = (double)(teta + rel_ugao) / krug180;
				Y_pos_sin = (double)(teta + rel_ugao) / krug180;
				X_pos_cos = cos(X_pos_cos * M_PI);
				Y_pos_sin = sin(Y_pos_sin * M_PI);
				X_pos_cos = rel_distanca * X_pos_cos;
				Y_pos_sin = rel_distanca * Y_pos_sin;
		
				X_cilj = X_pos + (signed long)(X_pos_cos);
				Y_cilj = Y_pos + (signed long)(Y_pos_sin);
				
				//slanje odgovora
				i = 0;
				sendArray[0] = receiveArray[0];
				while (i < 1)
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_E1_data, sendArray[i]);
					//sendArray[1] ^= sendArray[i];	//CHC
					if(byteToBuffer)
					{
						i++;
					}
				}
				RX_i_E1 = 0;
			}
			RX_i_E1 = 0;		//ako ne valja CHC ponistava se komanda
		}
	}
	//SET DIRECT OUT - 5
	else if(receiveArray[0] == 5)					//provera funkcijskog bajta
	{
		if(RX_i_E1 >= 3)							//stigla je cela poruka
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			{
				//ENABLE
				set_direct_out = 1;
				
				if(receiveArray[1] >= 128)
					PID_brzina_L = (receiveArray[1] - 128) * 5;	//podesiti faktor!
				else
					PID_brzina_L = (128 - receiveArray[1]) * (-5);
					
				if(receiveArray[2] >= 128)
					PID_brzina_R = (receiveArray[2] - 128) * 5;	//podesiti faktor!
				else
					PID_brzina_R = (128 - receiveArray[2]) * (-5);
				
				//slanje odgovora
				i = 0;
				sendArray[0] = receiveArray[0];
				while (i < 1)
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_E1_data, sendArray[i]);
					//sendArray[1] ^= sendArray[i];	//CHC
					if(byteToBuffer)
					{
						i++;
					}
				}
				RX_i_E1 = 0;
			}
			RX_i_E1 = 0;		//ako ne valja CHC ponistava se komanda
		}
	}			
	//PODESAVANJE FET izlaza i servoa - 6
	else if(receiveArray[0] == 6)					//provera funkcijskog bajta 
	{
		if(RX_i_E1 >= 7)							//stigla je cela poruka	(11)
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			{
				PORTC.OUT |= receiveArray[1] & receiveArray[2];	//izlazi + maska
				PORTC.OUT &= ~(receiveArray[1] ^ receiveArray[2]);	//izlazi + maska
				
				//120 - nulti polozaj, 280 - krajnji polozaj
				TCF0.CCA = receiveArray[3] + 120;
				TCF0.CCB = receiveArray[4] + 120;
				TCF0.CCC = receiveArray[5] + 120;
				TCF0.CCD = receiveArray[6] + 120;
			
				//slanje odgovora
				i = 0;
				sendArray[0] = receiveArray[0];
				while (i < 1)
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_E1_data, sendArray[i]);
					//sendArray[1] ^= sendArray[i];	//CHC
					if(byteToBuffer)
					{
						i++;
					}
				}
				RX_i_E1 = 0;
			}
			RX_i_E1 = 0;	//ako ne valja CHC ponistava se komanda
		}		
	}		
	//Upis trenutne pozicije - 7
	else if(receiveArray[0] == 7)					//provera funkcijskog bajta
	{
		if(RX_i_E1 >= 7)							//stigla je cela poruka	(11)
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			{
				//x_pos
				if(!(receiveArray[1] == 0xFF && receiveArray[2] == 0xFF))	//FF se koristi ako ne zelimo da menjamo parametar
				{
					X_pos = 0;
					X_pos |= (int)receiveArray[1] << 8;
					X_pos |= (int)receiveArray[2];
					X_pos = (X_pos * scale_factor_for_mm);
					X_cilj = X_pos;
					X_cilj_stari = X_pos;
				}
				//Y_pos
				if(!(receiveArray[3] == 0xFF && receiveArray[4] == 0xFF))	//FF se koristi ako ne zelimo da menjamo parametar
				{
					Y_pos = 0;
					Y_pos |= (int)receiveArray[3] << 8;
					Y_pos |= (int)receiveArray[4];
					Y_pos = Y_cilj * scale_factor_for_mm;
					Y_cilj = Y_pos;
					Y_cilj_stari = Y_pos;
				}
				
				//teta
				if(!(receiveArray[5] == 0xFF && receiveArray[6] == 0xFF))	//FF se koristi ako ne zelimo da menjamo parametar
				{
					teta = 0;	//i ovde ako se zadaje 0xFF nece se uvazavati ovaj parametar
					teta |= (int)receiveArray[5] << 8;
					teta |= (int)receiveArray[6];
					teta = (teta * krug360) / 360;
					teta_cilj = teta;
				}
				
				//slanje odgovora
				i = 0;
				sendArray[0] = receiveArray[0];
				while (i < 1)
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_E1_data, sendArray[i]);
					//sendArray[1] ^= sendArray[i];	//CHC
					if(byteToBuffer)
					{
						i++;
					}
				}
				RX_i_E1 = 0;
			}
			RX_i_E1 = 0;	//ako ne valja CHC ponistava se komanda
		}
	}
	//Total Stop - 8
	else if(receiveArray[0] == 8)					//provera funkcijskog bajta
	{
		if(RX_i_E1 >= 3)							//stigla je cela poruka
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			{
				
				if(receiveArray[1] == 0x01)
				{
					stop_PID_levi = 1;
					PID_brzina_L = 0;
				}				
				else
					stop_PID_levi = 0;
					
				if(receiveArray[2] == 0x01)
				{
					stop_PID_desni = 1;
					PID_brzina_R = 0;
				}					
				else
					stop_PID_desni = 0;
				
				//slanje odgovora
				i = 0;
				sendArray[0] = receiveArray[0];
				while (i < 1)
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_E1_data, sendArray[i]);
					//sendArray[1] ^= sendArray[i];	//CHC
					if(byteToBuffer)
					{
						i++;
					}
				}
				RX_i_E1 = 0;
			}
			RX_i_E1 = 0;		//ako ne valja CHC ponistava se komanda
		}
	}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//CIATNJE ADRESIRANOG PARAMETRA - 9
	else if(receiveArray[0] == 9)					//provera funkcijskog bajta
	{
		if(RX_i_E1 >= 6)								//stigla je cela poruka (7)
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena		
// 			i = 0;
// 			CHC = 0;
// 			for(i=0; i<=4; i++)						//racunanje CHC
// 				CHC ^= receiveArray[i];

			//if(receiveArray[x] == CHC)			//CHC ok
			{
				adresa_long = 0;
				adresa_long |= (long)receiveArray[2] << 24;
				adresa_long |= (long)receiveArray[3] << 16;
				adresa_long |= (long)receiveArray[4] << 8;
				adresa_long |= (long)receiveArray[5];
	
				//unsigned long registerValue = mmio32(adresa_long);		// read
					
				if (receiveArray[1] == 1)	//jednobajtna promenljiva
				{
					sendArray[4] = mmio32(adresa_long);
					sendArray[3] = 0;
					sendArray[2] = 0;
					sendArray[1] = 0;
				}
				else if (receiveArray[1] == 2) //dvobajtna promenljiva
				{
					sendArray[4] = mmio32(adresa_long);
					sendArray[3] = mmio32(adresa_long+1);
					sendArray[2] = 0;
					sendArray[1] = 0;
				}
				else if (receiveArray[1] == 4)	//cetvorobajtna promenljiva
				{
					sendArray[4] = mmio32(adresa_long);
					sendArray[3] = mmio32(adresa_long+1);
					sendArray[2] = mmio32(adresa_long+2);
					sendArray[1] = mmio32(adresa_long+3);
				}
								
				//slanje odgovora
				i = 0;
				sendArray[0] = receiveArray[0];
				while (i <= 4)
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_E1_data, sendArray[i]);
					//sendArray[5] ^= sendArray[i];	//CHC
					if(byteToBuffer)
					{
						i++;
					}
				}
				RX_i_E1 = 0;
			}
			RX_i_E1 = 0;		//ako ne valja CHC ponistava se komanda	
		}					
	}
	//UPIS U ADRESIRANU PROMENLJIVU - 10
	else if(receiveArray[0] == 10)					//provera funkcijskog bajta
	{
		if(RX_i_E1 >= 10)								//stigla je cela poruka (1+1+4+4+1)
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			{
				adresa_long = 0;
				adresa_long |= (long)receiveArray[2] << 24;
				adresa_long |= (long)receiveArray[3] << 16;
				adresa_long |= (long)receiveArray[4] << 8;
				adresa_long |= (long)receiveArray[5];

				// write
				if (receiveArray[1] == 4)	//cetvorobajtna promenljiva
				{
					mmio32(adresa_long) = receiveArray[9];                     
					mmio32(adresa_long+1) = receiveArray[8];					
					mmio32(adresa_long+2) = receiveArray[7];
					mmio32(adresa_long+3) = receiveArray[6];   
					
					sendArray[1] = mmio32(adresa_long+3);
					sendArray[2] = mmio32(adresa_long+2);
					sendArray[3] = mmio32(adresa_long+1);
					sendArray[4] = mmio32(adresa_long);  
				}	
				else if (receiveArray[1] == 2)	//dvobajtna promenljiva
				{
					mmio32(adresa_long) = receiveArray[9];                     
					mmio32(adresa_long+1) = receiveArray[8];					
					
					sendArray[1] = 0;
					sendArray[2] = 0;
					sendArray[3] = mmio32(adresa_long+1);
					sendArray[4] = mmio32(adresa_long);  
				}	
				else if (receiveArray[1] == 1)	//jednobajtna promenljiva
				{
					mmio32(adresa_long) = receiveArray[9];                        
					
					sendArray[1] = 0;
					sendArray[2] = 0;
					sendArray[3] = 0;
					sendArray[4] = mmio32(adresa_long);  
				}						               
									
				//slanje odgovora
				i = 0;
				sendArray[0] = receiveArray[0];
				while (i <= 4)
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_E1_data, sendArray[i]);
					//sendArray[5] ^= sendArray[i];	//CHC
					if(byteToBuffer)
					{
						i++;
					}
				}
				RX_i_E1 = 0;
			}
			RX_i_E1 = 0;		//ako ne valja CHC ponistava se komanda	
		}					
	}
	//PODESAVANJE NIZA ZA SNIMANJE - 11
	else if(receiveArray[0] == 11)					//provera funkcijskog bajta
	{
		if(RX_i_E1 >= 7)							//stigla je cela poruka (3)
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			{
				if (receiveArray[1] == 1)	//niz_1
				{
					sample_time_niz_1 = receiveArray[2];
					velicina_niz_1 = receiveArray[3];
					adresa_niz_1 = 0;
					adresa_niz_1 |= (long)receiveArray[4] << 24;
					adresa_niz_1 |= (long)receiveArray[5] << 16;
					adresa_niz_1 |= (long)receiveArray[6] << 8;
					adresa_niz_1 |= (long)receiveArray[7];  
				}
				if (receiveArray[1] == 2)	//niz_2
				{
					sample_time_niz_2 = receiveArray[2];
					velicina_niz_2 = receiveArray[3];
					adresa_niz_2 = 0;
					adresa_niz_2 |= (long)receiveArray[4] << 24;
					adresa_niz_2 |= (long)receiveArray[5] << 16;
					adresa_niz_2 |= (long)receiveArray[6] << 8;
					adresa_niz_2 |= (long)receiveArray[7];
				}
				
				if (receiveArray[1] == 3)	//niz_3
				{
					sample_time_niz_3 = receiveArray[2];
					velicina_niz_3 = receiveArray[3];
					adresa_niz_3 = 0;
					adresa_niz_3 |= (long)receiveArray[4] << 24;
					adresa_niz_3 |= (long)receiveArray[5] << 16;
					adresa_niz_3 |= (long)receiveArray[6] << 8;
					adresa_niz_3 |= (long)receiveArray[7];
				}
				
				//slanje odgovora
				i = 0;
				sendArray[0] = receiveArray[0];
				while (i < 1)
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_E1_data, sendArray[i]);
					//sendArray[1] ^= sendArray[i];	//CHC
					if(byteToBuffer)
					{
						i++;
					}
				}			
				RX_i_E1 = 0;
			}
			RX_i_E1 = 0;		//ako ne valja CHC ponistava se komanda	
		}					
	}
	//CITANJE SNIMLJENOG NIZA - 12
	else if(receiveArray[0] == 12)					//provera funkcijskog bajta
	{
		if(RX_i_E1 >= 2)							//stigla je cela poruka (3)
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			{
				if (receiveArray[1] == 1)			//niz_1
				{
					uint16_t k = 0, j = 0;
					for(k=0;k<127;k++)	//imamo 500 merenja
					{
						sendArray[j] = niz_1[k]>>8;
						j++;
						sendArray[j] = niz_1[k];
						j++;
					}
				}
				else if (receiveArray[1] == 2)			//niz_2
				{
					uint16_t k = 0, j = 0;
					for(k=0;k<127;k++)	//imamo 500 merenja
					{
						sendArray[j] = niz_2[k]>>8;
						j++;
						sendArray[j] = niz_2[k];
						j++;
					}
				}
				else if (receiveArray[1] == 3)			//niz_3
				{
					uint16_t k = 0, j = 0;
					for(k=0;k<127;k++)	//imamo 500 merenja
					{
						sendArray[j] = niz_3[k]>>8;
						j++;
						sendArray[j] = niz_3[k];
						j++;
					}
				}
				
				//slanje odgovora
				i = 0;
				while (i < 254)
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_E1_data, sendArray[i]);
					if(byteToBuffer)
					{
						i++;
					}
				}
				RX_i_E1 = 0;
			}
			RX_i_E1 = 0;	//ako ne valja CHC ponistava se komanda
		}
	}	
}
//Serijska komunikacija USART_C0 - BT - XmegaUSB		
ISR(USARTC0_RXC_vect)
{
	int i;
	USART_RXComplete(&USART_C0_data);
	receiveArray[RX_i_C0] = USART_RXBuffer_GetByte(&USART_C0_data);
	//USART_TXBuffer_PutByte(&USART_C0_data, receiveArray[RX_i_C0]);	//echo
	RX_i_C0++;
	
	//vremenska zastita
	if (RX_i_C0 >= 1)
 		proveri_vreme_primanja = 1;
	 
	//CITANJE PARAMETARA - 1
	if(receiveArray[0] == 1)						//provera funkcijskog bajta >> 1-citanje osnovnih parametara
	{
		if(RX_i_C0 == 1)								//stigla je cela poruka (2)
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			
			//if(receiveArray[1] == 1)				//CHC ok
			{
				//Slanje		
				sendArray[0] = receiveArray[0];										//vracamo funkcijski broj
				sendArray[1] = (X_pos / scale_factor_for_mm) >> 8;					//Absolutna X pozicija HI
				sendArray[2] = X_pos / scale_factor_for_mm;							//Absolutna X pozicija LO
				sendArray[3] = (Y_pos / scale_factor_for_mm) >> 8;					//Absolutna Y pozicija HI
				sendArray[4] = Y_pos / scale_factor_for_mm;							//Absolutna Y pozicija LO
				sendArray[5] = ((teta * 360) / krug360) >> 8;						//Teta HI
				sendArray[6] = ((teta * 360) / krug360);							//Teta LO	
				sendArray[7] = (rastojanje_cilj_temp / scale_factor_for_mm) >> 8;	//Rastojanje od zadate tacke HI
				sendArray[8] = (rastojanje_cilj_temp / scale_factor_for_mm);		//Rastojanje od zadate tacke LO
				sendArray[9] = stigao_flag;											//stigao flag
				sendArray[10] = sample_L16;											//trenutna brzina leva
				sendArray[11] = sample_R16;											//trenutna brzina desna
				sendArray[12] =	ADC_ResultCh_GetWord(&ADCA.CH0, offset);			//struja motora 1
				sendArray[13] =	ADC_ResultCh_GetWord(&ADCA.CH1, offset);			//struja motora 2
				sendArray[14] =	PORTB.IN;											//digitalni ulazi
				
				//sendArray[15] = 0;													//nuliramo CHC
				i = 0;
				while (i <= 14) 
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_C0_data, sendArray[i]);
					//sendArray[15] ^= sendArray[i];	//CHC
					if(byteToBuffer)
						i++;
				}
				RX_i_C0 = 0;
			}
			RX_i_C0 = 0;		//ako ne valja CHC ponistava se komanda				
		}		
	}
	//ZADAVANJE X,Y KOORDINATA - 2
	else if(receiveArray[0] == 2)					//provera funkcijskog bajta >> 2-upis x,y koordinate
	{
		if(RX_i_C0 >= 5)							//stigla je cela poruka	(5 bajtova)
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			//ENABLE
			stop_PID_desni = 0;
			stop_PID_levi = 0;
			set_direct_out = 0;
			
			X_cilj = 0;
			X_cilj |= ((int)receiveArray[1]) << 8;
			X_cilj |= (int)receiveArray[2];
			X_cilj = (X_cilj * scale_factor_for_mm);
			//Y_cilj
			Y_cilj = 0;
			Y_cilj |= ((int)receiveArray[3]) << 8;
			Y_cilj |= (int)receiveArray[4];
			Y_cilj = Y_cilj * scale_factor_for_mm;
			
			//slanje odgovora
			sendArray[0] = receiveArray[0];	
			i = 0;
			while (i < 1)
			{
				bool byteToBuffer;
				byteToBuffer = USART_TXBuffer_PutByte(&USART_C0_data, sendArray[i]);
				if(byteToBuffer)
				{
					i++;
				}
			}
			RX_i_C0 = 0;
		}
	}	
	//ZADAVANJE X,Y KOORDINATA I PARAMETRE KRETANJA - 3
	else if(receiveArray[0] == 3)					//provera funkcijskog bajta >> 3 - X,Y koordinate sa svim parametrima kretanja
	{
		if(RX_i_C0 >= 11)							//stigla je cela poruka	(11)
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			{
				//x_cilj
				if(!(receiveArray[1] == 0xFF && receiveArray[2] == 0xFF))	//FF se koristi ako ne zelimo da menjamo parametar
				{
					//ENABLE
					stop_PID_desni = 0;
					stop_PID_levi = 0;
					set_direct_out = 0;
					
					X_cilj = 0;
					X_cilj |= ((int)receiveArray[1]) << 8;
					X_cilj |= (int)receiveArray[2];
					X_cilj = (X_cilj * scale_factor_for_mm);	
				}	
				//Y_cilj
				if(!(receiveArray[3] == 0xFF && receiveArray[4] == 0xFF))	//FF se koristi ako ne zelimo da menjamo parametar
				{		
					stop_PID_desni = 0;
					stop_PID_levi = 0;	
					set_direct_out = 0;
					Y_cilj = 0;
					Y_cilj |= ((int)receiveArray[3]) << 8;
					Y_cilj |= (int)receiveArray[4];
					Y_cilj = Y_cilj * scale_factor_for_mm;
				}	
				
				//teta_cilj_final_absolute
				if(!(receiveArray[5] == 0xFF && receiveArray[6] == 0xFF))	//FF se koristi ako ne zelimo da menjamo parametar
				{
					teta_cilj_final = 0;	//i ovde ako se zadaje 0xFF nece se uvazavati ovaj parametar
					teta_cilj_final |= ((int)receiveArray[5]) << 8;
					teta_cilj_final |= (int)receiveArray[6];
					teta_cilj_final = (teta_cilj_final * krug360) / 360;
				}	
				//teta_cilj_final_relative
				else if(!(receiveArray[7] == 0xFF && receiveArray[8] == 0xFF))	//FF se koristi ako ne zelimo da menjamo parametar
				{
					teta_cilj_final = 0;
					teta_cilj_final |= ((int)receiveArray[7]) << 8;
					teta_cilj_final |= (int)receiveArray[8];
					teta_cilj_final = teta + (teta_cilj_final * krug360) / 360;
				}
				else
					teta_cilj_final = 0xFFFFFFFF;				
				//bzina
				if(receiveArray[9] != 0xFF)	// ako zadajemo 0xFF ne menja se brzina
				{
					zeljena_pravolinijska_brzina = receiveArray[9] * 3;	//podesiti faktor!
					zeljena_brzina_okretanja = zeljena_pravolinijska_brzina / 2;
				}				
				//smer
				if(receiveArray[10] != 0xFF)	// ako zadajemo 0xFF ne menja se smer
					smer_zadati = receiveArray[10];	
				
				//pokretanje snimanja u nizove
				sample_counter_niz_1 = 0;
				niz_counter_niz_1 = 0;	
				sample_counter_niz_2 = 0;
				niz_counter_niz_2 = 0;
				sample_counter_niz_3 = 0;
				niz_counter_niz_3 = 0;		
	
				//slanje odgovora
				i = 0;
				sendArray[0] = receiveArray[0];	
				while (i < 1)
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_C0_data, sendArray[i]);
					//sendArray[1] ^= sendArray[i];	//CHC
					if(byteToBuffer)
					{
						i++;
					}
				}
				RX_i_C0 = 0;
			}
			RX_i_C0 = 0;		//ako ne valja CHC ponistava se komanda	
		}					
	}
	//RELATIVNA DISTANCA I UGAO - 4
	else if(receiveArray[0] == 4)					//provera funkcijskog bajta
	{
		if(RX_i_C0 >= 5)								//stigla je cela poruka
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			{
				//ENABLE
				stop_PID_desni = 0;
				stop_PID_levi = 0;
				set_direct_out = 0;
				
				rel_distanca = 0;
				rel_distanca |= ((int)receiveArray[1]) << 8;
				rel_distanca |= (int)receiveArray[2];
				rel_distanca = (rel_distanca * scale_factor_for_mm);

				rel_ugao = 0;
				rel_ugao |= ((int)receiveArray[3]) << 8;
				rel_ugao |= (int)receiveArray[4];
				rel_ugao = (rel_ugao * krug360) / 360;
		
				//racunanje koordinate
				double X_pos_cos, Y_pos_sin;
				//X_pos_cos = (double)(teta + rel_ugao) / krug180;
				//Y_pos_sin = (double)(teta + rel_ugao) / krug180;
				X_pos_cos = cos(((teta + rel_ugao) / krug180) * M_PI);
				Y_pos_sin = sin(((teta + rel_ugao) / krug180) * M_PI);
				X_pos_cos = rel_distanca * X_pos_cos;
				Y_pos_sin = rel_distanca * Y_pos_sin;
		
				X_cilj = X_pos + (signed long)(X_pos_cos);
				Y_cilj = Y_pos + (signed long)(Y_pos_sin);
				
				//slanje odgovora
				i = 0;
				sendArray[0] = receiveArray[0];
				while (i < 1)
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_C0_data, sendArray[i]);
					//sendArray[1] ^= sendArray[i];	//CHC
					if(byteToBuffer)
					{
						i++;
					}
				}
				RX_i_C0 = 0;
			}
			RX_i_C0 = 0;		//ako ne valja CHC ponistava se komanda
		}
	}
	//SET DIRECT OUT - 5
	else if(receiveArray[0] == 5)					//provera funkcijskog bajta
	{
		if(RX_i_C0 >= 3)							//stigla je cela poruka
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			{
				//ENABLE
				set_direct_out = 1;
				
				if(receiveArray[1] >= 128)
					PID_brzina_L = (receiveArray[1] - 128) * 5;	//podesiti faktor!
				else
					PID_brzina_L = (128 - receiveArray[1]) * (-5);
					
				if(receiveArray[2] >= 128)
					PID_brzina_R = (receiveArray[2] - 128) * 5;	//podesiti faktor!
				else
					PID_brzina_R = (128 - receiveArray[2]) * (-5);
				
				//slanje odgovora
				i = 0;
				sendArray[0] = receiveArray[0];
				while (i < 1)
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_C0_data, sendArray[i]);
					//sendArray[1] ^= sendArray[i];	//CHC
					if(byteToBuffer)
					{
						i++;
					}
				}
				RX_i_C0 = 0;
			}
			RX_i_C0 = 0;		//ako ne valja CHC ponistava se komanda
		}
	}			
	//PODESAVANJE FET izlaza i servoa - 6
	else if(receiveArray[0] == 6)					//provera funkcijskog bajta 
	{
		if(RX_i_C0 >= 7)							//stigla je cela poruka	(11)
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
// 			i = 0;
// 			CHC = 0;
// 			for(i=0; i<= 4; i++)					//racunanje CHC
// 			CHC ^= receiveArray[i];

			//if(receiveArray[x] == CHC)				//CHC ok
			{
				PORTC.OUT |= receiveArray[1] & receiveArray[2];	//izlazi + maska
				PORTC.OUT &= ~(receiveArray[1] ^ receiveArray[2]);	//izlazi + maska
				
				//120 - nulti polozaj, 280 - krajnji polozaj
				TCF0.CCA = receiveArray[3] + 120;
				TCF0.CCB = receiveArray[4] + 120;
				TCF0.CCC = receiveArray[5] + 120;
				TCF0.CCD = receiveArray[6] + 120;
			
				//slanje odgovora
				i = 0;
				sendArray[0] = receiveArray[0];
				while (i < 1)
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_C0_data, sendArray[i]);
					//sendArray[1] ^= sendArray[i];	//CHC
					if(byteToBuffer)
					{
						i++;
					}
				}
				RX_i_C0 = 0;
			}
			RX_i_C0 = 0;	//ako ne valja CHC ponistava se komanda
		}		
	}		
	//Upis trenutne pozicije - 7
	else if(receiveArray[0] == 7)					//provera funkcijskog bajta
	{
		if(RX_i_C0 >= 7)							//stigla je cela poruka	(11)
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			{
				//x_pos
				if(!(receiveArray[1] == 0xFF && receiveArray[2] == 0xFF))	//FF se koristi ako ne zelimo da menjamo parametar
				{
					X_pos = 0;
					X_pos |= ((int)receiveArray[1]) << 8;
					X_pos |= (int)receiveArray[2];
					X_pos = (X_pos * scale_factor_for_mm);
					X_cilj = X_pos;
					X_cilj_stari = X_pos;
				}
				//Y_pos
				if(!(receiveArray[3] == 0xFF && receiveArray[4] == 0xFF))	//FF se koristi ako ne zelimo da menjamo parametar
				{
					Y_pos = 0;
					Y_pos |= ((int)receiveArray[3]) << 8;
					Y_pos |= (int)receiveArray[4];
					Y_pos = Y_cilj * scale_factor_for_mm;
					Y_cilj = Y_pos;
					Y_cilj_stari = Y_pos;
				}
				
				//teta
				if(!(receiveArray[5] == 0xFF && receiveArray[6] == 0xFF))	//FF se koristi ako ne zelimo da menjamo parametar
				{
					teta = 0;	//i ovde ako se zadaje 0xFF nece se uvazavati ovaj parametar
					teta |= ((int)receiveArray[5]) << 8;
					teta |= (int)receiveArray[6];
					teta = (teta * krug360) / 360;
					teta_cilj = teta;
				}
				
				//slanje odgovora
				i = 0;
				sendArray[0] = receiveArray[0];
				while (i < 1)
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_C0_data, sendArray[i]);
					//sendArray[1] ^= sendArray[i];	//CHC
					if(byteToBuffer)
					{
						i++;
					}
				}
				RX_i_C0 = 0;
			}
			RX_i_C0 = 0;	//ako ne valja CHC ponistava se komanda
		}
	}
	//Total Stop - 8
	else if(receiveArray[0] == 8)					//provera funkcijskog bajta
	{
		if(RX_i_C0 >= 3)							//stigla je cela poruka
		{
			proveri_vreme_primanja = 0;				//zastita iskljucena
			{
				
				if(receiveArray[1] == 0x01)
				{
					stop_PID_levi = 1;
					PID_brzina_L = 0;
				}				
				else
					stop_PID_levi = 0;
					
				if(receiveArray[2] == 0x01)
				{
					stop_PID_desni = 1;
					PID_brzina_R = 0;
				}					
				else
					stop_PID_desni = 0;
				
				//slanje odgovora
				i = 0;
				sendArray[0] = receiveArray[0];
				while (i < 1)
				{
					bool byteToBuffer;
					byteToBuffer = USART_TXBuffer_PutByte(&USART_C0_data, sendArray[i]);
					//sendArray[1] ^= sendArray[i];	//CHC
					if(byteToBuffer)
					{
						i++;
					}
				}
				RX_i_C0 = 0;
			}
			RX_i_C0 = 0;		//ako ne valja CHC ponistava se komanda
		}
	}
}

//DESNI PASIVNi QDEC
ISR(TCD0_OVF_vect)
{
	if((TCD0.CTRLFSET & TC0_DIR_bm) == 0)
		PASIVNI_QDEC_OWF_R = 1;
	
	if((TCD0.CTRLFSET & TC0_DIR_bm) == 1)
		PASIVNI_QDEC_OWF_R = 2;
}
//LEVI PASIVNI QDEC
ISR(TCD1_OVF_vect)
{
	if((TCD1.CTRLFSET & TC1_DIR_bm) == 0)
		PASIVNI_QDEC_OWF_L = 1;
	
	if((TCD1.CTRLFSET & TC1_DIR_bm) == 1)
		PASIVNI_QDEC_OWF_L = 2;
}

////tajmer za servo
//ISR(TCF0_OVF_vect)
//{
	//return;
//}


ISR(USARTE0_DRE_vect)
{
	USART_DataRegEmpty(&USART_E0_data);
}
ISR(USARTE1_DRE_vect)
{
	USART_DataRegEmpty(&USART_E1_data);
}
ISR(USARTC0_DRE_vect)
{
	USART_DataRegEmpty(&USART_C0_data);
}

//Interrupt na taster
ISR(PORTB_INT0_vect)
{
	PORT_TogglePins(&PORTC, 0xFF);
	//PORT_TogglePins(&PORTF, 0x08);
	//USART_TXBuffer_PutByte(&USART_C0_data, 43);	//+
	
	broj = motor_sample_L16; //ispis zeljene promenljive
	
	if(broj	> 0)	
		USART_TXBuffer_PutByte(&USART_E1_data, 43);	//+
	else if(broj < 0)	
		USART_TXBuffer_PutByte(&USART_E1_data, 45);	//-
	else
		USART_TXBuffer_PutByte(&USART_E1_data, 32);	//space		
	if(broj < 0)	//pozitiviziramo broj
		broj = broj * (-1);	
	broj1 = broj % 10 | 0x30;	
	broj = broj / 10;
	broj10 = broj % 10 | 0x30;
	broj = broj / 10;
	broj100 = broj % 10 | 0x30;
	broj = broj / 10;
	broj1000 = broj % 10 | 0x30;
	broj = broj / 10;
	broj10000 = broj % 10 | 0x30;
	USART_TXBuffer_PutByte(&USART_E1_data, broj10000);
	USART_TXBuffer_PutByte(&USART_E1_data, broj1000);
	USART_TXBuffer_PutByte(&USART_E1_data, broj100);
	USART_TXBuffer_PutByte(&USART_E1_data, broj10);
	USART_TXBuffer_PutByte(&USART_E1_data, broj1);
	USART_TXBuffer_PutByte(&USART_E1_data, 32);	//space		
	USART_TXBuffer_PutByte(&USART_E1_data, 0x3A);	//:
	USART_TXBuffer_PutByte(&USART_E1_data, 32);	//space		
	
	
	broj = motor_sample_R16;	//ispis zeljene promenljive
	
	if(broj	> 0)	
		USART_TXBuffer_PutByte(&USART_E1_data, 43);	//+
	else if(broj < 0)	
		USART_TXBuffer_PutByte(&USART_E1_data, 45);	//-
	else
		USART_TXBuffer_PutByte(&USART_E1_data, 32);	//space	
	if(broj < 0)	//pozitiviramo broj
		broj = broj * (-1);
	broj1 = broj % 10 | 0x30;	
	broj = broj / 10;
	broj10 = broj % 10 | 0x30;
	broj = broj / 10;	
	broj100 = broj % 10 | 0x30;
	broj = broj / 10;
	broj1000 = broj % 10 | 0x30;
	broj = broj / 10;
	broj10000 = broj % 10 | 0x30;
	USART_TXBuffer_PutByte(&USART_E1_data, broj10000);
	USART_TXBuffer_PutByte(&USART_E1_data, broj1000);
	USART_TXBuffer_PutByte(&USART_E1_data, broj100);
	USART_TXBuffer_PutByte(&USART_E1_data, broj10);
	USART_TXBuffer_PutByte(&USART_E1_data, broj1);
	USART_TXBuffer_PutByte(&USART_E1_data, 13);	//CR
}

