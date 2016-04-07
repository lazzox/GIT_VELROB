/*
 * main.c
 *
 * Poslednje_izmene: 27/03/2016 01:47:41
 * Autor: AXIS team
 
 Izmene:
 -Dodate funkcije za sendMsg/sendChar
 -Izbacena inicijalizacija bluetooth-a
 -Dodat PGM_Mode funkciju
 -PID nije najbolji, ali uvek stigne gde treba :)
 
 
 
 Potrebne izmene: 
 -Promeniti baudrate (prepisano od malog robota)
 -Dodati displej :)
 
 # NAUCITE DA DODAJETE POTPIS FUNKCIJE U HEADER FAJLA GDE KREIRATE NOVU FUNKCIJU KAKO BI ONA ISPRAVNO RADILA!!!
 
 
 
 */ 

#include <avr/io.h>
#include "Headers/avr_compiler.h"
#include "Headers/usart_driver.h"
#include "Headers/port_driver.h"
#include "Headers/adc_driver.h"
#include "math.h"
#include "Headers/globals.h"
#include "Headers/mechanism.h"
#include "Headers/hardware.h"
#include "Headers/funkcije.h"

volatile signed int
PID_brzina_L,
PID_brzina_R,
motor_sample_L16,
motor_sample_R16,
Pe_brzina_L,
Pe_brzina_R,
Ie_brzina_L,
Ie_brzina_R;

volatile unsigned int PRG_flag = 0;

volatile signed long
rastojanje_cilj,
PID_teta;

volatile float
sharp1_value;

int main(void)
{
	int msg_counter = 0;
	int servo_counter = 0;
	okay_flag = 0;
	vreme_primanja = 0;

	Podesi_Oscilator();					//podesavanje oscilatora
	Podesi_Parametre_Robota();			//podesavanje broja impulsa u krugu
	Podesi_PID_Pojacanja();				//podesavanje pojacanja PID regulatora
	PodesiADC();						//podesavanje AD konvertora
	Podesi_Tajmere();					//podesavanje tajmera
	Podesi_QDEC();						//podesavanje kvadraturnih dekodera
	Podesi_PWM();						//podesavanje PWM signala za motore i servoe
	Podesi_Interapt();					//podesavanje interapt prioriteta
	Podesi_Pinove();					//podesavanje I/O pinova
	Podesi_USART_Komunikaciju();		//podesavanje komunikacije
	
	_delay_ms(1000);					//cekanje da se stabilizuje sistem
	nuliraj_poziciju_robota(); 	
	
	//idi_pravo(500,0,0);
	while(1)
	{
		//CHECK PGM MODE - Uvek mora biti ispred svega!
		//while(PGM_Mode()){
			//set_direct_out = 1;
			//PID_brzina_L = 0;
			//PID_brzina_R = 0;
				//if (!PRG_flag){
					//sendMsg("PGM_Mode");
					//PRG_flag = 1;
				//}
			//_delay_ms(500);
		//}
		set_direct_out = PRG_flag = 0;
		
	//---------------------------------------------------------------------//
	//------------------------------TAKTIKA--------------------------------//
	//---------------------------------------------------------------------//
			//kocka();
			//proba();
	//---------------------------------------------------------------------//
	//---------------TAKTIKA-----------------------------------------------//
	//---------------------------------------------------------------------//
	//---------------Racunanje trenutne pozicije---------------------------//
		if (Rac_tren_poz_sample_counter >= 3){		//9ms   3
			Rac_tren_poz_sample_counter = 0;
			Racunanje_trenutne_pozicije();
		}
		
		//Korekcija pravca i distance prema cilju
		if(Pracenje_Pravca_sample_counter >= 30){	//90ms   30
			
			msg_counter++;
			servo_counter++;
			Pracenje_Pravca_sample_counter = 0;
			Pracenje_pravca();
		}
		
		//PID regulacija
		if(PID_pozicioni_sample_counter >= 3){		//9ms    3
			PID_pozicioni_sample_counter = 0;
			PID_ugaoni();
			PID_pravolinijski();			
			//PID_brzinski se poziva direktno u interaptu sistemskog tajmera TCE1!
		}
		
		if(vreme_primanja > 500){
			vreme_primanja = 0;
			RX_i_E0 = 0;
		}
		
		if (okay_flag == 1){
			SendChar('O');
			okay_flag = 0;
		}
	}
}
