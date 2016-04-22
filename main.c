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



volatile float
sharp1_value;

int main(void)
{
	int msg_counter = 0;
	int servo_counter = 0;
	okay_flag = 0;
	vreme_primanja = 0;
	stigao_flag_sigurnosni = 0;
	kraj_meca = 0;
	
	Podesi_Oscilator();					//podesavanje oscilatora
	Podesi_Parametre_Robota();			//podesavanje broja impulsa u krugu
	Podesi_PID_Pojacanja();				//podesavanje pojacanja PID regulatora
	PodesiADC();						//podesavanje AD konvertora
	Podesi_Tajmere();					//podesavanje tajmera
	Podesi_QDEC();						//podesavanje kvadraturnih dekodera
	Podesi_PWM();						//podesavanje PWM signala za motore i servo
	Podesi_Interapt();					//podesavanje interapt prioriteta
	Podesi_Pinove();					//podesavanje I/O pinova
	Podesi_USART_Komunikaciju();		//podesavanje komunikacije
	//inicijalizuj_servo_tajmer_20ms();	//Inicijalizuje tajmer za servoe
	//Kada se inicijalizuje ovaj tajmer prestane da radi USART za komunikaciju!
	
	nuliraj_poziciju_robota(); 
	_delay_ms(1000);					//cekanje da se stabilizuje sistem
	//postavi_sistem(210,1020,90);	

	SendChar_USB('U');
	SendChar_USB('U');
	SendChar_USB('U');
	
// 	idi_pravo(300,500,90);
// 	stigao_flag_sigurnosni = 1;
		
	while(1)
	{
		//CHECK PGM MODE - Uvek mora biti ispred svega!
		while(PGM_Mode()){
			set_direct_out = 1;
			PID_brzina_L = 0;
			PID_brzina_R = 0;
			if (!PRG_flag){
				sendMsg("PGM_Mode");
				PRG_flag = 1;
			}
			_delay_ms(500);
		}
		set_direct_out = PRG_flag = 0;
		
	//---------------------------------------------------------------------//
	//------------------------------TAKTIKA--------------------------------//
	//---------------------------------------------------------------------//
			 //kocka();
			// kocka_poy();
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
		
		if (okay_flag == 1){
			SendChar('O');
			SendChar('1');
			SendChar('2');
			SendChar('3');
			SendChar('4');
			SendChar('5');
			SendChar('6');
			SendChar('K');
			okay_flag = 0;
		}
		
		while (kraj_meca)
		{
			nuliraj_poziciju_robota();
			set_direct_out = 1;
			PID_ukupni_L = 0;
			PID_ukupni_R = 0;
			zeljena_brzina_okretanja=0;
			zeljena_pravolinijska_brzina=0;
			Kp_teta=0;
			Kd_teta=0;
			//  cli();
			   
		}
		
		if (stigao_flag == 1){
			SendChar('S');
			SendChar('1');
			SendChar('2');
			SendChar('3');
			SendChar('4');
			SendChar('5');
			SendChar('6');
			SendChar('T');
			stigao_flag=0;
			stigao_flag_sigurnosni = 0;
		}
		
		
		
		
	}
}
