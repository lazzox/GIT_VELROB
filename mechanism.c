/*
 * mechanism.c
 *
 * Created: 4/19/2011 3:24:54 PM
 *  Author: robert kovacs
 */
#include "Headers/avr_compiler.h"
#include "Headers/usart_driver.h"
#include "Headers/port_driver.h"
#include "Headers/globals.h"
#include "Headers/mechanism.h"
#include "math.h"
#include "float.h"

volatile float
//regulacija
Kp_pravolinijski,
Kd_pravolinijski,
Ki_pravolinijski,

Kp_teta_okretanje,
Kp_teta_pravolinijski,

Kp_teta,
Kd_teta,
Ki_teta,

Kp_brzina,
Ki_brzina,
Kd_brzina,

flag_krug;


volatile unsigned char
//regulacija
stop_PID_levi,
stop_PID_desni,
set_direct_out,
smer_zadati,
stigao_flag = 0,
stigao_sigurnosni,
struja_L,
struja_R,
//komunikacija
sendArray[128],
receiveArray[128],
vreme_primanja,
okay_flag,
CHC,
RX_i_E0,
RX_i_E1,
RX_i_C0,
proveri_vreme_primanja,
velicina_niz_1,
velicina_niz_2,
velicina_niz_3,
sample_time_niz_1,
sample_time_niz_2,
sample_time_niz_3,
sample_counter_niz_1,
sample_counter_niz_2,
sample_counter_niz_3,
niz_counter_niz_1,
niz_counter_niz_2,
niz_counter_niz_3,
//qdec
MOTOR_QDEC_OWF_R,
MOTOR_QDEC_OWF_L,
PASIVNI_QDEC_OWF_L,
PASIVNI_QDEC_OWF_R,
//sistem tajming
Rac_tren_poz_sample_counter = 0,
PID_pozicioni_sample_counter = 1,
Pracenje_Pravca_sample_counter = 2;	//broje sa pomerrenim fazama


volatile signed char
fsm_crveni_step=0,
fsm_zuti_step=0,
smer_trenutni,
offset;	//za ADC

volatile unsigned int
broj1, broj10, broj100, broj1000, broj10000,
sysvrem,
fsm_timer,
count_L,
last_count_L,
motor_count_R,
motor_count_L,
last_motor_count_R = 0,
last_motor_count_L = 0,
scale_factor_for_mm,
vreme_cekanja_tete,
vreme_pozicioniranja,
sys_time,
Accel_PID_pos,
meca;

volatile signed int
broj,
niz_1[500],
niz_2[500],
niz_3[500],
//ODOMETRIJA
translacija_10ms,
rotacija_10ms,
sample_R16,
sample_L16,
motor_sample_R16,
motor_sample_L16,
PID_brzina_L,
PID_brzina_pret_L,
PID_brzina_R,
PID_brzina_pret_R,
Pe_brzina_L,
Pe_brzina_R,
Ie_brzina_L,
Ie_brzina_R,
De_brzina_L,
De_brzina_R,
timeout,

rezervni_ugao,
modifikovana_zeljena_pravolinijska_brzina,
max_brzina_motora,
zeljena_pravolinijska_brzina,
zeljena_brzina_okretanja,
PWM_perioda;

volatile signed long	
Pasivni_desni_CNT,
count_R,
last_count_R,
adresa_long,
vrednost_prom,
adresa_niz_1,
adresa_niz_2,
adresa_niz_3,
//REGULACIJA
X_pos,
Y_pos,
X_cilj,
Y_cilj,
X_cilj_stari,
Y_cilj_stari,
rel_distanca,
rel_ugao,
translacija,
teta,
teta_cilj,
teta_cilj_final = 0xFFFFFFFF, //ako je FF onda robot drzi ugao kojom je stigao do cilja
teta_greska,
teta_greska_prethodno,
teta_greska_sum,
PID_teta,
PID_pozicija,
PID_pozicija_pret,
rastojanje_cilj,
rastojanje_cilj_temp,
pozicija_greska,
pozicija_pret_greska,
pozicija_greska_sum,
PID_ukupni_L,
PID_ukupni_R,
dif_error_ugao,
dif_error_pravolinijski,
//KRUG i METAR
metar,
krug45,
krug90,
krug180,
krug180_PI,
krug360;


void Racunanje_trenutne_pozicije(void)
{
	//uzimanje stanja enkodera
	count_L = TCD1.CNT;
	count_R = TCD0.CNT;
	
	//desni enkoder
	if(PASIVNI_QDEC_OWF_R == 0)
	{
		sample_R16 = (count_R - last_count_R);
	}
	else
	{
		if(PASIVNI_QDEC_OWF_R == 1)
		{
			sample_R16 = count_R + ~(last_count_R);
		}
		else if(PASIVNI_QDEC_OWF_R == 2)
		{
			sample_R16 = -(last_count_R + ~(count_R));
		}
		
		PASIVNI_QDEC_OWF_R = 0;
	}
	
	//levi enkoder
	if(PASIVNI_QDEC_OWF_L == 0)
	{
		sample_L16 = (count_L - last_count_L);
	}
	else
	{
		if(PASIVNI_QDEC_OWF_L == 1)
		{
			sample_L16 = count_L + ~(last_count_L);
		} 
		else if(PASIVNI_QDEC_OWF_L == 2)
		{
			sample_L16 = -(last_count_L + ~(count_L));
		}
		
		PASIVNI_QDEC_OWF_L = 0;
	}
	
	
	//osvezavanje last_ promenjive
	last_count_R = count_R;
	last_count_L = count_L;
	
	//potrebne vrednosti za trigonometriju
	translacija_10ms = (sample_R16 + sample_L16);
	rotacija_10ms = (sample_R16 - sample_L16);
	teta += rotacija_10ms;
	translacija += ((long)(translacija_10ms));
	
	//ako predje ceo krug u pozitivnom smeru  //PROVERI OVO
	if(teta >= krug360)
		teta -= krug360;
	
	//ako predje u negativan smer
	if(teta < 0)
		teta += krug360;	
	
	//racunanje pozicije
	double X_pos_cos, Y_pos_sin;
	X_pos_cos = cos((double)teta / krug180_PI);
	Y_pos_sin = sin((double)teta / krug180_PI);
	X_pos += (int)(((double)translacija_10ms * X_pos_cos));
	Y_pos += (int)(((double)translacija_10ms * Y_pos_sin));
}

void Pracenje_pravca(void) 
{
	//Ulaz X_cilj i Y_cilj
	//Izlaz je teta_cilj i rastojanje_cilj
	double X_razlika, Y_razlika, XY_zbir, teta_razlika, teta_cilj_radian;
	
	//ako stignu nove zadate koordinate
	if (X_cilj_stari != X_cilj || Y_cilj_stari != Y_cilj)	
	{
		rezervni_ugao = krug45/45;	//precizno se pozicioniramo u mestu
		stigao_flag = 0;
	}
	X_cilj_stari = X_cilj;
	Y_cilj_stari = Y_cilj;
	
	X_razlika = (X_cilj - X_pos);
	Y_razlika = (Y_cilj - Y_pos);
	X_razlika *= X_razlika;
	Y_razlika *= Y_razlika;
	XY_zbir = X_razlika + Y_razlika;
	rastojanje_cilj_temp = sqrt(XY_zbir);
	
	if(rastojanje_cilj_temp > (metar / 12 ))  // metar/12
	{
		rastojanje_cilj = rastojanje_cilj_temp;
		translacija = 0;
		vreme_pozicioniranja = 0;
		stigao_flag = 0;
		
		X_razlika = (X_cilj - X_pos);
		Y_razlika = (Y_cilj - Y_pos);
		teta_cilj_radian = atan2((double)(Y_razlika), (double)(X_razlika));
		
		teta_cilj = (signed long)(teta_cilj_radian *krug180_PI);
	
		
		//Za automatsko kontanje rikverca po uglu
		if((smer_zadati) == 0)	//Sam bira smer
		{
			teta_razlika = teta - teta_cilj;
			if(teta_razlika > (krug90) || teta_razlika < (-krug90))
			{
				smer_trenutni = -1;
				teta_cilj -= krug180;
			}	
			else
			{
				smer_trenutni = 1;
			}
		}
		else if(smer_zadati == 1)	//Samo napred
		{
			smer_trenutni = 1; //1
		}
		else if(smer_zadati == 2)	//Samo nazad
		{
			smer_trenutni = -1; //-1
			teta_cilj -= krug180;
			
		}
	
		if(teta_cilj < 0)
			teta_cilj += krug360;
	}
	else if (vreme_pozicioniranja>800)
	{
		if (stigao_sigurnosni) 
		{
			stigao_sigurnosni=0;
		}
	}
	else if (vreme_pozicioniranja >= 600 && !stigao_sigurnosni)	//stigli smo do cilja
	{
		if (stigao_flag == 0 )//&& stigao_sigurnosni)
		{
			SendChar_USB('S');
			SendChar_USB('t');
			SendChar_USB('t');
			SendChar_USB('S');
			stigao_flag = 1;
			stigao_sigurnosni=0;
			vreme_pozicioniranja=0;
//  			USART_TXBuffer_PutByte(&USART_E0_data, 75);	//O
//  			USART_TXBuffer_PutByte(&USART_E0_data, 75);	//K
//  			USART_TXBuffer_PutByte(&USART_E0_data, 33);	//!
//  			USART_TXBuffer_PutByte(&USART_E1_data, 79);	//O
//  			USART_TXBuffer_PutByte(&USART_E1_data, 75);	//K
//  			USART_TXBuffer_PutByte(&USART_E1_data, 33);	//!
		}
		
		if (teta_cilj_final != 0xFFFFFFFF)	//ako treba zauzmemo krajnji ugao
		{
			teta_cilj = teta_cilj_final;
			teta_cilj_final = 0xFFFFFFFF;	//postavlja se na FF, da sledeci put ne bi se izvrsavao
		}		
	}
}

void PID_pravolinijski(void)
{	
	pozicija_greska = rastojanje_cilj * smer_trenutni - translacija;
	dif_error_pravolinijski = PID_pozicija - PID_pozicija_pret;
	pozicija_greska_sum += pozicija_greska;
	
	//anti wind-up
	if(pozicija_greska_sum > 300)
		pozicija_greska_sum = 300;
	else if(pozicija_greska_sum < -300)
		pozicija_greska_sum = -300;
	
	//za 50cm greske se dobija zeljena_pravoliijska_brzina (kada je Kp_pravolinijski = 1)
	PID_pozicija =	((float)(pozicija_greska*Kp_pravolinijski) + 
					(float)(dif_error_pravolinijski*Kd_pravolinijski) + 
					(float)(pozicija_greska_sum*Ki_pravolinijski)) /
					((float)((metar>>1 ) / zeljena_pravolinijska_brzina));	

	//ogranicenje
	if(PID_pozicija < -modifikovana_zeljena_pravolinijska_brzina)
		PID_pozicija = -modifikovana_zeljena_pravolinijska_brzina;
	if(PID_pozicija > modifikovana_zeljena_pravolinijska_brzina)
		PID_pozicija = modifikovana_zeljena_pravolinijska_brzina;
		
	//if(PID_pozicija < -zeljena_pravolinijska_brzina)
		//PID_pozicija = -zeljena_pravolinijska_brzina;
	//if(PID_pozicija > zeljena_pravolinijska_brzina)
		//PID_pozicija = zeljena_pravolinijska_brzina;
		
		
	//ubrzavanje po rampi
	if(PID_pozicija < 0)
	{
		if(PID_pozicija_pret > PID_pozicija)	//UBRZANJE U MINUS
		{
			if((abs(PID_pozicija) - abs(PID_pozicija_pret)) > Accel_PID_pos)
				PID_pozicija = PID_pozicija_pret - Accel_PID_pos;
		}
	}
	else
	{
		if(PID_pozicija_pret < PID_pozicija)	//UBRZANJE U PLUS
		{
			if((abs(PID_pozicija) - abs(PID_pozicija_pret)) > Accel_PID_pos)
				PID_pozicija = PID_pozicija_pret + Accel_PID_pos;
		}
	}
	PID_pozicija_pret = PID_pozicija;
}

void PID_ugaoni(void)
{	
	teta_greska = teta_cilj - teta;
	
	dif_error_ugao = teta_greska - teta_greska_prethodno; // D dejstvo
	teta_greska_prethodno = teta_greska; //D dejstvo
	
	//korigovanje greske, da bi se roobot uvek okretao u blizem smeru
	if(teta_greska <= -krug180)
		teta_greska += krug360;
	else if(teta_greska > krug180)
		teta_greska -= krug360;
		
	teta_greska_sum += teta_greska;
	
	//anti wind-up
	if(teta_greska_sum > 200)
		teta_greska_sum = 200;
	else if(teta_greska_sum < -200)
		teta_greska_sum = -200;
	
	//podesavanje pravca robota dok ne stigne u blizinu cilja
	if(rastojanje_cilj_temp > (metar/10))  /// bilo /10 ? 
	{
		if(labs(teta_greska) > 1500)	//okrecemo se u mestu kad treba
		{
			modifikovana_zeljena_pravolinijska_brzina = 0;	//zaustavlja se robot za okretanje u mestu
			rezervni_ugao = krug45/45;
			vreme_cekanja_tete = 0;
		}
		else if(vreme_cekanja_tete >= 200)
		{
			//stigao_flag = 2;
			vreme_cekanja_tete = 0;
			modifikovana_zeljena_pravolinijska_brzina=zeljena_pravolinijska_brzina;
			//Kp_teta=Kp_teta_pravolinijski;
			
			
				// robot se krece pravolinijski
		}
	}
	
	//PID izlaz:
	PID_teta =	((float)(teta_greska * Kp_teta) + 
				(float)(dif_error_ugao * Kd_teta) +
				(float)(teta_greska_sum * Ki_teta)) / 
				(krug90 / zeljena_brzina_okretanja);
	
	if(PID_teta < -zeljena_brzina_okretanja)
		PID_teta = -zeljena_brzina_okretanja;
	if(PID_teta > zeljena_brzina_okretanja)
		PID_teta = zeljena_brzina_okretanja;
}

void PID_brzinski(void)
{
	//////////////PID//////////////////
	//ako nema stop signala, postavlja se izlaz
	
	if((stop_PID_levi == 0) && (set_direct_out == 0))
		PID_brzina_L = (PID_pozicija - PID_teta);
	if((stop_PID_desni == 0) && (set_direct_out == 0))
		PID_brzina_R = (PID_pozicija + PID_teta); //+
	
	//Ogranicenje brzinskog PID-a
	if(PID_brzina_L > max_brzina_motora)
		PID_brzina_L = max_brzina_motora;
	if(PID_brzina_L < -max_brzina_motora)
		PID_brzina_L = -max_brzina_motora;
	if(PID_brzina_R > max_brzina_motora)
		PID_brzina_R = max_brzina_motora;
	if(PID_brzina_R < -max_brzina_motora)
		PID_brzina_R = -max_brzina_motora;
	
	Pe_brzina_L = PID_brzina_L;
	Pe_brzina_R = PID_brzina_R;
	Ie_brzina_L += Pe_brzina_L;
	Ie_brzina_R += Pe_brzina_R;
	De_brzina_L = PID_brzina_pret_L - PID_brzina_L;
	De_brzina_R = PID_brzina_pret_R - PID_brzina_R;
	
	PID_brzina_pret_L = PID_brzina_L;
	PID_brzina_pret_R = PID_brzina_R;
	
	//anti wind-up
	if(Ie_brzina_L > 200) //300
		Ie_brzina_L = 200;
	if(Ie_brzina_L < -200)
		Ie_brzina_L = -200;
	if(Ie_brzina_R > 200)
		Ie_brzina_R = 200;
	if(Ie_brzina_R < -200)
		Ie_brzina_R = -200;

	PID_ukupni_L = (float)(Pe_brzina_L) * Kp_brzina + (float)(Ie_brzina_L) * Ki_brzina + (float)(De_brzina_L) * Kd_brzina; // znak - je zbog smera kretanja
	PID_ukupni_R = (float)(Pe_brzina_R) * Kp_brzina + (float)(Ie_brzina_R) * Ki_brzina + (float)(De_brzina_R) * Kd_brzina;
	
	//preskaliranje - ne mora da se radi posto su max_brzina_motora i PWM_perioda slicne velicine
	//PID_ukupni_L = (PID_ukupni_L * PWM_perioda) / max_brzina_motora; 
	
	//Ogranicenje PID izlaza
	if(PID_ukupni_L > PWM_perioda)
		PID_ukupni_L = PWM_perioda;
	if(PID_ukupni_L < -PWM_perioda)
		PID_ukupni_L = -PWM_perioda;
		
	if(PID_ukupni_R > PWM_perioda)
		PID_ukupni_R = PWM_perioda;
	if(PID_ukupni_R < -PWM_perioda)
		PID_ukupni_R = -PWM_perioda;
		
	//levi motor
	if (PID_ukupni_L > 5)/*if (PID_ukupni_L > 5)*/	//smer 1
	{
		PORT_ClearPins(&PORTH, 0b00010000);	//IN_A2=0
		PORT_SetPins(&PORTH, 0b10000000);	//IN_B2=1
		TCF1.CCBBUF = PID_ukupni_L;
	}
	else if (PID_ukupni_L < -5)	//smer 2
	{
		PORT_ClearPins(&PORTH, 0b10000000);	//IN_B2=0
		PORT_SetPins(&PORTH, 0b00010000);	//IN_A2=1,
		TCF1.CCBBUF = -PID_ukupni_L;
	}
	else	//kocenje
		PORT_ClearPins(&PORTH, 0b10010000);	//IN_A2=0, IN_B2=0	
	//desni motor
	if (PID_ukupni_R > 5) //smer 1
	{
		PORT_ClearPins(&PORTH, 0b00001000);	//IN_B1=0
		PORT_SetPins(&PORTH, 0b00000001);	//IN_A1=1
		TCF1.CCABUF = PID_ukupni_R;
	}
	else if (PID_ukupni_R < -5)	//smer 2
	{
		PORT_ClearPins(&PORTH, 0b00000001);	//IN_A1=0
		PORT_SetPins(&PORTH, 0b00001000);	//IN_B1=1
		TCF1.CCABUF = -PID_ukupni_R;
	}
	else //kocenje
		PORT_ClearPins(&PORTH, 0b00001001);	//IN_A1=0, IN_B1=0
		
}

int PGM_Mode(){
	if((PORTB.IN & 0b00000001) == 0)
	{//Pritisnut taster
		return 1;
	}
	else{
		set_direct_out = 0;
		return 0;
	}
}
