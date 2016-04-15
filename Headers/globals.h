/*
 * globals.h
 *
 * Created: 4/19/2011 2:16:36 PM
 *  Author: robert kovacs
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

#include "avr_compiler.h"
#include "usart_driver.h"

#define mmio32(x)   (*(volatile unsigned long *)(x))

// USART data struct  
USART_data_t USART_E0_data;
USART_data_t USART_E1_data;
USART_data_t USART_C0_data;

///////////////////////////Globalne promenljive///////////////////////
extern volatile float
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

extern volatile unsigned char
//regulacija
stop_PID_levi,
stop_PID_desni,
set_direct_out,
smer_zadati,
stigao_flag,
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
Rac_tren_poz_sample_counter,
PID_pozicioni_sample_counter,
Pracenje_Pravca_sample_counter;

extern volatile signed char
smer_trenutni,
offset; // za ADC


extern volatile unsigned int
broj1, broj10, broj100, broj1000, broj10000,
sysvrem,
count_L,
last_count_L,
motor_count_R,
motor_count_L,
last_motor_count_R,
last_motor_count_L,
scale_factor_for_mm,
vreme_cekanja_tete,
vreme_pozicioniranja,
sys_time,
Accel_PID_pos,
fsm_timer,
meca;

extern volatile signed int
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

extern volatile signed long	
//DESNI PASIVNI ENKODER
Pasivni_desni_CNT,
count_R,
last_count_R,
//KOMUNIKACIJA
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
teta_cilj_final,
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
							
#endif /* GLOBALS_H_ */