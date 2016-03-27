/*
 * funkcije.h
 *
 * Created: 27/03/16 14:36:33
 *  Author: AXIS team 
 */ 


#ifndef FUNKCIJE_H_
#define FUNKCIJE_H_

char proveri_poziciju(void);
void nuliraj_poziciju_robota(void);
void zadaj_X_Y_teta(signed long x, signed long y, signed long teta_des, unsigned char dir);
void zadaj_X_Y(signed long x, signed long y, unsigned char dir);
void zadaj_teta(signed long teta_des, unsigned char dir);
void idi_pravo(signed long x, signed long y, unsigned long ugao);
void idi_unazad(signed long x, signed long y, unsigned long ugao);
void rotiraj();

void inicijalizuj_bluetooth();
void sendMsg(char *poruka);
void sendChar(char c);

void inicijalizuj_servo_tajmer_20ms();
void pomeri_servo_1(uint16_t deg);

void demo_1(void);
void demo_2(void);
void demo_3(void);

#endif /* FUNKCIJE_H_ */