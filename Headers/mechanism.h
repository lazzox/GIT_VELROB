/*
 * mechanism.h
 *
 * Created: 4/19/2011 6:27:43 PM
 * Author: robert kovacs
 */ 


#ifndef MECHANISM_H_
#define MECHANISM_H_

void Racunanje_trenutne_pozicije(void);
void PID_brzinski(void);
void PID_ugaoni(void);
void PID_pravolinijski(void);
void PID_izlazni(void);
void Pracenje_pravca(void);
void CheckInputMotorControl(void);
void Communication(void);


#endif /* MECHANISM_H_ */