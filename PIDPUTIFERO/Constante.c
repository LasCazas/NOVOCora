// Constantes.h

#ifndef CONSTANTES_H
#define CONSTANTES_H

// Declaração de Constantes

#define Delta 20
#define DifTras 18
//////////////////////////////////////// BOTOES ////////////////////////////////////////
#define BotCalibra A3
#define BotStart A0
#define BUZZ A1
//////////////////////////////////////// SENSOR ////////////////////////////////////////
#define QTSensores 11
#define MUX_SIG A2
const int MUX_S[4] = {2, 3, 4, 5}; // Pinos do mux {A,B,C,D}
#define BRANCO 1
#define PRETO 0
#define NumLeituras 5
//////////////////////////////////////// MOTOR ////////////////////////////////////////
#define ENA 6 // Esquerda
#define IN1 7 // Esquerda
#define IN2 8 // Esquerda
#define IN3 11 // Direita
#define IN4 10 // Direita
#define ENB 12 // Direita
#define STBY 9

#define PWME 130
#define PWMD 130
#define MAXR 130
//////////////////////////////////////// AUXLIARES ////////////////////////////////////////
#define Parar 0
#define Frente 1
#define Direita 2
#define Esquerda 3

#endif // CONSTANTES_H
