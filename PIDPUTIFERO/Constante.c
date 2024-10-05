// Constantes.h

#ifndef CONSTANTES_H
#define CONSTANTES_H

// Declaração de Constantes

#define Delta 20
#define DifTras 18
const int Branco = 1;
const int Preto = 0;
//////////////////////////////////////// BOTOES ////////////////////////////////////////
#define BotCalibra A3
#define BotStart A0
#define BUZZ A1
//////////////////////////////////////// SENSOR ////////////////////////////////////////
#define QTSensores 11
#define MUX_SIG A2
const int MUX_S[4] = {2, 3, 4, 5}; // Pinos do mux {A,B,C,D}
#define BRANCO 0
#define PRETO 1
#define NumLeituras 5
//////////////////////////////////////// MOTOR ////////////////////////////////////////
#define ENA 6 // Esquerda
#define IN1 8 // Esquerda
#define IN2 7 // Esquerda
#define IN3 10 // Direita
#define IN4 11 // Direita
#define ENB 12 // Direita

#define PWME 80
#define PWMD 80
#define MAXR 80
//////////////////////////////////////// AUXLIARES ////////////////////////////////////////
#define Parar 0
#define Frente 1
#define Direita 2
#define Esquerda 3

#endif // CONSTANTES_H
