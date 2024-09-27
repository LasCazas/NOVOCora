// Constantes.h

#ifndef CONSTANTES_H
#define CONSTANTES_H

// Declaração de Constantes

#define Delta 20
#define DifTras 18
const int Branco = 1;
const int Preto = 0;
//////////////////////////////////////// BOTOES ////////////////////////////////////////
#define BotCalibra 1
#define BotStart 1
//////////////////////////////////////// SENSOR ////////////////////////////////////////
#define QTSensores 11
#define MUX_SIG 1
const int MUX_S[4] = {1, 1, 1, 1}; // Inicializa o array com valores
#define BRANCO 0
#define PRETO 1
//////////////////////////////////////// MOTOR ////////////////////////////////////////
#define ENA 3 // Esquerda
#define IN1 5 // Esquerda
#define IN2 4 // Esquerda
#define IN3 7 // Direita
#define IN4 6 // Direita
#define ENB 9 // Direita

#define PWME 80
#define PWMD 80
#define MAXR 80
//////////////////////////////////////// AUXLIARES ////////////////////////////////////////
#define Parar 0
#define Frente 1
#define Direita 2
#define Esquerda 3

#endif // CONSTANTES_H