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
const int MUX_S[4] = {5, 4, 3, 2}; // Pinos do mux {A,B,C,D}
#define BRANCO 1
#define PRETO 0
#define NumLeituras 5
//////////////////////////////////////// MOTOR ////////////////////////////////////////
#define ENA 6 // Esquerda
#define IN1 7 // Esquerda
#define IN2 8 // Esquerda
#define IN3 11 // Direita
#define IN4 12 // Direita
#define ENB 10 // Direita
#define STBY 9

#define PWME 80
#define PWMD 80
#define MAXR 60
//////////////////////////////////////// Square ////////////////////////////////////////
#define BUFFER_SIZE 5
#define re 1
#define PraFrente 0

#endif // CONSTANTES_H
