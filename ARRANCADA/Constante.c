// Constantes.h

#ifndef CONSTANTES_H
#define CONSTANTES_H

// Declaração de Constantes
//////////////////////////////////////// BOTOES ////////////////////////////////////////
#define BotCalibra A3
#define BotStart A0
#define BUZZ A1
//////////////////////////////////////// SENSOR ////////////////////////////////////////
#define QTSensores 11
#define MUX_SIG A2
const int MUX_S[4] = {5, 4, 3, 2}; // Pinos do mux {A,B,C,D}
#define BRANCO 0
#define PRETO 1
#define NumLeituras 5
#define AumentaCorte 0
//////////////////////////////////////// MOTOR ////////////////////////////////////////
#define ENA 6 // Esquerda
#define IN1 7 // Esquerda
#define IN2 8 // Esquerda
#define IN3 11 // Direita
#define IN4 12 // Direita
#define ENB 10 // Direita
#define STBY 9

#define PWME 200
#define PWMD 200
#define MAXR 200
//////////////////////////////////////// Square ////////////////////////////////////////
#define BUFFER_SIZE 5
#define re 100
#define PraFrente 0

#endif // CONSTANTES_H
