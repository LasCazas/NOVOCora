// Constantes.h

#ifndef CONSTANTES_H
#define CONSTANTES_H

// Declaração de Constantes

#define Delta 20
#define DifTras 18
//////////////////////////////////////// BOTOES ////////////////////////////////////////
//#define BotCalibra A3
//#define BotStart A0
//#define BUZZ A1
//////////////////////////////////////// SENSOR ////////////////////////////////////////
#define QTSensores 13
#define MUX_SIG A0
const int MUX_S[4] = {14,12,13,15}; // Pinos do mux {A,B,C,D}0,4,5,16
#define BRANCO 1

#define PRETO 0
#define NumLeituras 5
//////////////////////////////////////// MOTOR ////////////////////////////////////////
const int pwmMotorE = 5; //d1 Velocidade (PWM) do motor esquerdo
const int dirMotorE = 0; //d3 Direção (DIR) do motor esquerdo

// Motor Direito (Motor B no shield)
const int pwmMotorD = 4; // d2Velocidade (PWM) do motor direito
const int dirMotorD = 2; //d4  Direção (DIR) do motor direito


#define PWME 255
#define PWMD 255
#define MAXR 255
//////////////////////////////////////// AUXLIARES ////////////////////////////////////////
#define Parar 0
#define Frente 1
#define Direita 2
#define Esquerda 3

#endif // CONSTANTES_H
