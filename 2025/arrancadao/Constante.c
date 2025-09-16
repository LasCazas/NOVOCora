// Constantes.h

#ifndef CONSTANTES_H
#define CONSTANTES_H

// Declaração de Constantes

#define Delta 20
#define DifTras 18
//////////////////////////////////////// BOTOES ////////////////////////////////////////
#define pinoMultifuncao A0
//#define BotStart A0
//#define BUZZ A1
//////////////////////////////////////// PID ////////////////////////////////////////

//////////////////////////////////////// SENSOR ////////////////////////////////////////
#define QTSensores 13
#define SENSOR_CENTRAL (QTSensores / 2)
#define MUX_SIG A0
const int MUX_S[4] = {14,12,13,15}; // Pinos do mux {A,B,C,D}0,4,5,16
#define BRANCO 0
#define PRETO 1
#define NumLeituras 2
//////////////////////////////////////// MOTOR ////////////////////////////////////////
const int pwmMotorD = 5; //d1 Velocidade (PWM) do motor esquerdo
const int dirMotorD = 0; //d3 Direção (DIR) do motor esquerdo

// Motor Direito (Motor B no shield)
const int pwmMotorE = 4; // d2Velocidade (PWM) do motor direito
const int dirMotorE = 2; //d4  Direção (DIR) do motor direito


#define PWME 150
#define PWMD 150
#define MAXR 200
//////////////////////////////////////// AUXLIARES ////////////////////////////////////////
#define Parar 0
#define Frente 1
#define Direita 2
#define Esquerda 3

#endif // CONSTANTES_H
