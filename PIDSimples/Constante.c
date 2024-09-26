#line 1 "C:\\Users\\LAIC\\Documents\\Arduino\\PIDSimples\\Constante.c"
//Declaração de Constantes
#define DELTAPULSO 11 //Pesquisei e deu 11 pulsos por volta
#define PWME 80//1.2    120   140
#define PWMD 80//100 116.6
#define MAXR 80

#define Delta 20
#define DifTras 18
const int Branco = 1;
const int Preto = 0;
#define Kp 16//17 25
#define Ki 0.00
#define Kd 0.0//0.55 
#define TempErrandooMax 500
// Define os pinos d  os sensores E,C,D,O,Esq,Centro,Direita,Oposto
#define PSenOE 12
#define PSenEE 11
#define PSenE A2
#define PSenC A3
#define PSenD A4
#define PSenDD 10
#define PSenOD A5

#define corteOE 700   
#define corteE 700
#define corteC 500
#define corteD 840
#define corteOD 600
//MOTORES
#define ENA 3 //Esquerda  //D1
#define IN1 5 //Esquerda //D2
#define IN2 4//Esquerda //D3
#define IN3 7//Direita  //D4
#define IN4 6//Direita //D5
#define ENB 9 //Direita  //D6
#define Parar 0
#define Frente 1
#define Direita 2
#define Esquerda 3
