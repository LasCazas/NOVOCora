#define PSenOE 2
#define PSenEE 3
#define PSenE 4
#define PSenD 5
#define PSenDD 6
#define PSenOD 7

#define Preto 0
#define Branco 1

/*
motores 
*/
#define ENA 3
#define IN1 3
#define IN2 3

#define IN3 3 // motor direito 
#define IN4 3
#define ENB 3
/*
pid
*/
#define Kp 15 
#define Ki 0
#define Kd 0
int P = 0, I = 0, D = 0, PID = 0;
int erro = 0,erroA = 0;

int PWM = 70;
int VeloE = 0, VeloD = 0; // variaveis de velocidade que vai mandar para o motor corrigido
int SenOE, SenEE,SenE, SenD, SenDD, SenOD;
void Leitura(){
  SenOE = digitalRead(PSenOE);
  SenEE = digitalRead(PSenEE);
  SenE = digitalRead(PSenE);
  SenD = digitalRead(PSenD);
  SenDD = digitalRead(PSenDD);
  SenOD = digitalRead(PSenOD);
  //Serial.println(String(SenOE) + " "+ String(SenEE) + " "+ String(SenE) + " "+ String(SenD) +" "+ String(SenDD) + " "+ String(SenOD ) );

}
void CalculaErro(){
  if ((SenEE == Preto) && (SenE == Preto) && (SenD == Preto)&& (SenDD == Branco)){erro = -2;}
  else if ((SenEE == Preto) && (SenE == Preto) && (SenD == Branco) && (SenDD == Branco)){erro = -1.5;}
  else if ((SenEE == Preto) && (SenE == Preto) && (SenD == Branco) && (SenDD == Preto)){erro = -1;}

  else if((SenEE == Preto) && (SenE == Preto) && (SenD == Preto)&& (SenDD == Preto) ){erro = 0;}
  else if ((SenEE == Preto) && (SenE == Branco) && (SenD == Branco) && (SenDD == Preto)){erro = 0;}

  else if ((SenEE == Preto) && (SenE == Branco) && (SenD == Preto) && (SenDD == Preto)){erro = 1;}
  else if ((SenEE == Branco) && (SenE == Branco) && (SenD ==  Preto)&& (SenDD ==  Preto) ){erro = 1.5;}
  else if ((SenEE == Branco) && (SenE == Preto) && (SenD == Preto) && (SenDD ==  Preto)){erro = 2;}
}
void CalculaPID() {
  P = erro * Kp;

  // Limita a parte integrativa (anti-windup)
  I = I + erro;
  if (I > 255) {I = 255;} 
  else if (I < -255) {I = -255;}

  // Verifique se o sinal do erro mudou de positivo para negativo
  if ((erro >= 0 && erroA < 0) || (erro < 0 && erroA >= 0)) {I = 0;} // Zere a parte integrativa quando o sinal do erro muda
  D = erro - erroA;
  PID = P + (Ki * I) + (Kd * D);
  erroA = erro;
}
void Seguir() {
  CalculaErro();
  CalculaPID();
  
  if (PID >= 0) {   // Esq
    VeloE = PWM - PID;
    VeloD = PWM;
  } else {    // Dir
    VeloE = PWM;
    VeloD = PWM - PID;
  }
  
  if (VeloD < 0) {VeloD = 0;} // n deixa ele ser negativo
  if (VeloE < 0) {VeloE = 0;} // n deixa ele ser negativo
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, VeloE);
  
  // Motor_D
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, VeloD);
  
  Serial.println("VeloE: " + String(VeloE) + " || VeloD: " + String(VeloD) + " || Erro: " + String(erro));
}
void setup() {
  //Sensores
  pinMode(PSenOE,INPUT);
  pinMode(PSenEE,INPUT);
  pinMode(PSenE,INPUT);
  pinMode(PSenD,INPUT);
  pinMode(PSenDD,INPUT);
  pinMode(PSenOD,INPUT);
  Serial.begin(9600);
}
void loop() {
  Leitura(); 
  Seguir();
}
