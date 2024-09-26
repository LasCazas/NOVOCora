#include "Constante.c"
int ASenOE, ASenE, ASenC, ASenD, ASenOD;
bool SenOE, SenE,SenEE, SenC, SenD,SenDD, SenOD; 
int P = 0, I = 0, D = 0, PIDD = 0;
int erro = 0,erroA = 0,TempA = 0,TempQ =0;
int VeloE,VeloD,QuadradoE = 0,QuadradoD = 0,QuadRefE = 0,QuadRefD = 0,Es = 1;
bool LeAntC =0,LeAntD = 0 ,LeAntE = 0;
bool QuadradoDetectado = false; // Variável para controlar se um quadrado
unsigned long TempoInicioQuadrado = 0, TempErrando = 0;
unsigned long TempoLimiteQuadrado = 500,TempVirando = 0, TempVirandoMax = 1000;

void Leitura(){
  SenOE = digitalRead(PSenOE);
  SenOE = !SenOE;
  SenEE = digitalRead(PSenEE);
  SenEE = !SenEE;
  ASenE = analogRead(PSenE);
  ASenC = analogRead(PSenC);
  ASenD = analogRead(PSenD);
  SenDD = digitalRead(PSenDD);
  SenDD = !SenDD;
  ASenOD = analogRead(PSenOD);
  Discretiza();
  //Serial.println(SenEE);
  //Serial.println(String(SenOE) + " "+ String(ASenE) + " "+ String(ASenC) + " "+ String(ASenD) + " "+ String(ASenOD));
  Serial.println(String(SenOE) + " "+ String(SenE)  + " "+ String(SenC) + " "+ String(SenD)+ " "+ String(SenOD) + String(erro));
}
void Discretiza(){
  //SenOE = (ASenOE <= corteOE)? Branco :Preto;
  SenE = (ASenE <= corteE)? Branco :Preto;
  SenC = (ASenC <= corteC)? Branco :Preto;
  SenD = (ASenD <= corteD)? Branco :Preto;
  SenOD = (ASenOD <= corteOD)? Branco :Preto;
}
void CalculaErro(){ //negativo = mais pa esquerda positivo mais pa direita
       if((SenEE == Preto) && (SenE == Preto) && (SenC == Branco) && (SenD == Preto) && (SenDD == Preto) ){erro = 0;}
  else if ((SenEE == Preto) && (SenE == Branco) && (SenC == Branco) && (SenD == Preto) && (SenDD == Preto) ){erro = -1;}
  else if ((SenEE == Preto) && (SenE == Branco) && (SenC == Preto) && (SenD == Preto) && (SenDD == Preto) ){erro = -2;}
  else if ((SenEE == Branco) && (SenE == Branco) && (SenC == Preto) && (SenD == Preto) && (SenDD == Preto) ){erro = -2.5;}
  else if ((SenEE == Branco) && (SenE == Preto) && (SenC == Preto) && (SenD == Preto) && (SenDD == Preto) ){erro = -3;}
  else if ((SenOE ==Branco) &&  (SenC == Preto)  && (SenDD == Preto)&&(SenOD == Preto) ){erro = -5;}
  
  else if ((SenE == Preto) && (SenE == Preto) && (SenC == Branco) && (SenD == Branco) && (SenDD == Preto) ){erro = 1;}
  else if ((SenE == Preto) && (SenE == Preto) && (SenC == Preto) && (SenD == Branco) && (SenDD == Preto) ){erro = 2;}
  else if ((SenE == Preto) && (SenE == Preto) && (SenC == Preto) && (SenD == Branco) && (SenDD == Branco) ){erro = 2.5;}
  else if ((SenE == Preto) && (SenE == Preto) && (SenC == Preto) && (SenD == Preto) && (SenDD == Branco) ){erro = 3;}
  else if ((SenOE ==Preto)  && (SenC == Preto) && (SenD == Preto) &&(SenOD == Branco) ){erro = 5;}
  else{erro = erroA;}
  
}
void CalculaPID() {
  P = erro * Kp;
  I = I + erro;
  D = erro - erroA;
  AntiWindUp();// Limita a parte integrativa (anti-windup)
  PIDD = P + (Ki * I) + (Kd * D);
  erroA = erro;
}
void AntiWindUp(){ //funcao que limita o valor da roda 
  if (erro == 0){I = 0;}
  //if (I > 255) {I = 255;} 
  //else if (I < -255) {I = -255;}
  if ((erro >= 0 && erroA < 0) || (erro < 0 && erroA >= 0)) {I = 0;} // Zere a parte integrativa quando o sinal do erro muda
}
void Seguir() {
  CalculaErro();
  CalculaPID();
 
  if (PIDD < -MAXR) {PIDD = -MAXR;} // n deixa ele ser negativo
  if (PIDD > MAXR) {PIDD = MAXR;} // n deixa ele ser negativo
  if (PIDD > 0) {   // Dir
    VeloE = PWME;
    VeloD = PWMD - PIDD;
  } else {    // Esq
    VeloE = PWME - abs(PIDD);
    VeloD = PWMD;
  }
  
  //VeloD = constrain(VeloD,55,PWMD);
  //VeloE = constrain(VeloE,55,PWME);
  if (VeloD < 0) {VeloD = 0;} // n deixa ele ser negativo
  if (VeloE < 0) {VeloE = 0;} // n deixa ele ser negativo
  /*if((erro == 5) && (erroA > -2 && erroA < 2)){
    Andar(Direita);
    Serial.println("VirandoForte");
  } else if(erro == -5){
    Andar(Esquerda);
    Serial.println("VirandoForte");
  }else{*/
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, VeloE);
      // Motor_D
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, VeloD);
  //}
  //Serial.println("VeloE: " + String(VeloE) + " || VeloD: " + String(VeloD) + " || Erro: " + String(erro)+ " || PID: " + String(PIDD));
}


void ContQuad(){
  // Se um quadrado foi detectado e o tempo de detecção foi maior que o limite
  if ((millis() - TempoInicioQuadrado) >= TempoLimiteQuadrado)  {
    Serial.println("Passou");
    // Registre a detecção do quadrado
    if ((SenOE == Branco) && ((erro > -2) && (erro < 2) )) {  // Quadrado na Esquerda
      QuadradoE++;
      QuadRefE = QuadradoE;
   
      TempoInicioQuadrado = millis();

    } 
    // Zere a variável de detecção
    
  }
}
void SubViraQuad(){ // Subtrai os quadrados e vira no momento certo
  //if( ( (QuadradoD > 1) || (QuadRefD != QuadradoD)) && (SenC == Branco) && (SenDD == Branco) && (SenOE == Branco) ){QuadradoD = QuadradoD - 1;}
  if( ( (QuadradoE > 1) || (QuadRefE != QuadradoE)) && (SenC == Branco) && (SenEE == Branco) && (SenOE == Branco) ){QuadradoE = QuadradoE - 1;}
  
  if((QuadradoE == 1) && (SenC == Branco) && (SenEE == Branco) && (SenOE == Branco)){
    Es = 3;  //Virar para Esquerda
    Andar(Parar);
     delay(500);
    Andar(Esquerda);
    QuadradoE = 0;
    QuadRefE = 0;
    TempVirando = millis();
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
     analogWrite(ENA, VeloE);
      // Motor_D
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Virando");
  }
}
void setup() {
  //Sensores
  pinMode(PSenOE,INPUT);
  pinMode(PSenEE,INPUT);
  pinMode(PSenE,INPUT);
  pinMode(PSenC,INPUT);
  pinMode(PSenD,INPUT);
  pinMode(PSenDD,INPUT);
  pinMode(PSenOD,INPUT);
  //Motores
  pinMode(ENA,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENB,OUTPUT);
  Serial.begin(9600);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, VeloE+10);
      // Motor_D
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, VeloD+10);
  delay(500);
}
void loop() {
  Leitura();
  //Andar(Frente);
  //Le os Sensores
  if(Es == 1) {
    Seguir(); //Estado Padrao ele segue a linha

    ///////////////// Leitura de Quadrados /////////////////
    if (millis() - TempQ >= 500){//Impede a leitura de quadrados se ele n tiver andado mais de 1 segundo para frente dps de ter virado
      if ((LeAntE != SenOE) || (LeAntD != SenOD)){
        ContQuad();
        //Serial.println("passou =================");
        }
      SubViraQuad();  //Subtrai quadrados e analisa se esta na hora de virar
    } 
  } 
  ////ANALISA SE TERMINOU DE FAZER A CURVA, (TESTA SE VOLTOU PARA A LINHA PRA ESQUERDA) Es = 2 (Esquerda) DIR Es = 3 (direita)
  if ((millis() - TempVirando) >= TempVirandoMax){
  if((Es == 3) && (SenOE == Preto)  && (SenEE == Preto)&&((SenE == Branco) || (SenC == Branco) || (SenD == Branco) ) && (SenOD == Preto)){
    Es = 1; //Volta para o modo normal
    TempQ = millis(); //Ve o tempo que ele terminou a curva

  }
  }
  if ( (SenOE == Preto) && (SenEE == Preto) && (SenE == Preto) && (SenC == Preto) && (SenD == Preto) && (SenDD == Preto) && (SenOD == Preto)&& (LeAntC == Branco) ){
    Andar(Parar);
  }
  LeAntE = SenOE;
  LeAntD = SenOD;
  LeAntC = SenC;
  Serial.println("Quantidade Quadrados: " + String(QuadradoE) + " | |Estados: " + String(Es) + " erro:"  + String(erro));
  TempA = millis();
}
