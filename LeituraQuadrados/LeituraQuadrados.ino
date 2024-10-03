#include "Constante.c"

int Sensor[QTSensores] = {0}; // Inicializa zerando tudo
bool SensorBIN[QTSensores] = {0};
int HistoricoLeituras[QTSensores][NumLeituras];  // Armazena as últimas 5 leituras de cada sensor
int IndiceLeitura = 0;                    // Índice de controle para o histórico de leituras
bool Mandar_Mux_Bin[4] = {0};
int corte[QTSensores] = {0};

float erro = 0, erroA = 0;
int VeloE, VeloD; // valores a serem mandados no motor
//////////////////////////////////////// PID ////////////////////////////////////////
int P = 0, I = 0, D = 0, PID = 0;
float Kp = 1.0, Ki = 0.1, Kd = 0.01; // Parâmetros do PID
float targetValue = 0; // Valor alvo
bool autoTuningEnabled = false; // Habilitar/desabilitar auto-tuning
unsigned long lastTuneTime = 0; // Tempo da última atualização de tuning
const unsigned long tuneInterval = 1000; // Intervalo de tempo para ajuste
/////////////////////////////////////////////////////////////////////////////////////
int i = 0, j = 0;
/////////////////////////////////////// Quadrado ////////////////////////////////////
int bufferLeft[BUFFER_SIZE] = {0};  // Buffer para armazenar as últimas leituras do sensor esquerdo
int bufferRight[BUFFER_SIZE] = {0}; // Buffer para armazenar as últimas leituras do sensor direito
int bufferIndex = 0;                // Índice circular para o buffer
int leAntLeft = 0, leAntRight = 0;
int QtQuadradoLeft = 0, QtQuadradoRight = 0;
bool encruzilhada = false; // Flag para indicar se encontrou encruzilhada
bool deveVirar = false; // Flag para indicar se deve virar na próxima encruzilhada
bool ladoVirar = 0;
////////////////////////////////////////////////////////////////////////////////////
int Estado = 0;

void Leitura() {
  for (int i = 0; i < QTSensores; i++) {
      // Configura os pinos do MUX com base nos bits
      Mandar_Mux_Bin[0] = (i & 0x01); // LSB
      Mandar_Mux_Bin[1] = (i & 0x02) >> 1; // Bit 1
      Mandar_Mux_Bin[2] = (i & 0x04) >> 2; // Bit 2
      Mandar_Mux_Bin[3] = (i & 0x08) >> 3; // MSB

      for (int j = 0; j < 4; j++) {
          digitalWrite(MUX_S[j], Mandar_Mux_Bin[j]); // Usando o array MUX_S
      }

      // Lê o valor do sensor através do MUX
      int leituraAtual = analogRead(MUX_SIG);

      // Atualiza o histórico de leituras do sensor 'i'
      HistoricoLeituras[i][IndiceLeitura] = leituraAtual;

      // Calcula a média das 5 últimas leituras
      int soma = 0;
      for (int k = 0; k < NumLeituras; k++) {
          soma += HistoricoLeituras[i][k];
      }
      Sensor[i] = soma / NumLeituras; // Armazena a média no vetor Sensor
  }

  // Atualiza o índice de controle para o histórico de leituras (circular)
  IndiceLeitura = (IndiceLeitura + 1) % NumLeituras;

  // Impressão de dados para depuração
  ImprimirSensores(1);

  // Funções auxiliares para processamento de dados
  Discretiza();
}
void ImprimirSensores(int Antropofagico) {
  if (Antropofagico == 1){
    for (int i = 0; i < QTSensores; i++) {
        Serial.print(Sensor[i]);
        if (i < QTSensores - 1) {
            Serial.print("| "); // Adiciona vírgula entre os sensores
        }
    }
  }
  else if (Antropofagico == 2){
    for (int i = 0; i < QTSensores; i++) {
      Serial.print(SensorBIN[i]);
      if (i < QTSensores - 1) {
          Serial.print("| "); // Adiciona vírgula entre os sensores
      }
    }
  }
  //Serial.print(" | Erro: ");
  //Serial.println(erro); // Imprime o valor do erro
  Serial.println();
}

void Discretiza() {
  for (int i = 0; i < QTSensores; i++) {
      // Discretiza o valor com base no valor de corte
      if (Sensor[i] > corte[i]) {
          SensorBIN[i] = true; // Estado ALTO
      } else {
          SensorBIN[i] = false; // Estado BAIXO
      }
  }
}

void CalculaErro() { // Negativo = mais para a esquerda, positivo = mais para a direita
  // Quando o sensor central (SensorBIN[5]) detecta branco
  if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == PRETO) &&
      (SensorBIN[4] == PRETO) && (SensorBIN[5] == BRANCO) && (SensorBIN[6] == PRETO) && 
      (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO)) {
      erro = 0;
  
  } else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == PRETO) &&
             (SensorBIN[4] == BRANCO) && (SensorBIN[5] == BRANCO) && (SensorBIN[6] == PRETO) &&
             (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO)) {
      erro = -0.5;
  } else if ( (SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == PRETO) &&
             (SensorBIN[4] == BRANCO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) &&
             (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO) ) {
      erro = -1;
  }else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == BRANCO) &&
             (SensorBIN[4] == BRANCO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) &&
             (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO)) {
      erro = -1.5;
  }else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == BRANCO) &&
             (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) &&
             (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO)) {
      erro = -2;
  }else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == BRANCO) && (SensorBIN[3] == BRANCO) &&
             (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) &&
             (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO)) {
      erro = -2.5;
  }else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == BRANCO) && (SensorBIN[3] == PRETO) &&
             (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) &&
             (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO)) {
      erro = -3;
  }else if ((SensorBIN[1] == BRANCO) && (SensorBIN[2] == BRANCO) && (SensorBIN[3] == PRETO) &&
             (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) &&
             (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO)) {
      erro = -3.5;
  }else if ((SensorBIN[1] == BRANCO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == PRETO) &&
             (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) &&
             (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO)) {
      erro = -4;
  } else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == PRETO) &&
             (SensorBIN[4] == PRETO) && (SensorBIN[5] == BRANCO) && (SensorBIN[6] == BRANCO) &&
             (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO)) {
      erro = 0.5;
  } else if ( (SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == PRETO) &&
             (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == BRANCO) &&
             (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO) ) {
      erro = 1;
  }else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == PRETO) &&
             (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == BRANCO) &&
             (SensorBIN[7] == BRANCO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO)) {
      erro = 1.5;
  }else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == PRETO) &&
             (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) &&
             (SensorBIN[7] == BRANCO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO)) {
      erro = 2;
  }else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == PRETO) &&
             (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) &&
             (SensorBIN[7] == BRANCO) && (SensorBIN[8] == BRANCO) && (SensorBIN[9] == PRETO)) {
      erro = 2.5;
  }else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == PRETO) &&
             (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) &&
             (SensorBIN[7] == PRETO) && (SensorBIN[8] == BRANCO) && (SensorBIN[9] == PRETO)) {
      erro = 3;
  }else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == PRETO) &&
             (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) &&
             (SensorBIN[7] == PRETO) && (SensorBIN[8] == BRANCO) && (SensorBIN[9] == BRANCO)) {
      erro = 3.5;
  }else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == PRETO) &&
             (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) &&
             (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == BRANCO)) {
      erro = 4;
  // Caso nenhum dos padrões seja detectado, manter erro anterior
  } else {
      erro = erroA; // Assume que erroA é uma variável definida anteriormente
  }
}


void CalculaPID() {
  P = erro * Kp;
  I = I + erro;
  D = erro - erroA;
  ////////// anti windupp //////////
  if (erro == 0) { I = 0; }
  if ((erro > 0 && erroA < 0) || (erro < 0 && erroA >= 0)) {
      I = 0; // Zera a parte integrativa quando o sinal do erro muda
  }
  /////////////////////////////////
  PID = P + (Ki * I) + (Kd * D);
  erroA = erro;
}

void AutoTunePID() {
  if (autoTuningEnabled && (millis() - lastTuneTime > tuneInterval)) {
      // Ajusta Kp, Ki, Kd com base na resposta do sistema
      if (erro > 0) {
          Kp += 0.1; // Aumenta Kp se o erro for positivo
      } else {
          Kp -= 0.1; // Diminui Kp se o erro for negativo
      }

      Ki += 0.01; // Aumenta Ki
      Kd += 0.001; // Aumenta Kd

      // Limita os valores dos parâmetros para evitar crescimento excessivo
      Kp = constrain(Kp, 0, 10);
      Ki = constrain(Ki, 0, 1);
      Kd = constrain(Kd, 0, 1);

      lastTuneTime = millis(); // Atualiza o tempo da última modificação
  }
}

void Seguir() {
    CalculaErro();
    CalculaPID();
    AutoTunePID(); // Chama o auto-tuning

    if (PID < -MAXR) { PID = -MAXR; }
    if (PID > MAXR) { PID = MAXR; }
    
    if (PID > 0) { // Direita
        VeloE = PWME;
        VeloD = PWMD - PID;
    } else { // Esquerda
        VeloE = PWME - abs(PID);
        VeloD = PWMD;
    }
    
    if (VeloD < 0) { VeloD = 0; }
    if (VeloE < 0) { VeloE = 0; }

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, VeloE);
    
    // Motor_D
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, VeloD);
}

void Calibracao() {
  const unsigned long tempoCalibracao = 5000; // Tempo total de calibração em milissegundos
  unsigned long tempoInicial = millis(); // Captura o tempo inicial

  // Arrays para armazenar os 5 maiores e 5 menores valores
  int maiores[5] = {0}; // Inicializa com 0
  int menores[5] = {1023}; // Inicializa com um valor alto

  // Realiza as leituras por 5 segundos
  while (millis() - tempoInicial < tempoCalibracao) {
    for (int sensorIndex = 0; sensorIndex < QTSensores; sensorIndex++) {
        // Configura os pinos do MUX para o sensor atual
        Mandar_Mux_Bin[0] = (sensorIndex & 0x01); // LSB
        Mandar_Mux_Bin[1] = (sensorIndex & 0x02) >> 1; // Bit 1
        Mandar_Mux_Bin[2] = (sensorIndex & 0x04) >> 2; // Bit 2
        Mandar_Mux_Bin[3] = (sensorIndex & 0x08) >> 3; // MSB
        
        for (int j = 0; j < 4; j++) {
            digitalWrite(MUX_S[j], Mandar_Mux_Bin[j]); // Configura pinos do MUX
        }

        // Lê o valor do sensor
        int valorLido = analogRead(MUX_SIG);

        // Atualiza os 5 maiores
        if (valorLido > menores[0]) {
            menores[0] = valorLido;
            // Bubble sort simples
            for (int k = 0; k < 5 - 1; k++) {
                if (menores[k] < menores[k + 1]) {
                    int aux = menores[k];
                    menores[k] = menores[k + 1];
                    menores[k + 1] = aux;
                }
            }
        }

        // Atualiza os 5 menores
        if (valorLido < maiores[4]) {
            maiores[4] = valorLido;
            // Bubble sort simples
            for (int k = 4; k > 0; k--) {
                if (maiores[k] > maiores[k - 1]) {
                    int aux = maiores[k];
                    maiores[k] = maiores[k - 1];
                    maiores[k - 1] = aux;
                }
            }
        }
    }
    delay(5); // Atraso entre as leituras
  }

  // Calcula a média dos 5 maiores
  int somaMaiores = 0;
  for (int i = 0; i < 5; i++) {
      somaMaiores += maiores[i];
  }
  float mediaMaiores = somaMaiores / 5.0;

  // Calcula a média dos 5 menores
  int somaMenores = 0;
  for (int i = 0; i < 5; i++) {
      somaMenores += menores[i];
  }
  float mediaMenores = somaMenores / 5.0;

  // Calcula o valor de corte para cada sensor
  for (int sensorIndex = 0; sensorIndex < QTSensores; sensorIndex++) {
      corte[sensorIndex] = (mediaMaiores + mediaMenores) / 2;
  }

  // Imprime os resultados de corte
  Serial.print("Corte de cada sensor: ");
  for (i = 0; i < QTSensores; i++) {
      Serial.print(String(corte[i]) + "|");
  }
  Serial.println();
}
void LeituraQuadrados() {
    // Adiciona as leituras atuais aos buffers circulares
    bufferLeft[bufferIndex] = SensorBIN[0];   // Leitura do sensor esquerdo
    bufferRight[bufferIndex] = SensorBIN[10]; // Leitura do sensor direito
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE; // Atualiza o índice do buffer (circular)

    // Calcula a média das últimas 5 leituras para cada sensor
    int mediaLeft = 0;
    int mediaRight = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        mediaLeft += bufferLeft[i];
        mediaRight += bufferRight[i];
    }
    mediaLeft /= BUFFER_SIZE;
    mediaRight /= BUFFER_SIZE;

    // Verifica mudanças baseadas nas médias calculadas
    if (leAntLeft != mediaLeft) {
        if ((mediaLeft == BRANCO) && (SensorBIN[10] == PRETO) && (SensorBIN[9] == PRETO)) {
            QtQuadradoLeft += 1; // Contabiliza quadrado na esquerda
        }
    }
    
    if (leAntRight != mediaRight) {
        if ((mediaRight == BRANCO) && (SensorBIN[0] == PRETO) && (SensorBIN[1] == PRETO)) {
            QtQuadradoRight += 1; // Contabiliza quadrado na direita
        }
    }

    // Atualiza as leituras anteriores
    leAntLeft = mediaLeft;
    leAntRight = mediaRight;
}
bool DetectarEncruzilhada() {
  // Condição para detectar uma encruzilhada (exemplo de encruzilhada em "T" ou "X"):
  // Se o sensor central (frente) e os sensores laterais (esquerda e direita) estão detectando a linha preta simultaneamente,
  // assumimos que estamos em uma encruzilhada.
  
  if (SensorBIN[0] == PRETO && SensorBIN[6] == PRETO && SensorBIN[10] == PRETO) {
    // Detecção de uma encruzilhada completa (cruzamento em X ou em T)
    return true;
  }
  // Encruzilhada em T 
  else if (SensorBIN[0] == PRETO && SensorBIN[3] == PRETO) {
    // Detecta uma encruzilhada em "T" à esquerda
    return true;
  } 
  else if (SensorBIN[10] == PRETO && SensorBIN[8] == PRETO) {
    // Detecta uma encruzilhada em "T" à direita
    return true;
  }
  
  return false; // Não encontrou encruzilhada
}
void Virar(bool LadoVira){
  if (LadoVira == 0 ){
    digitalWrite(IN1, LOW); // VIRA PARA A ESQUERDA
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, PWME);
    
    // Motor_D
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, PWMD);
    delay(500);
    digitalWrite(IN1, HIGH); // Anda um tiquin reto
    digitalWrite(IN2, LOW);
    analogWrite(ENA, PWME);
    
    // Motor_D
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, PWMD);
    delay(100);
  } else{
    digitalWrite(IN1, HIGH); // VIRA PARA A DIREITA
    digitalWrite(IN2, LOW);
    analogWrite(ENA, PWME);
    
    // Motor_D
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, PWMD);
    delay(500);

    digitalWrite(IN1, HIGH); // Anda um tiquin reto
    digitalWrite(IN2, LOW);
    analogWrite(ENA, PWME);
    
    // Motor_D
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, PWMD);
    delay(100);
  }
}
void setup() {
  // Sensores
  pinMode(MUX_SIG, INPUT);
  for (i = 0; i < 4; i++) {
      pinMode(MUX_S[i], OUTPUT);
  }
  // Motores
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(BotCalibra, INPUT);
  pinMode(BotStart, INPUT);
  Serial.begin(9600);
  
  // Aguarda pressionar o botão de calibração
  Serial.println("Pressione o botão de calibração...");
  while (digitalRead(BotCalibra) == LOW) {
      // Aguarda até que o botão seja pressionado
  }
  Serial.println("Calibrando sensores...!");
  
  // Chama a função de calibração
  Calibracao();

  // Aguarda pressionar o botão de iniciar
  Serial.println("Pressione o botão para iniciar...");
  while (digitalRead(BotStart) == LOW) {
      // Aguarda até que o botão seja pressionado
  }
  Serial.println("3!...");
  delay(1000);
  Serial.println("2!...");
  delay(1000);
  Serial.println("1!...");
  delay(1000);
  Serial.println("======= avua fi!======");
}

void loop() {
  Leitura();
  LeituraQuadrados();

  if (DetectarEncruzilhada()) {
    encruzilhada = true; // Marca que uma encruzilhada foi encontrada
  }

  if (encruzilhada) {
    // Se o robô encontrou uma encruzilhada
    if (deveVirar) {
      // Se a flag "deveVirar" está ativa, o robô deve virar agora
      Virar (ladoVirar); // 0 para esquerda, e 1 para direita
      deveVirar = false; // Resetamos a flag após a virada
    } else {
      // Se há quadrados para processar à esquerda
      if (QtQuadradoLeft > 0) {
        QtQuadradoLeft--;
        ladoVirar = 0; // Sinaliza que o robô deve virar para a esquerda
      }

      // Se há quadrados para processar à direita
      if (QtQuadradoRight > 0) {
        QtQuadradoRight--;
        ladoVirar = 1; // Sinaliza que o robô deve virar para a direita
      }

      if (QtQuadradoLeft == 0 && QtQuadradoRight == 0) {
        // Se o contador de quadrados chegou a zero para ambos os lados,
        // sinaliza que o robô deve virar na próxima encruzilhada
        deveVirar = true;
      }
    }

    encruzilhada = false; // Reseta a flag de encruzilhada
  } else {
    // Continua seguindo a linha normalmente se não houver encruzilhada
    Seguir();
  }
}
