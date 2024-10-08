#include "Constante.c"

int Sensor[QTSensores] = {0}; // Inicializa zerando tudo
bool SensorBIN[QTSensores] = {0};
int HistoricoLeituras[QTSensores][NumLeituras];  // Armazena as últimas 5 leituras de cada sensor
int IndiceLeitura = 0;                    // Índice de controle para o histórico de leituras

bool Mandar_Mux_Bin[4] = {0};
int corte[QTSensores] = {0};
int P = 0, I = 0, D = 0, PID = 0;
float erro = 0, erroA = 0;
int VeloE, VeloD;
unsigned long CalibraInterval = 0; // Tempo de inicia de calibracao
//////////////////////////////////////// PID ////////////////////////////////////////
float Kp = 10, Ki = 0.5, Kd = 2.0; // Parâmetros do PID
float targetValue = 0; // Valor alvo
bool autoTuningEnabled = false; // Habilitar/desabilitar auto-tuning
unsigned long lastTuneTime = 0; // Tempo da última atualização de tuning
const unsigned long tuneInterval = 1000; // Intervalo de tempo para ajuste
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
int i = 0, j = 0;
unsigned long startTime = 0;  // Tempo inicial do softstart
const unsigned long softstartDuration = 1000;  // Duração do softstart em milissegundos (exemplo: 3 segundos)
int VeloE_target = PWME;  // Velocidade final desejada para o motor esquerdo
int VeloD_target = PWMD;  // Velocidade final desejada para o motor direito

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
  //ImprimirSensores(3);

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
  } else if (Antropofagico == 2){
    for (int i = 0; i < QTSensores; i++) {
      Serial.print(SensorBIN[i]);
      if (i < QTSensores - 1) {
          Serial.print("| "); // Adiciona vírgula entre os sensores
      }
    }
  } else if (Antropofagico == 3){
    for (int i = 0; i < QTSensores; i++) {
      Serial.print(SensorBIN[i]);
      if (i < QTSensores - 1) {
          Serial.print("| "); // Adiciona vírgula entre os sensores
      }
    }
    Serial.print( " |Erro: "+String(erro)+ "|SquarL: "+ String(QtQuadradoLeft) + "  SquarR: "+  String(QtQuadradoRight) +  "  Encru: " + String(encruzilhada) );
  }

  Serial.print(erro); // Imprime o valor do erro
  //Serial.println();

  //Serial.print(" | VeloE: " + String(VeloE) + " | VeloD: " + String(VeloD));
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
  
  }else if ( ((SensorBIN[5] == BRANCO) || (SensorBIN[6] == BRANCO)) && 
            (SensorBIN[0] == BRANCO) && (SensorBIN[10] == PRETO)) {
      erro = 0;
  }else if (((SensorBIN[4] == BRANCO) || (SensorBIN[5] == BRANCO)) && 
            (SensorBIN[0] == PRETO) && (SensorBIN[10] == BRANCO)) {
      erro = 0;
  }else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == PRETO) &&
            (SensorBIN[4] == BRANCO) && (SensorBIN[5] == BRANCO) && (SensorBIN[6] == PRETO) &&
            (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO) &&
            (SensorBIN[0] == PRETO) && (SensorBIN[10] == PRETO)) {
      erro = -0.5;
  } else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == PRETO) &&
            (SensorBIN[4] == BRANCO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) &&
            (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO) &&
            (SensorBIN[0] == PRETO) && (SensorBIN[10] == PRETO)) {
      erro = -1;
  } else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == BRANCO) &&
            (SensorBIN[4] == BRANCO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) &&
            (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO) &&
            (SensorBIN[0] == PRETO) && (SensorBIN[10] == PRETO)) {
      erro = -1.5;
  } else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == BRANCO) &&
            (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) &&
            (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO) &&
            (SensorBIN[0] == PRETO) && (SensorBIN[10] == PRETO)) {
      erro = -2;
  } else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == BRANCO) && (SensorBIN[3] == BRANCO) &&
            (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) &&
            (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO) &&
            (SensorBIN[0] == PRETO) && (SensorBIN[10] == PRETO)) {
      erro = -2.5;
  } else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == BRANCO) && (SensorBIN[3] == PRETO) &&
            (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) &&
            (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO) &&
            (SensorBIN[0] == PRETO) && (SensorBIN[10] == PRETO)) {
      erro = -3;
  } else if ((SensorBIN[1] == BRANCO) && (SensorBIN[2] == BRANCO) && (SensorBIN[3] == PRETO) &&
            (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) &&
            (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO) &&
            (SensorBIN[0] == PRETO) && (SensorBIN[10] == PRETO)) {
      erro = -3.5;
  } else if ((SensorBIN[1] == BRANCO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == PRETO) && 
            (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) && 
            (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO) &&
            (SensorBIN[0] == PRETO) && (SensorBIN[10] == PRETO)) {
      erro = -4;
  } else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == PRETO) &&
            (SensorBIN[4] == PRETO) && (SensorBIN[5] == BRANCO) && (SensorBIN[6] == BRANCO) &&
            (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO) &&
            (SensorBIN[0] == PRETO) && (SensorBIN[10] == PRETO)) {
      erro = 0.5;
  } else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == PRETO) &&
            (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == BRANCO) &&
            (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO) &&
            (SensorBIN[0] == PRETO) && (SensorBIN[10] == PRETO)) {
      erro = 1;
  } else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == PRETO) &&
            (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == BRANCO) &&
            (SensorBIN[7] == BRANCO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO) &&
            (SensorBIN[0] == PRETO) && (SensorBIN[10] == PRETO)) {
      erro = 1.5;
  } else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == PRETO) &&
            (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) &&
            (SensorBIN[7] == BRANCO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == PRETO) &&
            (SensorBIN[0] == PRETO) && (SensorBIN[10] == PRETO)) {
      erro = 2;
  } else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == PRETO) &&
            (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) &&
            (SensorBIN[7] == BRANCO) && (SensorBIN[8] == BRANCO) && (SensorBIN[9] == PRETO) &&
            (SensorBIN[0] == PRETO) && (SensorBIN[10] == PRETO)) {
      erro = 2.5;
  } else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == PRETO) &&
            (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) &&
            (SensorBIN[7] == PRETO) && (SensorBIN[8] == BRANCO) && (SensorBIN[9] == PRETO) &&
            (SensorBIN[0] == PRETO) && (SensorBIN[10] == PRETO)) {
      erro = 3;
  } else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == PRETO) &&
            (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) &&
            (SensorBIN[7] == PRETO) && (SensorBIN[8] == BRANCO) && (SensorBIN[9] == BRANCO) &&
            (SensorBIN[0] == PRETO) && (SensorBIN[10] == PRETO)) {
      erro = 3.5;
  } else if ((SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) && (SensorBIN[3] == PRETO) &&
            (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && (SensorBIN[6] == PRETO) &&
            (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && (SensorBIN[9] == BRANCO) &&
            (SensorBIN[10] == PRETO)) {
      erro = 4;
  } else{
  // Caso nenhum dos padrões seja detectado, manter erro anterior else {
      erro = erroA; // Assume que erroA é uma variável definida anteriormente
  }
}

void CalculaPID() {
  P = erro * Kp;
  I = I + erro;
  D = erro - erroA;
  AntiWindUp(); // Limita a parte integrativa (anti-windup)
  PID = P + (Ki * I) + (Kd * D);
  erroA = erro;
}

void AntiWindUp() { 
  if (erro == 0) { I = 0; }
  if ((erro > 0 && erroA < 0) || (erro < 0 && erroA >= 0)) {
      I = 0; // Zera a parte integrativa quando o sinal do erro muda
  }
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
void SoftStart() {
    unsigned long elapsedTime = millis() - startTime;
    
    // Se o tempo de softstart ainda não terminou
    if (elapsedTime < softstartDuration) {
        // Calcula a fração de tempo passado em relação à duração total do softstart
        float progress = (float)elapsedTime / softstartDuration;
        
        // Calcula a velocidade atual com base no progresso
        VeloE = VeloE_target * progress;
        VeloD = VeloD_target * progress;
    } else {
        // Se o softstart terminou, fixa as velocidades no valor alvo
        VeloE = VeloE_target;
        VeloD = VeloD_target;
    }

    // Aplica as velocidades calculadas nos motores
    digitalWrite(IN1, HIGH); // Anda um tiquin reto
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
    const unsigned long IntervaloTempoBUZZ = 1000;
    const int QtLeituras = 20;
    // Arrays para armazenar os 5 maiores e 5 menores valores de cada sensor
    int maiores[QTSensores][QtLeituras] = {0}; // Inicializa com 0
    int menores[QTSensores][QtLeituras]; // Inicializa como não definidos
    for (int i = 0; i < QTSensores; i++) {
        for (int j = 0; j < QtLeituras; j++) {
            menores[i][j] = 1023; // Inicializa com um valor alto
        }
    }

    // Realiza as leituras por 5 segundos
    while (millis() - tempoInicial < tempoCalibracao) {
        if (millis() - CalibraInterval >= IntervaloTempoBUZZ ){
          tone(BUZZ,20,300); // PIN, FREQUENCIA, TEMPO DE DURACAO DO BARULHO
          CalibraInterval = millis();
        }
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
            Serial.print("|" + String(maiores[sensorIndex][0]));

            // Atualiza os 5 maiores
            if (valorLido < menores[sensorIndex][0]) {
                menores[sensorIndex][0] = valorLido;
                // Bubble sort simples
                for (int k = 0; k < QtLeituras - 1; k++) {
                    if (menores[sensorIndex][k] < menores[sensorIndex][k + 1]) {
                        int aux = menores[sensorIndex][k];
                        menores[sensorIndex][k] = menores[sensorIndex][k + 1];
                        menores[sensorIndex][k + 1] = aux;
                    }
                }
            }

            // Atualiza os 5 menores
            if (valorLido > maiores[sensorIndex][QtLeituras - 1]) {
                maiores[sensorIndex][QtLeituras - 1] = valorLido;
                // Bubble sort simples
                for (int k = QtLeituras - 1; k > 0; k--) {
                    if (maiores[sensorIndex][k] > maiores[sensorIndex][k - 1]) {
                        int aux = maiores[sensorIndex][k];
                        maiores[sensorIndex][k] = maiores[sensorIndex][k - 1];
                        maiores[sensorIndex][k - 1] = aux;
                    }
                }
            }
        }
        Serial.println();
    }

    // Calcula a mediana dos 5 maiores e 5 menores para cada sensor
    for (int sensorIndex = 0; sensorIndex < QTSensores; sensorIndex++) {
        float medianaMaiores = calcularMediana(maiores[sensorIndex], QtLeituras);
        float medianaMenores = calcularMediana(menores[sensorIndex], QtLeituras);

        // Calcula o valor de corte para o sensor atual
        corte[sensorIndex] = (medianaMaiores + medianaMenores) / 2 +AumentaCorte ;

        // Imprime os resultados de cada sensor
        Serial.print("Sensor " + String(sensorIndex) + " - Mediana Maiores: " + String(medianaMenores) + ", Mediana Menores: " + String(medianaMaiores) + ", Corte: " + String(corte[sensorIndex]) + "\n");
    }
}
float calcularMediana(int valores[], int tamanho) {
    // Ordena o array
    for (int i = 0; i < tamanho - 1; i++) {
        for (int j = 0; j < tamanho - i - 1; j++) {
            if (valores[j] > valores[j + 1]) {
                int temp = valores[j];
                valores[j] = valores[j + 1];
                valores[j + 1] = temp;
            }
        }
    }
    // Retorna a mediana
    if (tamanho % 2 == 0) {
        return (valores[tamanho / 2 - 1] + valores[tamanho / 2]) / 2.0;
    } else {
        return valores[tamanho / 2];
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
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY,HIGH);
  Serial.begin(115200);
  
  pinMode(BotCalibra, INPUT);
  pinMode(BotStart, INPUT);
  pinMode(BUZZ, OUTPUT);
  // Aguarda pressionar o botão de calibração
  Serial.println("Pressione o botão de calibração...");
  while (digitalRead(BotCalibra) == LOW) {
      // Aguarda até que o botão seja pressionado
  }
  
  Serial.println("Calibrando sensores...!");

  // Chama a função de calibração
  Calibracao();
  tone(BUZZ,523,100); // PIN, FREQUENCIA, TEMPO DE DURACAO DO BARULHO
  delay(200);
  tone(BUZZ,523,100); // PIN, FREQUENCIA, TEMPO DE DURACAO DO BARULHO

  // Aguarda pressionar o botão de iniciar
  Serial.println("Pressione o botão para iniciar...");
  while (digitalRead(BotCalibra) == LOW) {
      // Aguarda até que o botão seja pressionado
  }
  
  Serial.println("3!...");
  tone(BUZZ,523,100); // PIN, FREQUENCIA, TEMPO DE DURACAO DO BARULHO
  delay(1000);
  Serial.println("2!...");
  tone(BUZZ,523,100); // PIN, FREQUENCIA, TEMPO DE DURACAO DO BARULHO
  delay(1000);
  Serial.println("1!...");
  tone(BUZZ,523,100); // PIN, FREQUENCIA, TEMPO DE DURACAO DO BARULHO
  delay(1000);
  Serial.println("======= avua fi!======");
  
}

void Seguir(bool Sentido) {
    CalculaErro();
    CalculaPID();
    AutoTunePID(); // Chama o auto-tuning
    
    if (PID < -MAXR) { PID = -MAXR; }
    if (PID > MAXR) { PID = MAXR; }

    // Ajusta as velocidades com base no PID
    if (PID > 0) { // Direita
        VeloE_target = PWME;
        VeloD_target = PWMD - PID;
    } else { // Esquerda
        VeloE_target = PWME - abs(PID);
        VeloD_target = PWMD;
    }

    if (VeloD_target < 0) { VeloD_target = 0; }
    if (VeloE_target < 0) { VeloE_target = 0; }
    
    SoftStart(); // Chama o softstart para aplicar as velocidades gradualmente
}
