#include "Constante.c"

int Sensor[QTSensores] = {0}; // Inicializa zerando tudo
bool SensorBIN[QTSensores] = {1};
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
/////////////////////////////////////////////////////////////////////////////////////
int i = 0, j = 0;
int Antropofagico = 2;
void Leitura() {
  // O loop continua lendo os canais do MUX na ordem normal (0, 1, 2, ...)
  for (int i = 0; i < QTSensores; i++) {
    // --- Configuração do MUX (não muda) ---
    Mandar_Mux_Bin[0] = (i & 0x01); // LSB
    Mandar_Mux_Bin[1] = (i & 0x02) >> 1; // Bit 1
    Mandar_Mux_Bin[2] = (i & 0x04) >> 2; // Bit 2
    Mandar_Mux_Bin[3] = (i & 0x08) >> 3; // MSB

    for (int j = 0; j < 4; j++) {
      digitalWrite(MUX_S[j], Mandar_Mux_Bin[j]);
    }

    // Lê o valor do sensor atual através do MUX (não muda)
    int leituraAtual = analogRead(MUX_SIG);

    // --- Armazenamento Invertido ---
    // Calcula o índice de destino invertido
    int indiceInvertido = (QTSensores - 1) - i; // << MUDANÇA PRINCIPAL AQUI

    // Atualiza o histórico de leituras na posição invertida
    HistoricoLeituras[indiceInvertido][IndiceLeitura] = leituraAtual; // << MUDANÇA AQUI

    // Calcula a média das 5 últimas leituras para o sensor na posição invertida
    long soma = 0; // Usar 'long' para a soma evita estouro (overflow)
    for (int k = 0; k < NumLeituras; k++) {
      soma += HistoricoLeituras[indiceInvertido][k]; // << MUDANÇA AQUI
    }
    // Armazena a média no vetor Sensor na posição invertida
    Sensor[indiceInvertido] = soma / NumLeituras; // << MUDANÇA AQUI
  }

  // O resto da função permanece igual
  // Atualiza o índice de controle para o histórico de leituras (circular)
  IndiceLeitura = (IndiceLeitura + 1) % NumLeituras;

  // Impressão de dados para depuração
  ImprimirSensores(Antropofagico);

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
    
    // --- LÓGICA DE CONTROLE DO MOTOR ATUALIZADA ---

    if((SensorBIN[0] == PRETO) && (SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) &&
       (SensorBIN[3] == PRETO) && (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && 
       (SensorBIN[6] == PRETO) && (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && 
       (SensorBIN[9] == PRETO )&& (SensorBIN[10] == BRANCO)) {
        
        // Curva fechada para a esquerda (Motor Esquerdo para frente, Direito para trás)
        digitalWrite(dirMotorE, LOW);  // Motor Esquerdo FRENTE
        digitalWrite(dirMotorD, HIGH); // Motor Direito TRÁS
        analogWrite(pwmMotorE, PWME);
        analogWrite(pwmMotorD, PWMD);

    } else if ((SensorBIN[0] == BRANCO) && (SensorBIN[1] == PRETO) && (SensorBIN[2] == PRETO) &&
       (SensorBIN[3] == PRETO) && (SensorBIN[4] == PRETO) && (SensorBIN[5] == PRETO) && 
       (SensorBIN[6] == PRETO) && (SensorBIN[7] == PRETO) && (SensorBIN[8] == PRETO) && 
       (SensorBIN[9] == PRETO )&& (SensorBIN[10] == PRETO)) {
        
        // Curva fechada para a direita (Motor Esquerdo para trás, Direito para frente)
        digitalWrite(dirMotorE, HIGH); // Motor Esquerdo TRÁS
        digitalWrite(dirMotorD, LOW);  // Motor Direito FRENTE
        analogWrite(pwmMotorE, PWME);
        analogWrite(pwmMotorD, PWMD);

    } else {
        // Seguir a linha (ambos os motores para frente com correção do PID)
        digitalWrite(dirMotorE, LOW); // Motor Esquerdo FRENTE
        digitalWrite(dirMotorD, LOW); // Motor Direito FRENTE
        analogWrite(pwmMotorE, VeloE);
        analogWrite(pwmMotorD, VeloD);
    }
}

void Calibracao() {
    const unsigned long tempoCalibracao = 5000;
    unsigned long tempoInicial = millis();
    const unsigned long IntervaloTempoBUZZ = 1000;
    const int QtLeituras = 20;
    
    // Arrays para armazenar os maiores e menores valores de cada sensor
    int maiores[QTSensores][QtLeituras] = {0};
    int menores[QTSensores][QtLeituras];

    // A inicialização pode ser mantida, pois preenche o array todo
    for (int i = 0; i < QTSensores; i++) {
        for (int j = 0; j < QtLeituras; j++) {
            menores[i][j] = 1023; // Inicializa com valor máximo
        }
    }

    // Realiza as leituras durante o tempo de calibração
    while (millis() - tempoInicial < tempoCalibracao) {
        if (millis() - CalibraInterval >= IntervaloTempoBUZZ) {
            // tone(BUZZ, 20, 300);
            CalibraInterval = millis();
        }

        // O loop varre os canais do MUX na ordem física (0, 1, 2...)
        for (int sensorIndex = 0; sensorIndex < QTSensores; sensorIndex++) {
            // Configura os pinos do MUX para o sensor atual (não muda)
            Mandar_Mux_Bin[0] = (sensorIndex & 0x01);
            Mandar_Mux_Bin[1] = (sensorIndex & 0x02) >> 1;
            Mandar_Mux_Bin[2] = (sensorIndex & 0x04) >> 2;
            Mandar_Mux_Bin[3] = (sensorIndex & 0x08) >> 3;
            
            for (int j = 0; j < 4; j++) {
                digitalWrite(MUX_S[j], Mandar_Mux_Bin[j]);
            }

            // Lê o valor do sensor (não muda)
            int valorLido = analogRead(MUX_SIG);

            // --- APLICA A INVERSÃO AQUI ---
            // Calcula o índice de destino invertido
            int indiceInvertido = (QTSensores - 1) - sensorIndex; // << MUDANÇA PRINCIPAL

            Serial.print("|" + String(maiores[indiceInvertido][0])); // << MUDANÇA AQUI

            // Atualiza os menores valores no índice invertido
            if (valorLido < menores[indiceInvertido][0]) { // << MUDANÇA AQUI
                menores[indiceInvertido][0] = valorLido; // << MUDANÇA AQUI
                // Bubble sort simples
                for (int k = 0; k < QtLeituras - 1; k++) {
                    if (menores[indiceInvertido][k] < menores[indiceInvertido][k + 1]) { // << MUDANÇA AQUI
                        int aux = menores[indiceInvertido][k];
                        menores[indiceInvertido][k] = menores[indiceInvertido][k + 1];
                        menores[indiceInvertido][k + 1] = aux;
                    }
                }
            }

            // Atualiza os maiores valores no índice invertido
            if (valorLido > maiores[indiceInvertido][QtLeituras - 1]) { // << MUDANÇA AQUI
                maiores[indiceInvertido][QtLeituras - 1] = valorLido; // << MUDANÇA AQUI
                // Bubble sort simples
                for (int k = QtLeituras - 1; k > 0; k--) {
                    if (maiores[indiceInvertido][k] > maiores[indiceInvertido][k - 1]) { // << MUDANÇA AQUI
                        int aux = maiores[indiceInvertido][k];
                        maiores[indiceInvertido][k] = maiores[indiceInvertido][k - 1];
                        maiores[indiceInvertido][k - 1] = aux;
                    }
                }
            }
        }
        Serial.println();
    }

    // Calcula a mediana e o corte para cada sensor, usando a lógica invertida
    for (int sensorIndex = 0; sensorIndex < QTSensores; sensorIndex++) {
        // Calcula o índice invertido novamente para garantir a correspondência
        int indiceInvertido = (QTSensores - 1) - sensorIndex; // << MUDANÇA PRINCIPAL

        // Acessa os dados já armazenados na ordem invertida
        float medianaMaiores = calcularMediana(maiores[indiceInvertido], QtLeituras); // << MUDANÇA AQUI
        float medianaMenores = calcularMediana(menores[indiceInvertido], QtLeituras); // << MUDANÇA AQUI

        // Armazena o valor de corte final na posição invertida
        corte[indiceInvertido] = (medianaMaiores + medianaMenores) / 2; // << MUDANÇA AQUI

        // Imprime os resultados. Note que usamos 'indiceInvertido' para o log
        // ser consistente com a ordem do array (Sensor 0, Sensor 1, etc.).
        Serial.print("Sensor " + String(indiceInvertido) + " - Mediana Maiores: " + String(medianaMenores) + ", Mediana Menores: " + String(medianaMaiores) + ", Corte: " + String(corte[indiceInvertido]) + "\n");
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
  pinMode(pwmMotorE, OUTPUT);
  pinMode(dirMotorE, OUTPUT);
  pinMode(pwmMotorD, OUTPUT);
  pinMode(dirMotorD, OUTPUT);

  Serial.begin(9600);
  
  //pinMode(BotCalibra, INPUT);
  //pinMode(BotStart, INPUT);
  //pinMode(BUZZ, OUTPUT);
  // Aguarda pressionar o botão de calibração
  
  Serial.println("Calibrando sensores...!");

  // Chama a função de calibração
  Calibracao();
  Serial.println("======= avua fi!======");
  //digitalWrite(6,HIGH);
}

void loop() {

  Leitura();
  //Seguir(); // Estado padrão ele segue a linha
}
