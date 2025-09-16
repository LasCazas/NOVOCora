#include "Constante.c"

// ----------------------- ARRAYS / ESTADO -----------------------
int Sensor[QTSensoresReal] = {0}; // valores médios
bool SensorBIN[QTSensoresReal];   // discretizados
int HistoricoLeituras[QTSensoresReal][NumLeituras];  // últimas leituras por sensor
int IndiceLeitura = 0;

bool Mandar_Mux_Bin[4] = {0};

// CORREÇÃO: corte tem que ter tamanho lógico (QTSensoresReal)
int corte[QTSensoresReal] = {0};

// Guarda min e max por sensor lógico (simplificado)
int menores[QTSensoresReal];
int maiores[QTSensoresReal];

// ----------------------- MODO / PID -----------------------
const int MODO_PRODUCAO = 1; // 0 = Debug (Serial ON), 1 = Produção (Serial OFF)
const int SEM_MOTOR = !MODO_PRODUCAO;

int P = 0, I = 0, D = 0, PID = 0;
float erro = 0, erroA = 0;
int VeloE = 0, VeloD = 0;

unsigned long CalibraInterval = 0;

// PID
float Kp = 15, Ki = 0.08, Kd = 3;
float targetValue = 0;
bool autoTuningEnabled = false;
unsigned long lastTuneTime = 0;
const unsigned long tuneInterval = 1000;

// misc
int i = 0, j = 0;
int Antropofagico = 2;

unsigned long tempoPerdaLinha = 0;
const unsigned long tempoLimiteParada = 850; 
bool roboParado = false;

// ----------------------- FUNÇÕES -----------------------

void Leitura() {
  int indiceReal = 0;

  for (int i = 0; i < QTSensores; i++) {
    // Ignora canais físicos configurados (como antes)
    if (i == 1 || i == 9) {
      continue;
    }

    // Configura bits do MUX
    Mandar_Mux_Bin[0] = (i & 0x01);
    Mandar_Mux_Bin[1] = (i & 0x02) >> 1;
    Mandar_Mux_Bin[2] = (i & 0x04) >> 2;
    Mandar_Mux_Bin[3] = (i & 0x08) >> 3;

    for (int j = 0; j < 4; j++) {
      digitalWrite(MUX_S[j], Mandar_Mux_Bin[j]);
    }

    // Pequeno tempo para estabilizar o MUX/Entrada analógica
    delayMicroseconds(20);
    int leituraAtual = analogRead(MUX_SIG);

    // Mapeamento invertido - primeiro lido -> última posição lógica
    int indiceDestino = (QTSensoresReal - 1) - indiceReal;

    // Segurança: só escreve se dentro dos limites lógicos
    if (indiceDestino >= 0 && indiceDestino < QTSensoresReal) {
      HistoricoLeituras[indiceDestino][IndiceLeitura] = leituraAtual;

      long soma = 0;
      for (int k = 0; k < NumLeituras; k++) {
        soma += HistoricoLeituras[indiceDestino][k];
      }
      Sensor[indiceDestino] = soma / NumLeituras;
    } else {
      // se algo errado, evita overflow e ignora
    }

    indiceReal++;
    if (indiceReal >= QTSensoresReal) break; // segurança extra
  }

  IndiceLeitura = (IndiceLeitura + 1) % NumLeituras;
  ImprimirSensores(Antropofagico);
  Discretiza();
}

void ImprimirSensores(int Antropofagico) {
  if (MODO_PRODUCAO == 0) {
    if (Antropofagico == 1) {
      for (int i = 0; i < QTSensoresReal; i++) {
        Serial.print(Sensor[i]);
        if (i < QTSensoresReal - 1) Serial.print("| ");
      }
    } else if (Antropofagico == 2) {
      for (int i = 0; i < QTSensoresReal; i++) {
        Serial.print(SensorBIN[i]);
        if (i < QTSensoresReal - 1) Serial.print("| ");
      }
    }

    if (Antropofagico != 0) {
      Serial.print(" | VeloE: " + String(VeloE) + " | VeloD: " + String(VeloD) + " | ");
      Serial.println(erro);
    }
  }
}

void Discretiza() {
  for (int i = 0; i < QTSensoresReal; i++) {
    // Se não calibrado, usa meio-range como fallback
    int corteLocal = (corte[i] > 0) ? corte[i] : 512;
    if (Sensor[i] > corteLocal) {
      SensorBIN[i] = BRANCO;
    } else {
      SensorBIN[i] = PRETO;
    }
  }
}

void CalculaErro() {
  long soma = 0;
  int ativos = 0;

  for (int i = 0; i < QTSensoresReal; i++) {
    if (SensorBIN[i] == BRANCO) {
      soma += i * 1000;
      ativos++;
    }
  }

  // determina centro lógico com checagem de limites
  int centroIndex = SENSOR_CENTRAL;
  if (centroIndex < 0 || centroIndex >= QTSensoresReal) centroIndex = QTSensoresReal / 2;

  if (ativos > 0) {
    float posicaoMedia = (float)soma / ativos;
    erro = (posicaoMedia - (centroIndex * 1000)) / 1000.0;
  } else {
    erro = erroA; // mantém erro anterior
  }
}

void CalculaPID() {
  P = erro * Kp;
  I = I + erro;
  D = erro - erroA;

  AntiWindUp();

  PID = P + (Ki * I) + (Kd * D);
  erroA = erro;
}

void AntiWindUp() {
  if (erro == 0) I = 0;
  if ((erro > 0 && erroA < 0) || (erro < 0 && erroA > 0)) I = 0;
}

void AutoTunePID() {
  if (autoTuningEnabled && (millis() - lastTuneTime > tuneInterval)) {
    if (erro > 0) Kp += 0.1;
    else Kp -= 0.1;

    Ki += 0.01;
    Kd += 0.001;

    // limites (ajuste conforme quiser)
    Kp = constrain(Kp, 0, 50);
    Ki = constrain(Ki, 0, 5);
    Kd = constrain(Kd, 0, 5);

    lastTuneTime = millis();
  }
}

void Seguir() {
  CalculaErro();
  CalculaPID();
  AutoTunePID();

  // --- Limita saída PID ---
  PID = constrain(PID, -MAXR, MAXR);

  // --- Calcula velocidades ---
  if (PID > 0) {
    // Correção para direita
    VeloE = PWME + PID;
    VeloD = PWMD - PID;
  } else {
    // Correção para esquerda
    VeloE = PWME + PID; // PID é negativo
    VeloD = PWMD - PID; // subtrair negativo = somar
  }

  // --- Limita velocidades ---
  VeloE = constrain(VeloE, 0, MAXR);
  VeloD = constrain(VeloD, 0, MAXR);

  // --- Controle dos motores ---
  if (!SEM_MOTOR) {
    if (VeloE >= MAXR && VeloD <= 0) {
      // Curva fechada esquerda
      digitalWrite(dirMotorE, LOW);  // Motor E frente
      digitalWrite(dirMotorD, HIGH); // Motor D trás
      analogWrite(pwmMotorE, VeloE);
      analogWrite(pwmMotorD, VeloE);
    } else if (VeloD >= MAXR && VeloE <= 0) {
      // Curva fechada direita
      digitalWrite(dirMotorE, HIGH); // Motor E trás
      digitalWrite(dirMotorD, LOW);  // Motor D frente
      analogWrite(pwmMotorE, VeloD);
      analogWrite(pwmMotorD, VeloD);
    } else {
      // Movimento normal (ambos frente)
      digitalWrite(dirMotorE, LOW);
      digitalWrite(dirMotorD, LOW);
      analogWrite(pwmMotorE, VeloE);
      analogWrite(pwmMotorD, VeloD);
    }
  }
}
void Calibracao() {
  const unsigned long tempoCalibracao = 5000;
  unsigned long tempoInicial = millis();

  // CORREÇÃO: arrays de calibração no tamanho lógico
  int calibMin[QTSensoresReal];
  int calibMax[QTSensoresReal];

  for (int i = 0; i < QTSensoresReal; i++) {
    calibMin[i] = 1023;
    calibMax[i] = 0;
  }

  if (MODO_PRODUCAO == 0) {
    Serial.println("Iniciando calibracao...");
  }

  while (millis() - tempoInicial < tempoCalibracao) {
    int indiceReal = 0;
    for (int sensorIndex = 0; sensorIndex < QTSensores; sensorIndex++) {
      if (sensorIndex == 1 || sensorIndex == 9) continue;

      Mandar_Mux_Bin[0] = (sensorIndex & 0x01);
      Mandar_Mux_Bin[1] = (sensorIndex & 0x02) >> 1;
      Mandar_Mux_Bin[2] = (sensorIndex & 0x04) >> 2;
      Mandar_Mux_Bin[3] = (sensorIndex & 0x08) >> 3;
      for (int j = 0; j < 4; j++) digitalWrite(MUX_S[j], Mandar_Mux_Bin[j]);

      delayMicroseconds(20);
      int valorLido = analogRead(MUX_SIG);

      int indiceDestino = (QTSensoresReal - 1) - indiceReal;
      if (indiceDestino >= 0 && indiceDestino < QTSensoresReal) {
        if (valorLido < calibMin[indiceDestino]) calibMin[indiceDestino] = valorLido;
        if (valorLido > calibMax[indiceDestino]) calibMax[indiceDestino] = valorLido;
      }

      indiceReal++;
      if (indiceReal >= QTSensoresReal) break;
    }
    yield();
  }

  if (MODO_PRODUCAO == 0) Serial.println("\nCalibracao finalizada!");

  // Calcula cortes e salva min/max
  for (int i = 0; i < QTSensoresReal; i++) {
    corte[i] = (calibMax[i] + calibMin[i]) / 2;
    menores[i] = calibMin[i];
    maiores[i] = calibMax[i];
    if (MODO_PRODUCAO == 0) {
      Serial.print("Sensor LOGICO " + String(i));
      Serial.print(" | Min: " + String(calibMin[i]));
      Serial.print(" | Max: " + String(calibMax[i]));
      Serial.println(" | Corte: " + String(corte[i]));
    }
  }
}

void pararMotores() {
  analogWrite(pwmMotorE, 0);
  analogWrite(pwmMotorD, 0);
  digitalWrite(dirMotorE, LOW);
  digitalWrite(dirMotorD, LOW);
}

/**
 * @brief Verifica se todos os sensores estão no PRETO (perda total)
 */
bool verificaPerdaTotalLinha() {
  // determina centro lógico com checagem
  int centroIndex = SENSOR_CENTRAL;
  if (centroIndex < 0 || centroIndex >= QTSensoresReal) centroIndex = QTSensoresReal / 2;

  int inicioSensoresCentrais = centroIndex - 2;
  int fimSensoresCentrais = centroIndex + 2;
  if (inicioSensoresCentrais < 0) inicioSensoresCentrais = 0;
  if (fimSensoresCentrais >= QTSensoresReal) fimSensoresCentrais = QTSensoresReal - 1;

  // checa pontas (usando índices lógicos)
  if (SensorBIN[0] == BRANCO || SensorBIN[QTSensoresReal - 1] == BRANCO) {
    return false;
  }

  // checa bloco central
  for (int i = inicioSensoresCentrais; i <= fimSensoresCentrais; i++) {
    if (SensorBIN[i] == BRANCO) return false;
  }

  return true;
}

void setup() {
  pinMode(MUX_SIG, INPUT);
  for (int i = 0; i < 4; i++) pinMode(MUX_S[i], OUTPUT);

  pinMode(pwmMotorE, OUTPUT);
  pinMode(dirMotorE, OUTPUT);
  pinMode(pwmMotorD, OUTPUT);
  pinMode(dirMotorD, OUTPUT);

  // inicializa arrays
  for (int i = 0; i < QTSensoresReal; i++) {
    Sensor[i] = 0;
    HistoricoLeituras[i][0] = HistoricoLeituras[i][1] = 0;
    SensorBIN[i] = PRETO;
    corte[i] = 0;
    menores[i] = 0;
    maiores[i] = 0;
    for (int k = 0; k < NumLeituras; k++) HistoricoLeituras[i][k] = 512;
  }

  if (MODO_PRODUCAO == 0) {
    Serial.begin(115200);
    Serial.println("Modo Debug Ativado. Calibrando sensores...!");
  }

  Calibracao();

  if (MODO_PRODUCAO == 0) {
    Serial.println("======= avua fi!======");
  }
}

void loop() {
  Leitura();

  if (roboParado) {
    pararMotores();
    return;
  }

  if (MODO_PRODUCAO == 0) {
    Serial.println(verificaPerdaTotalLinha());
  }

  if (verificaPerdaTotalLinha()) {
    if (tempoPerdaLinha == 0) tempoPerdaLinha = millis();
  } else {
    tempoPerdaLinha = 0;
  }

  if (tempoPerdaLinha != 0 && (millis() - tempoPerdaLinha > tempoLimiteParada)) {
    roboParado = true;
    if (MODO_PRODUCAO == 0) Serial.println("Linha perdida por mais de tempo limite. PARANDO!");
  }

  if (!roboParado) {
    Seguir();
  }

  yield();
}
