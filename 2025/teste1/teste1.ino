#include "Constante.c"

int Sensor[QTSensores] = {0}; // Inicializa zerando tudo
bool SensorBIN[QTSensores] = {1};
int HistoricoLeituras[QTSensores][NumLeituras];  // Armazena as últimas 5 leituras de cada sensor
int IndiceLeitura = 0;                    // Índice de controle para o histórico de leituras
const int MODO_PRODUCAO = 0; // 0 = Modo Debug (Serial ATIVO), 1 = Modo Produção (Serial DESATIVADO)
const int SEM_MOTOR = !MODO_PRODUCAO;

bool Mandar_Mux_Bin[4] = {0};
int corte[QTSensores] = {0};
// Arrays globais para guardar min e max de cada sensor
int menores[QTSensores][1];
int maiores[QTSensores][1];

int P = 0, I = 0, D = 0, PID = 0;
float erro = 0, erroA = 0;
int VeloE, VeloD;
unsigned long CalibraInterval = 0; // Tempo de inicia de calibracao
//////////////////////////////////////// PID ////////////////////////////////////////
float Kp = 16, Ki = 0.03, Kd = 3; // Parâmetros do PID
float targetValue = 0; // Valor alvo
bool autoTuningEnabled = false; // Habilitar/desabilitar auto-tuning
unsigned long lastTuneTime = 0; // Tempo da última atualização de tuning
const unsigned long tuneInterval = 1000; // Intervalo de tempo para ajuste
/////////////////////////////////////////////////////////////////////////////////////
int i = 0, j = 0;
int Antropofagico = 2;

// Variáveis para o sistema de quadrados e rotatória
int contadorQuadradosEsquerda = 0; // Contador de quadrados no sensor da esquerda
int contadorQuadradosDireita = 0;  // Contador de quadrados no sensor da direita
bool lastSensorEsquerda = false;    // Último estado do sensor da esquerda
bool lastSensorDireita = false;     // Último estado do sensor da direita
bool emModoRotatoria = false;       // Indica se está no modo rotatória
int contadorSaidas = 0;            // Contador de encruzilhadas a passar
const int sensorEsquerdaIndex = 0; // Sensor da esquerda
const int sensorDireitaIndex = QTSensores - 1; // Sensor da direita
const int limiarEncruzilhada = 9;  // Número de sensores ativos para detectar encruzilhada
bool isEncruzilhada = false;       // Indica se está em uma encruzilhada
const unsigned long tempoVirada = 200; // Tempo em ms para virar 90 graus

bool podeContarNovoQuadrado = true; // Flag para evitar contagem dupla
unsigned long tempoIgnoreEsquerda = 0;  // Tempo da última detecção de quadrado à esquerda
unsigned long tempoIgnoreDireita = 0;   // Tempo da última detecção de quadrado à direita
const unsigned long duracaoIgnore = 300; // Duração (ms) para ignorar sensores laterais após quadrado
unsigned long tempoPerdaLinha = 0;
const unsigned long tempoLimiteParada = 650; 
// Flag para controlar se o robô deve ficar parado
bool roboParado = false;
bool BRANCO
// Variáveis para faixa de pedestres
bool emFaixaPedestres = false; // Indica se o robô está na faixa de pedestres
unsigned long tempoParadaFaixa = 0; // Tempo de início da parada na faixa
const unsigned long tempoParadaMin = 5000; // 5 segundos de parada mínima
const unsigned long tempoParadaMax = 10000; // 10 segundos de parada máxima
const int limiarFaixaPedestres = QTSensores - 2; // Número mínimo de sensores detectando PRETO

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
    int indiceInvertido = (QTSensores - 1) - i; // 
    // Atualiza o histórico de leituras na posição invertida
    HistoricoLeituras[indiceInvertido][IndiceLeitura] = leituraAtual; // 

    long soma = 0; // Usar 'long' para a soma evita estouro (overflow)
    for (int k = 0; k < NumLeituras; k++) {
      soma += HistoricoLeituras[indiceInvertido][k]; // 
    }
    // Armazena a média no vetor Sensor na posição invertida
    Sensor[indiceInvertido] = soma / NumLeituras; // 
  }

  // O resto da função permanece igual
  // Atualiza o índice de controle para o histórico de leituras (circular)
  IndiceLeitura = (IndiceLeitura + 1) % NumLeituras;

  // Impressão de dados para depuração
  if (!MODO_PRODUCAO){
    ImprimirSensores(Antropofagico);
  }
  // Funções auxiliares para processamento de dados
  Discretiza();
  ContagemQuadrados();
}

void ContagemQuadrados() {
  // --- Configurações ---
  static unsigned long ultimaContagem[2] = {0, 0};   // 0 = esquerda, 1 = direita
  const unsigned long debounceInterval = 200;        // ms
  const int maxQuadrados = 1;

  unsigned long now = millis();

  // --- Leitura segura dos sensores vizinhos (evita out-of-bounds) ---
  bool sEsq = false;
  bool sDir = false;
  // supondo QTSensores disponível:
  if (sensorEsquerdaIndex >= 0 && sensorEsquerdaIndex < QTSensores) {
    sEsq = ((SensorBIN[sensorEsquerdaIndex+1] == BRANCO) && (SensorBIN[SENSOR_CENTRAL] == BRANCO) && (SensorBIN[sensorEsquerdaIndex + 2] == PRETO) && (SensorBIN[sensorEsquerdaIndex + 3] == PRETO));
  }
  if (sensorDireitaIndex >= 0 && sensorDireitaIndex < QTSensores) {
    sDir = ((SensorBIN[sensorDireitaIndex] == BRANCO) && (SensorBIN[SENSOR_CENTRAL] == BRANCO) && (SensorBIN[sensorDireitaIndex - 2] == PRETO) && (SensorBIN[sensorDireitaIndex - 3] == PRETO));
  }

  // --- Só conta quadrados se não estiver em encruzilhada ou faixa de pedestres ---
  if (!isEncruzilhada && !emFaixaPedestres) {
    // Contar somente na transição 0->1 (rising edge) e respeitar debounce
    if (sEsq && !lastSensorEsquerda && (now - ultimaContagem[0] > debounceInterval)) {
      contadorQuadradosEsquerda++;
      ultimaContagem[0] = now;
    }
    if (sDir && !lastSensorDireita && (now - ultimaContagem[1] > debounceInterval)) {
      contadorQuadradosDireita++;
      ultimaContagem[1] = now;
    }
  }

  // Limites
  contadorQuadradosEsquerda = constrain(contadorQuadradosEsquerda, 0, maxQuadrados);
  contadorQuadradosDireita  = constrain(contadorQuadradosDireita,  0, maxQuadrados);

  // Atualiza estados anteriores (para detecção de borda)
  lastSensorEsquerda = sEsq;
  lastSensorDireita  = sDir;

  if (!MODO_PRODUCAO){
    Serial.print("sE:"); Serial.print(sEsq);
    Serial.print(" lastE:"); Serial.print(lastSensorEsquerda);
    Serial.print(" sD:"); Serial.print(sDir);
    Serial.print(" lastD:"); Serial.print(lastSensorDireita);
    Serial.print(" isEncruz:"); Serial.print(isEncruzilhada);
    Serial.print(" QEsq:"); Serial.print(contadorQuadradosEsquerda);
    Serial.print(" QDir:"); Serial.println(contadorQuadradosDireita);
  }
}

void ImprimirSensores(int Antropofagico) {
  if (Antropofagico == 1) {
    for (int i = 0; i < QTSensores; i++) {
      Serial.print(Sensor[i]);
      if (i < QTSensores - 1) {
        Serial.print("| "); // Adiciona vírgula entre os sensores
      }
    }
  }
  else if (Antropofagico == 2) {
    for (int i = 0; i < QTSensores; i++) {
      Serial.print(SensorBIN[i]);
      if (i < QTSensores - 1) {
        Serial.print("| "); // Adiciona vírgula entre os sensores
      }
    }
  }
  if (Antropofagico != 0) {
    Serial.print(" | VeloE: " + String(VeloE) + " | VeloD: " + String(VeloD) + " | Erro: " + String(erro));
    Serial.print(" | Quadrados Esq: " + String(contadorQuadradosEsquerda) + " | Quadrados Dir: " + String(contadorQuadradosDireita));
    Serial.print(" | FaixaPedestres: " + String(emFaixaPedestres));
    Serial.println();
  }
}

void Discretiza() {
  for (int i = 0; i < QTSensores; i++) {
      // Discretiza o valor com base no valor de corte
      if (Sensor[i] > corte[i]) {
          SensorBIN[i] = true; // Estado ALTO
      } else {
          SensorBIN[i] = false; // Estado BAIXO
      }
      if (i == 2){
        if (Sensor[i] >= 33) {
          SensorBIN[i] = true; // Estado ALTO
          } else {
              SensorBIN[i] = false; // Estado BAIXO
          }
      }
  }
}

void CalculaErro() {
    long soma = 0;
    int ativos = 0;

    for (int i = 0; i < QTSensores; i++) {
        if (SensorBIN[i] == BRANCO) {  // Detecta a linha
            soma += i * 1000;         // Peso proporcional à posição
            ativos++;
        }
    }

    if (ativos > 0) {
        float posicaoMedia = (float)soma / ativos;  
        erro = (posicaoMedia - (SENSOR_CENTRAL * 1000)) / 1000.0;
    } else {
        erro = erroA; // Mantém o erro anterior se não encontrou linha
    }
}

void CalculaPID() {
  // --- Cálculo dos termos ---
  P = erro * Kp;
  I = I + erro;
  D = erro - erroA;

  AntiWindUp(); // Limita a parte integrativa

  // --- Saída PID ---
  PID = P + (Ki * I) + (Kd * D);

  // --- Atualiza erro anterior ---
  erroA = erro;
}

void AntiWindUp() { 
  // Zera a parte integrativa quando o erro some ou inverte
  if (erro == 0) {
    I = 0;
  }
  if ((erro > 0 && erroA < 0) || (erro < 0 && erroA > 0)) {
    I = 0;
  }
}

void AutoTunePID() {
  if (autoTuningEnabled && (millis() - lastTuneTime > tuneInterval)) {
    // Ajuste proporcional adaptativo simples
    if (erro > 0) {
      Kp += 0.1;
    } else {
      Kp -= 0.1;
    }

    // Pequeno ajuste em Ki e Kd
    Ki += 0.01;
    Kd += 0.001;

    // Evita valores explosivos
    Kp = constrain(Kp, 0, 10);
    Ki = constrain(Ki, 0, 1);
    Kd = constrain(Kd, 0, 1);

    lastTuneTime = millis();
  }
}

void virar90Direita() {
  // Parar roda direita (motorE), andar frente com esquerda (motorD)
  digitalWrite(dirMotorD, LOW);   // Motor esquerdo frente
  analogWrite(pwmMotorD, PWMD);
  digitalWrite(dirMotorE, HIGH);   // Motor direito frente (mas pwm 0 para parar)
  analogWrite(pwmMotorE, PWME);
  delay(tempoVirada);             // Tempo para virar 90 graus

  // Parar ambos os motores após a virada
  analogWrite(pwmMotorD, 0);
  analogWrite(pwmMotorE, 0);
}

void virar90Esquerda() {
  // Parar roda esquerda (motorD), andar frente com direita (motorE)
  digitalWrite(dirMotorD, HIGH);   
  analogWrite(pwmMotorD, PWMD);
  digitalWrite(dirMotorE, LOW);   // Motor direito frente
  analogWrite(pwmMotorE, PWME);
  delay(tempoVirada);             // Tempo para virar 90 graus

  // Parar ambos os motores após a virada
  analogWrite(pwmMotorD, 0);
  analogWrite(pwmMotorE, 0);
}

void Seguir() {
  CalculaErro();
  // Lógica para encruzilhadas e modo rotatória
  if (isEncruzilhada) {
    bool virarAgora = false;

    if (emModoRotatoria) {
      contadorSaidas--;
      if (contadorSaidas == 0) {
        virarAgora = true;
        emModoRotatoria = false;
      }
      // Caso contrário, passa reto (continua com PID)
    } else {
      // Usa o maior número de quadrados entre esquerda e direita
      int totalQuadrados = max(contadorQuadradosEsquerda, contadorQuadradosDireita);
      if (totalQuadrados > 0) {
        virarAgora = true;
        emModoRotatoria = true;
        contadorSaidas = totalQuadrados;
        contadorQuadradosEsquerda = 0;
        contadorQuadradosDireita = 0;
      }
      // Caso contrário, passa reto (continua com PID)
    }

    if (virarAgora) {
      // Decidir direção com base nos contadores de quadrados
      if (contadorQuadradosEsquerda >= contadorQuadradosDireita) {
        if (MODO_PRODUCAO){
          virar90Esquerda(); // Mais quadrados à esquerda, virar à esquerda
          if (Antropofagico != 0) {
            Serial.println("Virando à esquerda!");
          }
        }
      } else {
        if (MODO_PRODUCAO){
          virar90Direita(); // Mais quadrados à direita, virar à direita
          if (Antropofagico != 0) {
            Serial.println("Virando à direita!");
          }
        }
      }
      return; // Após virar, sai da função (próximo loop fará nova leitura)
    }
  }

  // Lógica PID normal (passa reto em encruzilhadas se não for para virar)
  CalculaPID();
  AutoTunePID();

  // --- Limita saída PID ---
  PID = constrain(PID, -MAXR, MAXR);

  // --- Calcula velocidades ---
  if (PID > 0) { // Correção para direita
    VeloE = PWME + PID;
    VeloD = PWMD - PID;
  } else {       // Correção para esquerda
    VeloE = PWME + PID;  // PID é negativo
    VeloD = PWMD - PID;  // subtrair negativo = somar
  }

  // --- Limita velocidades ---
  VeloE = constrain(VeloE, 0, MAXR);
  VeloD = constrain(VeloD, 0, MAXR);
  if (MODO_PRODUCAO){
    // --- Controle dos motores ---
      if (VeloE >= MAXR && VeloD <= 0) {
        // Curva fechada esquerda
        digitalWrite(dirMotorE, LOW);   // Motor E frente
        digitalWrite(dirMotorD, HIGH);  // Motor D trás
        analogWrite(pwmMotorE, VeloE);
        analogWrite(pwmMotorD, VeloD);
      } else if (VeloD >= MAXR && VeloE <= 0) {
        // Curva fechada direita
        digitalWrite(dirMotorE, HIGH);  // Motor E trás
        digitalWrite(dirMotorD, LOW);   // Motor D frente
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

    int calibMin[QTSensores];
    int calibMax[QTSensores] = {0};

    for (int i = 0; i < QTSensores; i++) {
        calibMin[i] = 1023;
    }

    if (MODO_PRODUCAO == 0) {
      Serial.println("Iniciando calibracao... Mova o robo sobre a linha e o fundo branco.");
    }

    // Realiza as leituras durante o tempo de calibração
    while (millis() - tempoInicial < tempoCalibracao) {
        // O loop varre os canais do MUX na ordem física (0, 1, 2...)
        for (int sensorIndex = 0; sensorIndex < QTSensores; sensorIndex++) {
            // Configura os pinos do MUX para o sensor atual
            Mandar_Mux_Bin[0] = (sensorIndex & 0x01);
            Mandar_Mux_Bin[1] = (sensorIndex & 0x02) >> 1;
            Mandar_Mux_Bin[2] = (sensorIndex & 0x04) >> 2;
            Mandar_Mux_Bin[3] = (sensorIndex & 0x08) >> 3;
            
            for (int j = 0; j < 4; j++) {
                digitalWrite(MUX_S[j], Mandar_Mux_Bin[j]);
            }

            delayMicroseconds(50);
            int valorLido = analogRead(MUX_SIG);

            // Calcula o índice invertido
            int indiceInvertido = (QTSensores - 1) - sensorIndex;

            // Usa o 'indiceInvertido' para armazenar os valores
            if (valorLido < calibMin[indiceInvertido]) {
                calibMin[indiceInvertido] = valorLido;
            }
            if (valorLido > calibMax[indiceInvertido]) {
                calibMax[indiceInvertido] = valorLido;
            }
        }
        yield();
    }

    if (MODO_PRODUCAO == 0) {
      Serial.println("\nCalibracao finalizada!");
    }

    // Calcula e armazena o corte
    for (int i = 0; i < QTSensores; i++) {
        corte[i] = (calibMax[i] + calibMin[i]) / 2;

        if (MODO_PRODUCAO == 0){
            Serial.print("Sensor " + String(i));
            Serial.print(" | Min: " + String(calibMin[i]));
            Serial.print(" | Max: " + String(calibMax[i]));
            Serial.println(" | Corte: " + String(corte[i]));
        }
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

bool DetectarQuadrado() {
    int ativos = 0;
    int meio = SENSOR_CENTRAL;
    for (int i = max(0, meio-3); i <= min(QTSensores-1, meio+3); i++) {
        if (SensorBIN[i] == BRANCO) ativos++;
    }
    return (ativos >= 2 && ativos <= 4);
}

bool DetectarEncruzilhada() {
  if (SensorBIN[0] == BRANCO && SensorBIN[6] == BRANCO && SensorBIN[10] == BRANCO) {
    return true;
  }
  else if (SensorBIN[0] == BRANCO && SensorBIN[1] == BRANCO && SensorBIN[2] == BRANCO) {
    return true;
  } 
  else if (SensorBIN[12] == BRANCO && SensorBIN[11] == BRANCO && SensorBIN[10] == BRANCO) {
    return true;
  }
  return false;
}

bool DetectarFaixaPedestres() {
  // Conta quantos sensores detectam PRETO (pista branca com linha preta)
  int sensoresPreto = 0;
  for (int i = 0; i < QTSensores; i++) {
    if (SensorBIN[i] == PRETO) {
      sensoresPreto++;
    }
  }
  // Detecta a faixa se a maioria dos sensores (limiarFaixaPedestres) vêem PRETO
  bool faixaDetectada = (sensoresPreto >= limiarFaixaPedestres);
  
  if (faixaDetectada && !MODO_PRODUCAO) {
    Serial.println("Faixa de pedestres detectada!");
  }
  return faixaDetectada;
}

void pararMotores() {
  analogWrite(pwmMotorE, 0);
  analogWrite(pwmMotorD, 0);
  digitalWrite(dirMotorE, LOW);
  digitalWrite(dirMotorD, LOW);
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
  if (!MODO_PRODUCAO) {
    Serial.begin(115200);
  }
  
  // Aguarda pressionar o botão de calibração
  if (!MODO_PRODUCAO) {
    Serial.println("Calibrando sensores...!");
  }
  // Chama a função de calibração
  Calibracao();
  delay(800);
  if (!MODO_PRODUCAO) {
    Serial.println("======= avua fi!======");
  }
}

bool verificaPerdaTotalLinha() {
  const int inicioSensoresCentrais = SENSOR_CENTRAL - 3;
  const int fimSensoresCentrais = SENSOR_CENTRAL + 3;

  // Ignora verificação de perda de linha durante a faixa de pedestres
  if (emFaixaPedestres) {
    return false; // Não considera perda de linha na faixa
  }

  // Verifica as pontas
  if (SensorBIN[0] == BRANCO || SensorBIN[QTSensores - 1] == BRANCO) {
    return false;
  }

  // Verifica o bloco de sensores centrais
  for (int i = inicioSensoresCentrais; i <= fimSensoresCentrais; i++) {
    if (SensorBIN[i] == BRANCO) {
      return false;
    }
  }
  return true;
}

void loop() {
  isEncruzilhada = DetectarEncruzilhada();
  Leitura();
  
  // Se o robô está parado devido à perda de linha, não faz nada
  if (roboParado) {
    pararMotores();
    return;
  }

  // Verifica faixa de pedestres
  if (!emFaixaPedestres && DetectarFaixaPedestres()) {
    emFaixaPedestres = true;
    tempoParadaFaixa = millis();
    pararMotores();
    if (!MODO_PRODUCAO) {
      Serial.println("Parando na faixa de pedestres!");
    }
  }

  // Lógica da faixa de pedestres
  if (emFaixaPedestres) {
    unsigned long tempoAtual = millis();
    if (tempoAtual - tempoParadaFaixa >= tempoParadaMin && tempoAtual - tempoParadaFaixa <= tempoParadaMax) {
      pararMotores(); // Continua parado durante o período de 5 a 10 segundos
    } else if (tempoAtual - tempoParadaFaixa > tempoParadaMax) {
      emFaixaPedestres = false; // Sai do modo faixa de pedestres
      tempoPerdaLinha = 0; // Reseta o cronômetro de perda de linha para ignorar o gap
      if (!MODO_PRODUCAO) {
        Serial.println("Retomando seguimento após faixa de pedestres!");
      }
    }
    return; // Não executa o seguimento enquanto está na faixa
  }

  // Verifica perda de linha
  if (verificaPerdaTotalLinha()) {
    if (tempoPerdaLinha == 0) {
      tempoPerdaLinha = millis();
    }
  } else {
    tempoPerdaLinha = 0;
  }

  // Para se a linha for perdida por muito tempo
  if (tempoPerdaLinha != 0 && (millis() - tempoPerdaLinha > tempoLimiteParada)) {
    roboParado = true;
    if (!MODO_PRODUCAO) {
      Serial.println("Linha perdida por mais de 500ms. PARANDO!");
    }
  }

  // Segue a linha normalmente se não está parado
  if (!roboParado) {
    Seguir();
  }
  yield();
}