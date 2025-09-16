#include "Constante.c"

int Sensor[QTSensores] = {0}; // Inicializa zerando tudo
bool SensorBIN[QTSensores] = {1};
int HistoricoLeituras[QTSensores][NumLeituras];  // Armazena as últimas 5 leituras de cada sensor
int IndiceLeitura = 0;                    // Índice de controle para o histórico de leituras

bool Mandar_Mux_Bin[4] = {0};
int corte[QTSensores] = {0};
// Arrays globais para guardar min e max de cada sensor
int menores[QTSensores][1];
int maiores[QTSensores][1];
// ======================= CONFIGURAÇÃO DE MODO =======================
// Mude para 1 para o modo de produção. Isso desativa TODA a comunicação
// serial para máxima eficiência e velocidade do robô.
const int MODO_PRODUCAO = 1; // 0 = Modo Debug (Serial ATIVO), 1 = Modo Produção (Serial DESATIVADO)
const int SEM_MOTOR = !MODO_PRODUCAO;
// ====================================================================

int P = 0, I = 0, D = 0, PID = 0;
float erro = 0, erroA = 0;
int VeloE, VeloD;
unsigned long CalibraInterval = 0; // Tempo de inicia de calibracao
//////////////////////////////////////// PID ////////////////////////////////////////
float Kp = 16, Ki = 0.02, Kd = 3; // Parâmetros do PID
float targetValue = 0; // Valor alvo
bool autoTuningEnabled = false; // Habilitar/desabilitar auto-tuning
unsigned long lastTuneTime = 0; // Tempo da última atualização de tuning
const unsigned long tuneInterval = 1000; // Intervalo de tempo para ajuste
/////////////////////////////////////////////////////////////////////////////////////
int i = 0, j = 0;
int Antropofagico = 2;

unsigned long tempoPerdaLinha = 0;

// Constante que define o tempo limite para a parada em milissegundos.
const unsigned long tempoLimiteParada = 650; 

// Flag para controlar se o robô deve ficar parado.
bool roboParado = false;
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
  if(Antropofagico != 0){
      Serial.print(" | VeloE: " + String(VeloE) + " | VeloD: " + String(VeloD) + "|");
  Serial.println(erro); // Imprime o valor do erro
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
      if (i == 5){
        if (Sensor[i] >= 400) {
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
bool DetectarEncruzilhada() {
    int ativos = 0;
    int sensoresEsq = 0;
    int sensoresDir = 0;

    for (int i = 0; i < QTSensores; i++) {
        if (SensorBIN[i] == BRANCO) {
            ativos++;
            if (i < SENSOR_CENTRAL) sensoresEsq++;
            else if (i > SENSOR_CENTRAL) sensoresDir++;
        }
    }

    // --- Critérios de encruzilhada ---
    // 1. Muitos sensores ativos ao mesmo tempo
    if (ativos >= 4) return true;

    // 2. Linha nos dois lados (esquerda e direita) junto com o centro
    if (sensoresEsq > 0 && sensoresDir > 0 && SensorBIN[SENSOR_CENTRAL] == BRANCO) {
        return true;
    }

    // 3. Todos sensores ativos (linha larga)
    if (ativos == QTSensores) return true;

    return false; // caso contrário, não é encruzilhada
}

void AntiWindUp(float limiteI = 10.0, float zonaMorta = 2) {
    // --- Zera integrador se erro quase zero (zona morta) ---
    if (abs(erro) < zonaMorta) {
        I = 0;
    }

    // --- Zera integrador se erro mudou de sinal ---
    if ((erro > 0 && erroA < 0) || (erro < 0 && erroA > 0)) {
        I = 0;
    }

    // --- Limita o integrador para evitar wind-up ---
    if (I > limiteI) I = limiteI;
    if (I < -limiteI) I = -limiteI;
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

void Seguir() {
  CalculaErro();
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

  // --- Controle dos motores ---
  if (!SEM_MOTOR){
    if (VeloE >= MAXR && VeloD <= 0) {
      // Curva fechada esquerda
      digitalWrite(dirMotorE, LOW);   // Motor E frente
      digitalWrite(dirMotorD, HIGH);  // Motor D trás
      analogWrite(pwmMotorE, VeloE);
      analogWrite(pwmMotorD, VeloE);

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

            // <<< CORREÇÃO: Calcula o índice invertido, exatamente como na função Leitura()
            int indiceInvertido = (QTSensores - 1) - sensorIndex;

            // <<< CORREÇÃO: Usa o 'indiceInvertido' para armazenar os valores
            // Se o valor lido for menor que o mínimo já salvo, atualiza o mínimo.
            if (valorLido < calibMin[indiceInvertido]) {
                calibMin[indiceInvertido] = valorLido;
            }

            // Se o valor lido for maior que o máximo já salvo, atualiza o máximo.
            if (valorLido > calibMax[indiceInvertido]) {
                calibMax[indiceInvertido] = valorLido;
            }
        }
        yield();
    }

    if (MODO_PRODUCAO == 0) {
      Serial.println("\nCalibracao finalizada!");
    }

    // Calcula e armazena o corte. Este loop já funciona corretamente, pois os 
    // dados nos arrays calibMin e calibMax já estão na ordem invertida.
    for (int i = 0; i < QTSensores; i++) {
        // A posição 'i' aqui corresponde ao 'indiceInvertido' desejado.
        corte[i] = (calibMax[i] + calibMin[i]) / 2;

        if (MODO_PRODUCAO == 0){
            // Imprime "Sensor 0" mas com os dados do último sensor físico, mantendo a consistência.
            Serial.print("Sensor " + String(i));
            Serial.print(" | Min: " + String(calibMin[i]));
            Serial.print(" | Max: " + String(calibMax[i]));
            Serial.println(" | Corte: " + String(corte[i]));
        }
    }
}
void pararMotores() {
  analogWrite(pwmMotorE, 0);
  analogWrite(pwmMotorD, 0);
  // Opcional: Apenas para garantir um estado conhecido.
  digitalWrite(dirMotorE, LOW);
  digitalWrite(dirMotorD, LOW);
}

/**
 * @brief Verifica se todos os sensores estão lendo a cor PRETO.
 * @return Retorna 'true' se todos os sensores estiverem no preto, 'false' caso contrário.
 */
bool verificaPerdaTotalLinha() {
  // ====================================================================
  // /// AJUSTE AQUI O INTERVALO DOS SEUS SENSORES CENTRAIS ///
  // ====================================================================
  // A definição exata depende do seu QTSensores.
  // Exemplo para QTSensores = 16: O centro está entre 7 e 8.
  //    Um bloco de 5 sensores seria do 5 ao 9 (5, 6, 7, 8, 9).
  // Exemplo para QTSensores = 8: O centro está entre 3 e 4.
  //    Um bloco de 5 sensores seria do 1 ao 5 (1, 2, 3, 4, 5).
  const int inicioSensoresCentrais = SENSOR_CENTRAL - 1; // << AJUSTE AQUI
  const int fimSensoresCentrais = SENSOR_CENTRAL + 1;    // << AJUSTE AQUI
  // ====================================================================

  // --- Passo 1: Verificar os sensores das pontas ---
  // Se o primeiro OU o último sensor virem a linha, ela não está perdida.
  if (SensorBIN[0] == BRANCO || SensorBIN[QTSensores - 1] == BRANCO) {
    return false; // Encontrou a linha em uma das pontas.
  }

  // --- Passo 2: Verificar o bloco de sensores centrais ---
  for (int i = inicioSensoresCentrais; i <= fimSensoresCentrais; i++) {
    // Se qualquer um dos sensores centrais vir a linha, ela não está perdida.
    if (SensorBIN[i] == BRANCO) {
      return false; // Encontrou a linha no centro.
    }
  }

  // --- Passo 3: Conclusão ---
  // Se o código chegou até aqui, significa que NEM as pontas NEM os sensores
  // centrais designados viram a linha. Portanto, a linha está perdida.
  return true;
}
void setup() {
  // Sensores
  pinMode(MUX_SIG, INPUT);
  for (int i = 0; i < 4; i++) {
      pinMode(MUX_S[i], OUTPUT);
  }
  // Motores
  pinMode(pwmMotorE, OUTPUT);
  pinMode(dirMotorE, OUTPUT);
  pinMode(pwmMotorD, OUTPUT);
  pinMode(dirMotorD, OUTPUT);

  // <<< ALTERADO: Inicializa o Serial e imprime mensagens apenas se não estiver em modo de produção.
  if (MODO_PRODUCAO == 0) {
    Serial.begin(115200);
    Serial.println("Modo Debug Ativado. Calibrando sensores...!");
  }
  Calibracao();
  pinMode(pinoMultifuncao, INPUT_PULLUP); 

  // Aguarda o botão ser pressionado para iniciar
  while(digitalRead(pinoMultifuncao) == HIGH) {
    // Laço vazio, apenas esperando o botão...
    yield(); // Boa prática para ESPs e outras arquiteturas
  }

  // O botão foi pressionado!
  Serial.println("Botao pressionado!");
  
  // Um pequeno delay para "debounce" - evitar múltiplas leituras de um só clique.
  delay(600); 
  // <<< ALTERADO: Imprime mensagem final apenas se não estiver em modo de produção.
  if (MODO_PRODUCAO == 0) {
    Serial.println("======= avua fi!======");
  }
  
  //digitalWrite(6,HIGH);
}

void loop() {
  // A leitura dos sensores sempre acontece, independente de qualquer outra coisa.
  Leitura();
  
  // Se o robô já recebeu a ordem de parar, ele apenas para os motores e não faz mais nada.
  if (roboParado) {
    pararMotores();
    return; // Encerra este ciclo do loop aqui.
  }

  // <<< INÍCIO DA LÓGICA DE PARADA AUTOMÁTICA >>>
  Serial.println(verificaPerdaTotalLinha());
  // 1. Verifica se a linha foi totalmente perdida neste ciclo.
  if (verificaPerdaTotalLinha()) {
    // 2. Se a linha foi perdida, verifica se o cronômetro já foi iniciado.
    if (tempoPerdaLinha == 0) {
      // Se for 0, significa que a perda acabou de acontecer.
      // Inicia o cronômetro marcando o tempo atual.
      tempoPerdaLinha = millis();
    }
  } else {
    // Se a linha foi encontrada, reseta o cronômetro.
    tempoPerdaLinha = 0;
  }

  // 3. Verifica se o robô deve ser parado.
  // A condição é: o cronômetro foi iniciado (é diferente de 0) E já se passaram 500ms.
  if (tempoPerdaLinha != 0 && (millis() - tempoPerdaLinha > tempoLimiteParada)) {
    // Define a flag de parada como verdadeira.
    roboParado = true;
    
    // Opcional: Imprime uma mensagem de aviso no modo de depuração.
    if(MODO_PRODUCAO == 0){
      Serial.println("Linha perdida por mais de 500ms. PARANDO!");
    }
  }
  
  // <<< FIM DA LÓGICA DE PARADA AUTOMÁTICA >>>

  // Se a flag roboParado ainda for falsa, o robô segue a linha normalmente.
  if (!roboParado) {
    Seguir();
  }

  yield();
}
