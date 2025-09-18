#include "Constante.c"

// =============================================================================
// 1. ARQUITETURA DA MÁQUINA DE ESTADOS
// =============================================================================

// Enumeração de todos os possíveis estados do robô
enum EstadoRobo {
  CALIBRANDO,
  SEGUINDO_LINHA,
  AVALIANDO_CRUZAMENTO, // Estado transitório para decidir o que fazer
  CURVA_SIMPLES,        // Executando uma curva de 90 graus
  NAVEGANDO_ROTATORIA,  // Dentro da rotatória, seguindo a linha e contando saídas
  SAINDO_ROTATORIA,     // Executando a manobra para sair da rotatória
  VERIFICA_FIM_PISTA,   // Viu tudo preto, vai tentar a ré
  MANOBRA_RE,           // Executando a ré
  PARADO                // Estado final, motores desligados
};

// Variável global que armazena o estado atual do robô
EstadoRobo estadoAtual = SEGUINDO_LINHA; // Começa seguindo a linha após a calibração

// =============================================================================
// 2. CONSTANTES E PARÂMETROS GLOBAIS (Seu código original)
// =============================================================================

// --- Modos de Operação ---
const int MODO_PRODUCAO = 0; // 0 = Debug (Serial ATIVO), 1 = Produção (Serial DESATIVADO)

// --- Parâmetros do Controle PID ---
float Kp = 16.0, Ki = 0.03, Kd = 3.0;

// --- Parâmetros de Navegação ---
const int sensorEsquerdaIndex = 0;
const int sensorDireitaIndex = QTSensores - 1;
const unsigned long tempoVirada = 200;
const unsigned long tempoMaximoEntreQuadrados = 5000;
const unsigned long TEMPO_MAXIMO_RE = 2000;
const unsigned long debouceBlack = 650;
const unsigned long tempoLimiteParada = 650;


// =============================================================================
// 3. VARIÁVEIS GLOBAIS (Seu código original, com adições para a FSM)
// =============================================================================

// --- Dados dos Sensores ---
int Sensor[QTSensores] = {0};
bool SensorBIN[QTSensores] = {false};
int corte[QTSensores] = {0};
int HistoricoLeituras[QTSensores][NumLeituras];
int IndiceLeitura = 0;
unsigned long tempoPerdaLinha = 0;
bool isEncruzilhada = false;

// --- Controle PID ---
float erro = 0.0, erroA = 0.0;
float P = 0, I = 0, D = 0;
int PID = 0;
int VeloE = 0, VeloD = 0;

// --- Lógica de Navegação e Estado ---
int contadorQuadradosEsquerda = 0;
int contadorQuadradosDireita = 0;
bool lastSensorEsquerda = false;
bool lastSensorDireita = false;
unsigned long tempoUltimoQuadrado = 0;

// --- Variáveis de controle para a Rotatória ---
enum Direcao { NENHUMA, ESQUERDA, DIREITA };
Direcao direcaoDaManobra = NENHUMA; // Usado para curvas e rotatórias
int saidasParaIgnorar = 0;
unsigned long timestampTransicao; // Para pausas entre estados

// --- Variáveis de Depuração ---
int Antropofagico = 2; // 1 = Analógico, 2 = Binário

// =============================================================================
// 4. SETUP E CALIBRAÇÃO (Seu código original, sem alterações)
// =============================================================================

void setup() {
  pinMode(MUX_SIG, INPUT);
  for (int i = 0; i < 4; i++) { pinMode(MUX_S[i], OUTPUT); }
  pinMode(pwmMotorE, OUTPUT);
  pinMode(dirMotorE, OUTPUT);
  pinMode(pwmMotorD, OUTPUT);
  pinMode(dirMotorD, OUTPUT);

  if (!MODO_PRODUCAO) {
    Serial.begin(115200);
    Serial.println("Iniciando... Pressione o botao para calibrar.");
  }
  delay(1000);
  Calibracao();
  delay(800);
  if (!MODO_PRODUCAO) {
    Serial.println("======= Calibracao concluida. Iniciando percurso! ======");
  }
}

void Calibracao() {
    const unsigned long tempoCalibracao = 5000;
    unsigned long tempoInicial = millis();

    int calibMin[QTSensores];
    int calibMax[QTSensores] = {0};

    // Inicializa o array de mínimos com o maior valor possível
    for (int i = 0; i < QTSensores; i++) {
        calibMin[i] = 1023;
    }

    if (MODO_PRODUCAO == 0) {
      Serial.println("Iniciando calibracao... Mova o robo sobre a linha e o fundo branco.");
    }

    // Loop de leitura durante o tempo de calibração
    while (millis() - tempoInicial < tempoCalibracao) {
        for (int sensorIndex = 0; sensorIndex < QTSensores; sensorIndex++) {
            // Configura os pinos de seleção do MUX
            for (int j = 0; j < 4; j++) {
                digitalWrite(MUX_S[j], (sensorIndex >> j) & 0x01);
            }
            
            int valorLido = analogRead(MUX_SIG);

            // A leitura é feita na ordem física (0, 1, 2...), mas os dados são
            // armazenados na ordem invertida para que o sensor 0 lógico seja o da esquerda.
            int indiceInvertido = (QTSensores - 1) - sensorIndex;

            if (valorLido < calibMin[indiceInvertido]) {
                calibMin[indiceInvertido] = valorLido;
            }
            if (valorLido > calibMax[indiceInvertido]) {
                calibMax[indiceInvertido] = valorLido;
            }
        }
        yield(); // Permite que processos de fundo do ESP32/ESP8266 rodem
    }

    // Calcula o ponto de corte (média entre min e max) para cada sensor
    for (int i = 0; i < QTSensores; i++) {
        corte[i] = (calibMax[i] + calibMin[i]) / 2;
    }

    if (MODO_PRODUCAO == 0) {
        Serial.println("\nCalibracao finalizada!");
        for (int i = 0; i < QTSensores; i++) {
            Serial.print("Sensor " + String(i));
            Serial.print(" | Min: " + String(calibMin[i]));
            Serial.print(" | Max: " + String(calibMax[i]));
              Serial.println(" | Corte: " + String(corte[i]));
        }
    }
}

// =============================================================================
// 5. LEITURA E PROCESSAMENTO DOS SENSORES
// =============================================================================

void Leitura() {
  bool Mandar_Mux_Bin[4];

  // Itera sobre cada sensor na ordem física do MUX
  for (int i = 0; i < QTSensores; i++) {
    // Configura os pinos de seleção do MUX para ler o sensor 'i'
    for (int j = 0; j < 4; j++) {
      digitalWrite(MUX_S[j], (i >> j) & 0x01);
    }
    
    int leituraAtual = analogRead(MUX_SIG);
    
    // Armazena a leitura na ordem lógica invertida
    int indiceInvertido = (QTSensores - 1) - i;
    HistoricoLeituras[indiceInvertido][IndiceLeitura] = leituraAtual;

    // Calcula a média móvel para suavizar a leitura
    long soma = 0;
    for (int k = 0; k < NumLeituras; k++) {
      soma += HistoricoLeituras[indiceInvertido][k];
    }
    Sensor[indiceInvertido] = soma / NumLeituras;
  }
  
  // Avança o índice do buffer circular
  IndiceLeitura = (IndiceLeitura + 1) % NumLeituras;

  // Processa os dados lidos
  Discretiza();
  ContagemQuadrados();
  
  if (!MODO_PRODUCAO) {
    ImprimirSensores(Antropofagico);
  }
}

void Discretiza() {
  for (int i = 0; i < QTSensores; i++) {
    // Converte a leitura analógica em binária (PRETO/BRANCO) usando o limiar de corte
    SensorBIN[i] = (Sensor[i] > corte[i]);
  }
  // Exceção/ajuste manual para o sensor 2, se necessário
  if (Sensor[2] >= 40) {
      SensorBIN[2] = true;
  } else {
      SensorBIN[2] = false;
  }
}

// =============================================================================
// 6. DETECÇÃO DE CARACTERÍSTICAS DA PISTA
// =============================================================================

void CalculaErro() {
    long somaPonderada = 0;
    int sensoresAtivos = 0;

    for (int i = 0; i < QTSensores; i++) {
        if (SensorBIN[i] == BRANCO) {
            // A posição de cada sensor é usada como peso
            somaPonderada += (long)i * 1000;
            sensoresAtivos++;
        }
    }

    if (sensoresAtivos > 0) {
        // O erro é a diferença entre a posição média da linha e o centro do robô
        float posicaoMedia = (float)somaPonderada / sensoresAtivos;
        erro = (posicaoMedia - (SENSOR_CENTRAL * 1000)) / 1000.0;
        erroA = erro; // Atualiza o erro anterior válido
    } else {
        // Se nenhuma linha for detectada, mantém o último erro conhecido
        erro = erroA;
    }
}

bool DetectarEncruzilhada() {
    // Define quantos sensores, para CADA lado do centro, vamos verificar.
    // Se SENSOR_CENTRAL é 6 e este valor é 4, vamos checar os sensores
    // [6, 5, 4, 3] para a esquerda e [6, 7, 8, 9] para a direita.
    // AJUSTE ESTE VALOR conforme a largura da sua linha de intersecção.
    const int sensoresParaVerificar = 4;

    // --- 1. Varre da metade para a ponta ESQUERDA ---
    bool ladoEsquerdoOK = true; // Assume que está tudo branco até que se prove o contrário
    for (int i = SENSOR_CENTRAL; i >= SENSOR_CENTRAL - sensoresParaVerificar; i--) {
        // Garante que não estamos lendo fora do array de sensores
        if (i < 0) break;

        // Se encontrar UM sensor preto, já sabemos que não é uma linha contínua.
        if (SensorBIN[i] == PRETO) {
            ladoEsquerdoOK = false;
            break; // Para a verificação deste lado
        }
    }

    // --- 2. Varre da metade para a ponta DIREITA ---
    bool ladoDireitoOK = true; // Assume que está tudo branco
    for (int i = SENSOR_CENTRAL; i <= SENSOR_CENTRAL + sensoresParaVerificar; i++) {
        // Garante que não estamos lendo fora do array
        if (i >= QTSensores) break;

        // Se encontrar UM sensor preto, para a verificação.
        if (SensorBIN[i] == PRETO) {
            ladoDireitoOK = false;
            break;
        }
    }

    // --- 3. Conclusão ---
    if (ladoEsquerdoOK || ladoDireitoOK) {
        return true;
    }

    // Se não atendeu à condição acima, não é uma encruzilhada.
    return false;
}
// FUNÇÕES AJUDANTES PARA A LÓGICA DE ZONAS

/**
 * Verifica se QUALQUER sensor em um intervalo está vendo BRANCO.
 * @param inicio O índice do primeiro sensor do intervalo.
 * @param fim O índice do último sensor do intervalo.
 * @return True se pelo menos um sensor estiver branco.
 */
// ---------- FUNÇÕES DE APOIO (substituir as antigas) ----------
int clampIndex(int x) {
  if (x < 0) return 0;
  if (x > QTSensores - 1) return QTSensores - 1;
  return x;
}

bool isQualquerBranco(int inicio, int fim) {
    inicio = clampIndex(inicio);
    fim    = clampIndex(fim);
    if (inicio > fim) return false;
    for (int i = inicio; i <= fim; i++) {
        if (SensorBIN[i] == BRANCO) return true;
    }
    return false;
}

bool isTudoPreto(int inicio, int fim) {
    inicio = clampIndex(inicio);
    fim    = clampIndex(fim);
    if (inicio > fim) return false; // Se range inválido, NÃO consideramos "tudo preto"
    for (int i = inicio; i <= fim; i++) {
        if (SensorBIN[i] == BRANCO) return false;
    }
    return true;
}

// Corrige verificações com erros de sinal/typo
bool verificaPerdaTotalLinha() {
    // Se as pontas veem a linha, não está perdida (usa -2..2 como "centro")
    if (erro <= 2 && erro >= -2) {
        return false;
    }
    // Se qualquer sensor ver branco, não está perdida
    for (int i = 0; i < QTSensores; i++) {
        if (SensorBIN[i] == BRANCO) return false;
    }
    return true;
}

bool verificaTudoPreto() {
    // Se erro está próximo do centro, não é "tudo preto"
    if (erro <= 2 && erro >= -2) { return false; }
    for (int i = 0; i < QTSensores; i++) {
        if (SensorBIN[i] != PRETO) return false;
    }
    return true;
}

void ContagemQuadrados() {
    static unsigned long ultimaContagem[2] = {0, 0}; // 0=Esq, 1=Dir
    const unsigned long debounceInterval = 200; // ms
    unsigned long now = millis();

    // Zonas (simplificado para clareza)
    bool linhaNoCentro = SensorBIN[SENSOR_CENTRAL] == BRANCO || SensorBIN[SENSOR_CENTRAL - 1] == BRANCO || SensorBIN[SENSOR_CENTRAL + 1] == BRANCO;
    bool marcaNaDireita = SensorBIN[sensorDireitaIndex] == BRANCO || SensorBIN[sensorDireitaIndex - 1] == BRANCO;
    bool marcaNaEsquerda = SensorBIN[sensorEsquerdaIndex] == BRANCO || SensorBIN[sensorEsquerdaIndex + 1] == BRANCO;
    bool hasGapEsquerda = SensorBIN[sensorEsquerdaIndex + 2] == PRETO || SensorBIN[sensorEsquerdaIndex + 3] == PRETO;
    bool hasGapDireita = SensorBIN[sensorEsquerdaIndex + 2] == PRETO || SensorBIN[sensorEsquerdaIndex + 3] == PRETO;
    // A condição para um "quadrado" é ter a linha no centro e uma marca na lateral ao mesmo tempo
    bool sDir = linhaNoCentro && marcaNaDireita;
    bool sEsq = linhaNoCentro && marcaNaEsquerda;
    
    // Se estiver em um estado de manobra, não conta quadrados para evitar leituras falsas
    if (estadoAtual != SEGUINDO_LINHA && estadoAtual != NAVEGANDO_ROTATORIA) {
        lastSensorEsquerda = sEsq;
        lastSensorDireita  = sDir;
        return;
    }

    // Zera contadores se o tempo entre quadrados for muito grande
    if (now - tempoUltimoQuadrado > tempoMaximoEntreQuadrados) {
        if (contadorQuadradosEsquerda > 0 || contadorQuadradosDireita > 0) {
            if (!MODO_PRODUCAO) Serial.println(">> Timeout entre quadrados: zerando contagem");
            contadorQuadradosEsquerda = 0;
            contadorQuadradosDireita = 0;
        }
    }

    // Contagem na borda de subida (quando o sensor detecta o quadrado pela primeira vez)
    if (sEsq && !lastSensorEsquerda && (now - ultimaContagem[0] > debounceInterval)) {
        contadorQuadradosEsquerda++;
        ultimaContagem[0] = now;
        tempoUltimoQuadrado = now;
        if (!MODO_PRODUCAO) {
            Serial.print(">> Quadrado ESQ detectado. Total: ");
            Serial.println(contadorQuadradosEsquerda);
        }
    }
    if (sDir && !lastSensorDireita && (now - ultimaContagem[1] > debounceInterval)) {
        contadorQuadradosDireita++;
        ultimaContagem[1] = now;
        tempoUltimoQuadrado = now;
        if (!MODO_PRODUCAO) {
            Serial.print(">> Quadrado DIR detectado. Total: ");
            Serial.println(contadorQuadradosDireita);
        }
    }

    lastSensorEsquerda = sEsq;
    lastSensorDireita  = sDir;
}

// Função auxiliar para evitar warnings de uso antes; retorna zonaDireitaInicio segura
int zonaDireitaStartSafe(int candidate) {
    return clampIndex(candidate);
}


// =============================================================================
// 7. CONTROLE PID
// =============================================================================

void CalculaPID() {
  P = erro * Kp;
  I = I + erro;
  D = erro - erroA;

  AntiWindUp(); 

  PID = P + (Ki * I) + (Kd * D);

  erroA = erro;
}

void AntiWindUp() {
  // Limita o termo integrativo para evitar que ele cresça indefinidamente
  // e cause instabilidade (efeito "wind-up").
  if (erro == 0 || (erro > 0 && erroA < 0) || (erro < 0 && erroA > 0)) {
    I = 0;
  }
}

// =============================================================================
// 8. AÇÕES E CONTROLE DOS MOTORES
// =============================================================================
// =============================================================================
// 8. AÇÕES E CONTROLE DOS MOTORES (Funções de Ação)
// =============================================================================

void pararMotores() {
  analogWrite(pwmMotorE, 0);
  analogWrite(pwmMotorD, 0);
}

void virar90(Direcao direcao) {
  if (direcao == DIREITA) {
    digitalWrite(dirMotorD, LOW);  // Motor D para frente
    analogWrite(pwmMotorD, PWMD);
    digitalWrite(dirMotorE, HIGH); // Motor E para trás
    analogWrite(pwmMotorE, PWME);
  } else if (direcao == ESQUERDA) {
    digitalWrite(dirMotorE, LOW);  // Motor E para frente
    analogWrite(pwmMotorE, PWME);
    digitalWrite(dirMotorD, HIGH); // Motor D para trás
    analogWrite(pwmMotorD, PWMD);
  }
  delay(tempoVirada); // ATENÇÃO: delay() trava o código.
  pararMotores();
}

void moverRe() {
  digitalWrite(dirMotorE, HIGH);
  digitalWrite(dirMotorD, HIGH);
  analogWrite(pwmMotorE, PWME);
  analogWrite(pwmMotorD, PWMD);
}

/**
 * @brief Função dedicada a seguir a linha usando o PID.
 * Esta função é chamada pelos estados que precisam seguir em frente.
 */
void SeguirLinha() {
  CalculaErro();
  CalculaPID();

  PID = constrain(PID, -MAXR, MAXR);

  VeloE = constrain(PWME + PID, 0, MAXR);
  VeloD = constrain(PWMD - PID, 0, MAXR);

  if (MODO_PRODUCAO) {
    digitalWrite(dirMotorE, LOW);
    digitalWrite(dirMotorD, LOW);
    analogWrite(pwmMotorE, VeloE);
    analogWrite(pwmMotorD, VeloD);
  }
}

// =============================================================================
// 9. LÓGICA DA MÁQUINA DE ESTADOS (O Coração da nova arquitetura)
// =============================================================================

/**
 * @brief Estado Padrão: Seguir a linha e verificar eventos.
 */
void handleSeguindoLinha() {
  // Ação:
  SeguirLinha();

  // Verificação de Transições:
  if (isEncruzilhada) {
    estadoAtual = AVALIANDO_CRUZAMENTO;
    pararMotores(); // Para para avaliar com calma
    if (!MODO_PRODUCAO) Serial.println("-> Estado: AVALIANDO_CRUZAMENTO");
  }
  else if (verificaTudoPreto()) {
    estadoAtual = VERIFICA_FIM_PISTA;
    timestampTransicao = millis(); // Marca o tempo que viu tudo preto
    if (!MODO_PRODUCAO) Serial.println("-> Estado: VERIFICA_FIM_PISTA");
  }
  else if (verificaPerdaTotalLinha()) {
    if(tempoPerdaLinha == 0) tempoPerdaLinha = millis();
    if(millis() - tempoPerdaLinha > tempoLimiteParada){
       estadoAtual = PARADO;
       if (!MODO_PRODUCAO) Serial.println("Linha perdida. -> Estado: PARADO");
    }
  } else {
    tempoPerdaLinha = 0; // Reseta timer se a linha for encontrada
  }
}

/**
 * @brief Estado de Decisão: Analisa os quadrados contados e decide se é
 * uma curva simples ou uma rotatória.
 */
void handleAvaliandoCruzamento() {
  int quadradosEsq = contadorQuadradosEsquerda;
  int quadradosDir = contadorQuadradosDireita;
  int totalQuadrados = max(quadradosEsq, quadradosDir);

  // Lógica para decidir a direção
  direcaoDaManobra = (quadradosEsq >= quadradosDir) ? ESQUERDA : DIREITA;

  // Se não houver quadrados, segue reto (se aplicável) ou para.
  // Neste caso, vamos assumir que toda encruzilhada tem um destino.
  if (totalQuadrados == 0) {
      // O que fazer se chegar numa encruzilhada sem ter visto quadrados?
      // Por enquanto, vamos voltar a seguir a linha.
      estadoAtual = SEGUINDO_LINHA;
      return;
  }
  
  // Decide se é uma rotatória (2 ou mais quadrados) ou curva simples (1 quadrado)
  if (totalQuadrados >= 2) {
    if (!MODO_PRODUCAO) Serial.println("ROTATORIA DETECTADA!");

    // LÓGICA DO PSEUDOCÓDIGO
    int saidaRotatoria = 0;
    if (totalQuadrados == 2) saidaRotatoria = 1; // 1a saida
    if (totalQuadrados == 3) saidaRotatoria = 2; // 2a saida
    if (totalQuadrados >= 4) saidaRotatoria = 3; // 3a saida
    
    saidasParaIgnorar = saidaRotatoria -1; // -1 pois a primeira saída não é ignorada
    if (saidasParaIgnorar <0) saidasParaIgnorar = 0;

    if (!MODO_PRODUCAO){
      Serial.print("Direcao: "); Serial.println(direcaoDaManobra == ESQUERDA ? "ESQUERDA" : "DIREITA");
      Serial.print("Saidas a ignorar: "); Serial.println(saidasParaIgnorar);
    }
    
    estadoAtual = NAVEGANDO_ROTATORIA;
    if (!MODO_PRODUCAO) Serial.println("-> Estado: NAVEGANDO_ROTATORIA");

  } else { // 1 quadrado
    if (!MODO_PRODUCAO) Serial.println("CURVA SIMPLES DETECTADA!");
    estadoAtual = CURVA_SIMPLES;
    if (!MODO_PRODUCAO) Serial.println("-> Estado: CURVA_SIMPLES");
  }

  // Zera os contadores para a próxima leitura
  contadorQuadradosEsquerda = 0;
  contadorQuadradosDireita = 0;
  
  // Avança um pouco para passar o centro do cruzamento
  digitalWrite(dirMotorE, LOW);
  digitalWrite(dirMotorD, LOW);
  analogWrite(pwmMotorE, PWME);
  analogWrite(pwmMotorD, PWMD);
  delay(150);
}

/**
 * @brief Estado de Ação: Executa uma curva de 90 graus.
 */
void handleCurvaSimples() {
  // Ação:
  virar90(direcaoDaManobra);
  
  // Transição:
  direcaoDaManobra = NENHUMA;
  estadoAtual = SEGUINDO_LINHA;
  if (!MODO_PRODUCAO) Serial.println("Curva concluida. -> Estado: SEGUINDO_LINHA");
  // Pequena pausa para estabilizar
  delay(100);
}

/**
 * @brief Estado Principal da Rotatória: Segue a linha e conta as saídas.
 */
void handleNavegandoRotatoria() {
  // Ação:
  SeguirLinha();

  // Verificação de Transições:
  // Detecta uma nova encruzilhada (uma saída da rotatória)
  if (isEncruzilhada) {
    if (saidasParaIgnorar > 0) {
      saidasParaIgnorar--;
      if (!MODO_PRODUCAO) {
        Serial.print("Saida ignorada. Restam: ");
        Serial.println(saidasParaIgnorar);
      }
      // Avança um pouco para passar a saída ignorada e não detectá-la de novo
      digitalWrite(dirMotorE, LOW);
      digitalWrite(dirMotorD, LOW);
      analogWrite(pwmMotorE, PWME);
      analogWrite(pwmMotorD, PWMD);
      delay(250); // Ajuste conforme necessário
    } else {
      // Esta é a saída correta!
      estadoAtual = SAINDO_ROTATORIA;
      if (!MODO_PRODUCAO) Serial.println("Saida correta encontrada! -> Estado: SAINDO_ROTATORIA");
    }
  }
}

/**
 * @brief Estado de Ação: Executa a virada para sair da rotatória.
 */
void handleSaindoRotatoria() {
  // Ação:
  virar90(direcaoDaManobra);

  // Transição:
  direcaoDaManobra = NENHUMA;
  estadoAtual = SEGUINDO_LINHA;
  if (!MODO_PRODUCAO) Serial.println("Saindo da rotatoria. -> Estado: SEGUINDO_LINHA");
  delay(100);
}


/**
 * @brief Estado de Verificação: Viu tudo preto, espera um tempo para confirmar.
 */
void handleVerificaFimPista() {
  pararMotores();
  
  // Se a condição "tudo preto" sumir, foi alarme falso.
  if (!verificaTudoPreto()){
    estadoAtual = SEGUINDO_LINHA;
    if (!MODO_PRODUCAO) Serial.println("Fim de pista era alarme falso. -> Estado: SEGUINDO_LINHA");
    return;
  }

  // Se a condição persistir pelo tempo de debounce, inicia a ré
  if(millis() - timestampTransicao > debouceBlack) {
    estadoAtual = MANOBRA_RE;
    timestampTransicao = millis(); // Marca o tempo de início da ré
    if (!MODO_PRODUCAO) Serial.println("Confirmado fim de pista. -> Estado: MANOBRA_RE");
  }
}

/**
 * @brief Estado de Ação: Move para trás procurando a linha.
 */
void handleManobraRe(){
  // Ação:
  moverRe();

  // Verificação de Transições:
  // Se encontrou a linha, volta a seguir
  if (!verificaTudoPreto()){
    pararMotores();
    estadoAtual = SEGUINDO_LINHA;
    if (!MODO_PRODUCAO) Serial.println("Linha reencontrada apos re. -> Estado: SEGUINDO_LINHA");
    delay(50);
  } 
  // Se o tempo máximo de ré esgotou, para de vez
  else if (millis() - timestampTransicao > TEMPO_MAXIMO_RE){
    estadoAtual = PARADO;
    if (!MODO_PRODUCAO) Serial.println("Timeout da re. -> Estado: PARADO");
  }
}

/**
 * @brief Estado Final: Apenas mantém os motores parados.
 */
void handleParado() {
  pararMotores();
}

// =============================================================================
// 10. LOOP PRINCIPAL (Agora com a Máquina de Estados)
// =============================================================================

void loop() {
  // Tarefas que rodam em quase todos os estados
  if (estadoAtual != PARADO && estadoAtual != CALIBRANDO) {
    Leitura();
    ContagemQuadrados(); // Continua contando quadrados em background
    isEncruzilhada = DetectarEncruzilhada();
  }

  // A MÁQUINA DE ESTADOS
  switch (estadoAtual) {
    case SEGUINDO_LINHA:
      handleSeguindoLinha();
      break;
    
    case AVALIANDO_CRUZAMENTO:
      handleAvaliandoCruzamento();
      break;

    case CURVA_SIMPLES:
      handleCurvaSimples();
      break;

    case NAVEGANDO_ROTATORIA:
      handleNavegandoRotatoria();
      break;

    case SAINDO_ROTATORIA:
      handleSaindoRotatoria();
      break;
    
    case VERIFICA_FIM_PISTA:
      handleVerificaFimPista();
      break;
    
    case MANOBRA_RE:
      handleManobraRe();
      break;

    case PARADO:
      handleParado();
      break;
  }

  // Imprime informações de debug, se não estiver em modo produção
  if (!MODO_PRODUCAO){
    ImprimirSensores(Antropofagico);
  }

  yield(); // Boa prática para ESPs
}


// =============================================================================
// 11. FUNÇÕES UTILITÁRIAS E DE DEPURAÇÃO (Seu código original, sem alterações)
// =============================================================================
void ImprimirSensores(int modo) {
  // Modo 1: Imprime os valores analógicos (0-1023)
  if (modo == 1) {
    for (int i = 0; i < QTSensores; i++) {
      Serial.print(Sensor[i]);
      if (i < QTSensores - 1) {
        Serial.print("|");
      }
    }
  }
  // Modo 2: Imprime os valores binarizados (0 ou 1)
  else if (modo == 2) {
    for (int i = 0; i < QTSensores; i++) {
      Serial.print(SensorBIN[i]);
      if (i < QTSensores - 1) {
        Serial.print("|");
      }
    }
  }

  // Imprime informações adicionais de estado
  if (modo != 0) {
    Serial.print(" | Err: " + String(erro));
    Serial.print(" | PID: " + String(PID));
    Serial.print(" | V_E: " + String(VeloE));
    Serial.print(" | V_D: " + String(VeloD));
    Serial.print(" | Q_Esq: " + String(contadorQuadradosEsquerda));
    Serial.print(" | Q_Dir: " + String(contadorQuadradosDireita));
    Serial.print(" | Encru: " + String(isEncruzilhada));
    Serial.print(" | Est: " + String(estadoAtual));
    Serial.println();
  }
}