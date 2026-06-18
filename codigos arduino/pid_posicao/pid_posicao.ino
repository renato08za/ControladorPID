// ============================================================
// CONTROLE PID DE POSIÇÃO ANGULAR — Arduino Uno
// Versão: Interface Python
//
// Descrição geral:
//   Este firmware implementa um controlador PID digital para
//   controle de posição angular de um motor CC com encoder
//   incremental. A posição é obtida pela contagem acumulada
//   de pulsos do encoder, convertida em graus, e utilizada
//   como realimentação do controlador PID discretizado.
//
//   O setpoint pode ser definido de forma absoluta (ângulo
//   desejado em graus) ou incremental (deslocamento relativo
//   à posição atual), ambos enviados pela interface Python.
//
//   A comunicação com a interface gráfica em Python ocorre
//   via porta serial, permitindo ajuste em tempo real dos
//   ganhos Kp, Ki e Kd, do setpoint e do estado do controle.
// ============================================================


// ============================================================
//  GANHOS DO CONTROLADOR PID
//  Inicializados em zero — valores enviados pela interface
// ============================================================
float kp = 0.00000;   // Ganho proporcional
float ki = 0.00000;   // Ganho integral
float kd = 0.00000;   // Ganho derivativo


// ============================================================
//  ENCODER E PINOS DO MOTOR
//
//  O encoder incremental em quadratura gera dois sinais (A e B)
//  defasados em 90°. A leitura em quatro bordas (CHANGE em A e B)
//  quadruplica a resolução efetiva:
//    PPR nominal = 840 pulsos/rev (já inclui a caixa de redução)
//    Fator de quadratura = 4
//    Resolução efetiva = 840 × 4 = 3360 pulsos/rev
// ============================================================
const byte interruptPinA = 3;    // Canal A do encoder (interrupção externa INT1)
const byte interruptPinB = 2;    // Canal B do encoder (interrupção externa INT0)
volatile long EncoderCount = 0;  // Contador acumulado de pulsos (volatile: acessado pela ISR)
const float PPR           = 840.0;  // Pulsos por revolução (com caixa de redução)
const int   CountPerPulse = 4;      // Fator de quadratura (contagem em 4 bordas)

const byte PWMPin  = 6;   // Pino de saída PWM para a ponte H
const byte DirPin1 = 7;   // Pino de direção 1 da ponte H L298N
const byte DirPin2 = 8;   // Pino de direção 2 da ponte H L298N


// ============================================================
//  LEITURA DE TENSÃO DO MOTOR
//  Leitura analógica da tensão nos terminais do motor via
//  divisor resistivo conectado ao pino A0.
// ============================================================
const byte  voltagePin = A0;   // Pino analógico de leitura de tensão
const float Vref       = 5.0;  // Tensão de referência do ADC (5 V)
float Vmax             =  5.0; // Tensão máxima de controle (V)
float Vmin             = -5.0; // Tensão mínima de controle (V)


// ============================================================
//  TIMER 1 — GERAÇÃO DO PERÍODO DE AMOSTRAGEM
//
//  O Timer1 do ATmega328P é configurado no modo CTC com
//  prescaler de 64, gerando interrupções periódicas a cada
//  10 ms de forma independente do laço principal.
//
//  Cálculo do registrador OCR1A:
//    OCR1A = (f_clk / (f_s × N)) - 1
//    OCR1A = (16.000.000 / (100 × 64)) - 1 = 2499
// ============================================================
const float dt_ideal         = 0.01;  // Período de amostragem desejado (10 ms)
volatile unsigned long count = 0;     // Contador de interrupções do Timer1 (volatile: ISR)
unsigned long count_prev     = 0;     // Valor do contador na amostra anterior


// ============================================================
//  VARIÁVEIS DE CONTROLE E MEDIÇÃO
// ============================================================
float Theta_d   = 0.0;  // Setpoint de posição angular (graus) — alvo absoluto
float Theta_deg = 0.0;  // Posição angular medida atual (graus)

float e         = 0.0;  // Erro atual: e = Theta_d - Theta_deg
float e_prev    = 0.0;  // Erro da amostra anterior (para ação derivativa)
float inte      = 0.0;  // Integral acumulada do erro (para ação integral)
float inte_prev = 0.0;  // Integral da amostra anterior
float V_control = 0.0;  // Sinal de controle calculado pelo PID (V)
float V_measured= 0.0;  // Tensão medida nos terminais do motor (V)

unsigned long t        = 0;  // Tempo atual em ms
unsigned long t_prev   = 0;  // Tempo da amostra anterior
int dt, dt2;                 // Intervalo real entre amostras (ms)

#define pi 3.1416            // Constante pi


// ============================================================
//  CONTROLE VIA INTERFACE PYTHON
// ============================================================
bool   motorEnabled   = false;  // Flag: habilita/desabilita o acionamento do motor
String inputString    = "";     // Buffer de recepção serial
bool   stringComplete = false;  // Flag: indica que uma linha completa foi recebida


// ============================================================
//  ISR — INTERRUPÇÃO DO ENCODER (CANAL A)
//
//  Chamada a cada borda (subida ou descida) do Canal A.
//  Determina o sentido de rotação comparando os estados
//  atuais dos canais A e B:
//    Canal B em LOW  + Canal A em HIGH → sentido horário  (+1)
//    Canal B em LOW  + Canal A em LOW  → sentido anti-horário (-1)
//    Canal B em HIGH + Canal A em HIGH → sentido anti-horário (-1)
//    Canal B em HIGH + Canal A em LOW  → sentido horário  (+1)
// ============================================================
void ISR_EncoderA() {
  bool pinB = digitalRead(interruptPinB);
  bool pinA = digitalRead(interruptPinA);
  if (pinB == LOW) {
    EncoderCount += (pinA == HIGH) ? 1 : -1;
  } else {
    EncoderCount += (pinA == HIGH) ? -1 : 1;
  }
}


// ============================================================
//  ISR — INTERRUPÇÃO DO ENCODER (CANAL B)
//
//  Chamada a cada borda (subida ou descida) do Canal B.
//  Complementa a ISR do Canal A para garantir a contagem
//  em quatro bordas (quadratura completa).
// ============================================================
void ISR_EncoderB() {
  bool pinA = digitalRead(interruptPinA);
  bool pinB = digitalRead(interruptPinB);
  if (pinA == LOW) {
    EncoderCount += (pinB == HIGH) ? -1 : 1;
  } else {
    EncoderCount += (pinB == HIGH) ? 1 : -1;
  }
}


// ============================================================
//  ISR — INTERRUPÇÃO DO TIMER1
//
//  Chamada automaticamente a cada 10 ms pelo hardware.
//  Incrementa o contador 'count', sinalizando ao loop
//  principal que um novo ciclo de controle deve ser executado.
// ============================================================
ISR(TIMER1_COMPA_vect) {
  count++;
}


// ============================================================
//  ACIONAMENTO DO MOTOR — PONTE H L298N
//
//  Converte a tensão de controle V em sinal PWM (0–255)
//  e define o sentido de rotação pelos pinos de direção:
//    V > 0 → sentido horário  (DirPin1=HIGH, DirPin2=LOW)
//    V < 0 → sentido anti-horário (DirPin1=LOW, DirPin2=HIGH)
//    V = 0 → motor parado    (DirPin1=LOW,  DirPin2=LOW)
//
//  PWM = (255 × |V|) / Vmax
// ============================================================
void WriteDriverVoltage(float V, float Vmax) {
  int PWMval = (int)(255.0 * abs(V) / Vmax);
  if (PWMval > 255) PWMval = 255;

  if (V > 0) {
    digitalWrite(DirPin1, HIGH);
    digitalWrite(DirPin2, LOW);
  } else if (V < 0) {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, HIGH);
  } else {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, LOW);
  }
  analogWrite(PWMPin, PWMval);
}


// ============================================================
//  CÁLCULO DO VALOR PWM (para envio à interface)
//
//  Retorna o valor inteiro do PWM sem acionar o motor,
//  utilizado apenas para transmissão de dados à interface.
// ============================================================
int getPWMValue(float V, float Vmax) {
  int PWMval = (int)(255.0 * abs(V) / Vmax);
  if (PWMval > 255) PWMval = 255;
  return PWMval;
}


// ============================================================
//  LEITURA DA TENSÃO DO MOTOR
//
//  Converte o valor bruto do ADC (0–1023) para tensão (V):
//    V = (leitura_ADC × Vref) / 1024
// ============================================================
float readMotorVoltage() {
  int raw = analogRead(voltagePin);
  return (raw * Vref) / 1024.0;
}


// ============================================================
//  PROCESSAMENTO DE COMANDOS DA INTERFACE PYTHON
//
//  Protocolo serial baseado em strings de texto:
//    TEST               → confirma conexão e informa formato dos dados
//    START              → habilita o motor
//    STOP               → desabilita o motor e o para
//    SETPID:kp,ki,kd,sp → atualiza ganhos e setpoint absoluto
//    SETPOS:incremento  → desloca o setpoint relativamente à posição atual
//    GETPARAMS          → retorna ganhos e setpoint atuais
//    RESETPOS           → zera encoder, setpoint e integradores
//    GETMODE            → informa o modo de operação (position)
// ============================================================
void processCommand(String command) {
  command.trim();  // Remove espaços e caracteres de controle (\r, \n)

  if (command == "TEST") {
    // Responde à tentativa de conexão da interface
    Serial.println("ARDUINO:Conexao estabelecida com sucesso!");
    Serial.println("ARDUINO:Controlador PID de Posicao pronto.");
    Serial.println("FORMATO:TEMPO_MS,THETA_DEG,THETA_D,TENSAO_V,PWM,ERRO,DT_MS");
  }
  else if (command == "START") {
    // Habilita o acionamento do motor
    motorEnabled = true;
    Serial.println("STATUS:Motor habilitado");
  }
  else if (command == "STOP") {
    // Desabilita o motor e o para imediatamente
    motorEnabled = false;
    WriteDriverVoltage(0, Vmax);
    Serial.println("STATUS:Motor parado");
  }
  else if (command.startsWith("SETPID:")) {
    // Atualiza ganhos e define novo setpoint absoluto
    // Formato: SETPID:kp,ki,kd,theta_d
    String params = command.substring(7);
    int idx1 = params.indexOf(',');
    int idx2 = params.indexOf(',', idx1 + 1);
    int idx3 = params.indexOf(',', idx2 + 1);

    if (idx1 > 0 && idx2 > 0 && idx3 > 0) {
      kp = params.substring(0, idx1).toFloat();
      ki = params.substring(idx1 + 1, idx2).toFloat();
      kd = params.substring(idx2 + 1, idx3).toFloat();

      // Se o novo alvo for diferente do atual, atualiza e zera integradores
      // para evitar transitório indesejado na troca de setpoint
      float novo_alvo = params.substring(idx3 + 1).toFloat();
      if (novo_alvo != Theta_d) {
        Theta_d   = novo_alvo;
        inte      = 0.0;
        inte_prev = 0.0;
        e_prev    = 0.0;
      }

      Serial.print("COMANDO:Parametros atualizados - Kp=");
      Serial.print(kp, 8);
      Serial.print(" Ki=");
      Serial.print(ki, 8);
      Serial.print(" Kd=");
      Serial.print(kd, 8);
      Serial.print(" Theta_d=");
      Serial.println(Theta_d, 2);
    }
  }
  else if (command.startsWith("SETPOS:")) {
    // Aplica um deslocamento incremental ao setpoint atual
    // Útil para comandos relativos: "avance 30 graus"
    // Formato: SETPOS:valor_incremental
    float incremento = command.substring(7).toFloat();
    Theta_d   += incremento;   // Soma o incremento ao alvo atual
    inte       = 0.0;          // Zera integradores para evitar transitório
    inte_prev  = 0.0;
    e_prev     = 0.0;

    Serial.print("COMANDO:Incremento=");
    Serial.print(incremento, 2);
    Serial.print(" Novo alvo=");
    Serial.println(Theta_d, 2);
  }
  else if (command == "GETPARAMS") {
    // Retorna os parâmetros atuais para a interface
    Serial.print("PARAMS:");
    Serial.print(kp, 8);      Serial.print(",");
    Serial.print(ki, 8);      Serial.print(",");
    Serial.print(kd, 8);      Serial.print(",");
    Serial.println(Theta_d, 2);
  }
  else if (command == "RESETPOS") {
    // Zera o encoder, o setpoint e todos os estados do controlador
    // Utilizado para reiniciar o sistema sem desligar o Arduino
    EncoderCount = 0;
    Theta_d      = 0.0;
    Theta_deg    = 0.0;
    inte         = 0.0;
    inte_prev    = 0.0;
    e_prev       = 0.0;
    Serial.println("STATUS:Posicao zerada");
  }
  else if (command == "GETMODE") {
    // Informa à interface que este firmware opera em modo posição
    Serial.println("MODE:position");
  }
}


// ============================================================
//  SETUP — Inicialização do sistema
// ============================================================
void setup() {
  Serial.begin(115200);  // Inicializa comunicação serial a 115200 baud

  // Configura pinos do encoder com resistores de pull-up internos
  pinMode(interruptPinA, INPUT_PULLUP);
  pinMode(interruptPinB, INPUT_PULLUP);

  // Anexa as rotinas de interrupção aos canais do encoder
  // CHANGE: dispara tanto na subida quanto na descida (quadratura completa)
  attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR_EncoderB, CHANGE);

  // Configura pinos de saída do driver de potência
  pinMode(DirPin1, OUTPUT);
  pinMode(DirPin2, OUTPUT);
  pinMode(PWMPin,  OUTPUT);
  pinMode(voltagePin, INPUT);

  // ---- Configuração do Timer1 no modo CTC (Clear Timer on Compare Match) ----
  cli();          // Desabilita interrupções globais durante a configuração
  TCCR1A = 0;    // Zera registrador de controle A
  TCCR1B = 0;    // Zera registrador de controle B
  TCNT1  = 0;    // Zera contador do Timer1

  // Define o valor de comparação para gerar interrupção a cada 10 ms:
  //   OCR1A = (f_clk / (f_s × prescaler)) - 1 = (16e6 / (100 × 64)) - 1 = 2499
  OCR1A = ((16000000.0 / ((1.0 / dt_ideal) * 64.0)) - 1);

  TCCR1B |= (1 << WGM12);              // Modo CTC: zera o contador ao atingir OCR1A
  TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler = 64 (CS11=1, CS10=1)
  TIMSK1 |= (1 << OCIE1A);             // Habilita interrupção por comparação do Timer1
  sei();          // Reabilita interrupções globais

  delay(1000);   // Aguarda estabilização do sistema

  Serial.println("ARDUINO:Sistema iniciado - Aguardando comandos");
  Serial.println("FORMATO:TEMPO_MS,THETA_DEG,THETA_D,TENSAO_V,PWM,ERRO,DT_MS");

  inputString.reserve(200);  // Pré-aloca memória para o buffer serial
}


// ============================================================
//  LOOP PRINCIPAL
// ============================================================
void loop() {

  // ---- Recepção de comandos seriais ----
  // Lê caractere por caractere até encontrar '\n' (fim de linha)
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;  // Sinaliza que uma linha completa foi recebida
    }
  }

  // Processa o comando recebido e limpa o buffer
  if (stringComplete) {
    processCommand(inputString);
    inputString    = "";
    stringComplete = false;
  }

  // ---- Ciclo de controle (executado a cada 10 ms pelo Timer1) ----
  if (count > count_prev) {
    t   = millis();       // Captura o tempo atual
    dt2 = (t - t_prev);  // Intervalo real entre amostras (ms) — usado no envio serial
    dt  = (t - t_prev);  // Intervalo real — usado nos cálculos do PID

    // ---- Cálculo da posição angular em graus ----
    // Theta_deg = (pulsos acumulados / pulsos por revolução total) × 360°
    //   pulsos por revolução total = PPR × CountPerPulse = 840 × 4 = 3360
    Theta_deg = (EncoderCount / (PPR * CountPerPulse)) * 360.0;

    // ---- Cálculo do erro de posição ----
    e = Theta_d - Theta_deg;

    // Zona morta: erros menores que ±0,5° são tratados como zero
    // Evita oscilações em torno do setpoint causadas por folgas mecânicas
    if (e < 0.5 && e > -0.5) {
      e = 0;
    }

    // ---- Controlador PID discretizado ----
    // Ação integral — método de Tustin (regra dos trapézios):
    //   inte[k] = inte[k-1] + (dt/2) × (e[k] + e[k-1])
    inte = inte_prev + (dt * (e + e_prev) / 2);

    // Sinal de controle PID:
    //   V = Kp × e + Ki × inte + Kd × (Δe / Δt)
    V_control = kp * e + ki * inte + kd * (e - e_prev) / dt;

    // Anti-windup (comentado — disponível para implementação futura):
    // Limita o sinal de controle para evitar saturação prolongada do integrador
    // if (V_control > Vmax) V_control = Vmax;
    // if (V_control < Vmin) V_control = Vmin;

    // ---- Acionamento do motor ----
    // Motor só é acionado se habilitado pela interface (comando START)
    if (motorEnabled) {
      WriteDriverVoltage(V_control, Vmax);
    } else {
      WriteDriverVoltage(0, Vmax);  // Motor parado se desabilitado
    }

    // ---- Leitura da tensão nos terminais do motor ----
    V_measured = readMotorVoltage();

    // ---- Transmissão de dados para a interface Python ----
    // Formato: TEMPO_MS,THETA_DEG,THETA_D,TENSAO_V,PWM,ERRO,DT_MS
    int pwmValue = getPWMValue(V_control, Vmax);
    Serial.print(t);              Serial.print(",");
    Serial.print(Theta_deg, 2);   Serial.print(",");
    Serial.print(Theta_d, 2);     Serial.print(",");
    Serial.print(V_measured, 3);  Serial.print(",");
    Serial.print(pwmValue);       Serial.print(",");
    Serial.print(e, 2);           Serial.print(",");
    Serial.println(dt2);

    // ---- Atualiza variáveis anteriores para o próximo ciclo ----
    count_prev = count;
    t_prev     = t;
    inte_prev  = inte;
    e_prev     = e;

  } // fim do if (count > count_prev)
}
