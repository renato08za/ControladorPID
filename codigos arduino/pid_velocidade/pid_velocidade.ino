// ============================================================
// CONTROLE PID DE VELOCIDADE ANGULAR — Arduino Uno
// Versão: Interface Python
//
// Descrição geral:
//   Este firmware implementa um controlador PID digital para
//   controle de velocidade angular de um motor CC com encoder
//   incremental. A velocidade é medida em RPM por meio da
//   contagem de pulsos do encoder, filtrada por um filtro
//   passa-baixa exponencial e utilizada como realimentação
//   do controlador PID discretizado.
//
//   A comunicação com a interface gráfica em Python ocorre
//   via porta serial, permitindo ajuste em tempo real dos
//   ganhos Kp, Ki e Kd, do setpoint e do estado do controle.
// ============================================================


// ============================================================
//  GANHOS DO CONTROLADOR PID
//  Inicializados em zero — valores enviados pela interface
// ============================================================
float kp = 0.00000000;   // Ganho proporcional
float ki = 0.00000000;   // Ganho integral
float kd = 0.00000000;   // Ganho derivativo


// ============================================================
//  FILTRO PASSA-BAIXA EXPONENCIAL
//  Atenua ruído de quantização na estimativa de velocidade.
//  alpha próximo de 1 → maior suavização, maior atraso.
//  alpha próximo de 0 → menor suavização, mais responsivo.
//  Valor ajustado experimentalmente: alpha = 0.85
// ============================================================
float alpha = 0.85;


// ============================================================
//  VARIÁVEIS DE TEMPO
// ============================================================
unsigned long t;             // Tempo atual em ms (millis())
unsigned long t_prev  = 0;   // Tempo da amostra anterior
unsigned long t_total = 0;   // Tempo total acumulado (não utilizado no controle)


// ============================================================
//  ENCODER E PINOS DO MOTOR
//
//  O encoder incremental em quadratura gera dois sinais (A e B)
//  defasados em 90°. A leitura em quatro bordas (CHANGE em A e B)
//  quadruplica a resolução efetiva:
//    PPR nominal = 210 pulsos/rev
//    Resolução efetiva = 210 × 4 = 840 pulsos/rev
// ============================================================
const byte interruptPinA = 3;    // Canal A do encoder (interrupção externa INT1)
const byte interruptPinB = 2;    // Canal B do encoder (interrupção externa INT0)
volatile long EncoderCount = 0;  // Contador de pulsos (volatile: acessado pela ISR)
const float PPR          = 210;  // Pulsos por revolução do encoder
const int CountPerPulse  = 4;    // Fator de quadratura (contagem em 4 bordas)

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
float V_measured       = 0;    // Tensão medida nos terminais do motor


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
volatile unsigned long count = 0;   // Contador de interrupções do Timer1 (volatile: ISR)
unsigned long count_prev     = 0;   // Valor do contador na amostra anterior
float dt_ideal               = 0.01; // Período de amostragem desejado (10 ms)


// ============================================================
//  VARIÁVEIS DE CONTROLE E MEDIÇÃO
// ============================================================
float Theta       = 0;      // Posição angular atual em revoluções
float Theta_prev  = 0;      // Posição angular na amostra anterior (para cálculo de RPM)
float RPM_d       = 100;    // Setpoint de velocidade (RPM) — valor inicial
int   dt, dt2;              // Intervalo real entre amostras (ms)
float RPM         = 0;      // Velocidade filtrada (RPM)
float RPMnf       = 0;      // Velocidade não filtrada (RPM) — antes do filtro passa-baixa

#define pi 3.1416           // Constante pi (usada em conversões angulares, se necessário)

float Vmax =  5.0;  // Tensão máxima de controle (V) — limite superior do saturador
float Vmin = -5.0;  // Tensão mínima de controle (V) — limite inferior do saturador
float V    =  0.1;  // Sinal de controle calculado pelo PID (V)

float e         = 0;  // Erro atual: e = RPM_d - RPM
float e_prev    = 0;  // Erro da amostra anterior (para ação derivativa)
float inte      = 0;  // Integral acumulada do erro (para ação integral)
float inte_prev = 0;  // Integral da amostra anterior


// ============================================================
//  CONTROLE VIA INTERFACE PYTHON
// ============================================================
bool   controlActive  = false;  // Flag: habilita/desabilita a execução do PID
String inputString    = "";     // Buffer de recepção serial
bool   stringComplete = false;  // Flag: indica que uma linha completa foi recebida


// ============================================================
//  ISR — INTERRUPÇÃO DO ENCODER (CANAL A)
//
//  Chamada a cada borda (subida ou descida) do Canal A.
//  Determina o sentido de rotação comparando os estados
//  atuais dos canais A e B:
//    Canal B em LOW + Canal A em HIGH → sentido horário  (+1)
//    Canal B em LOW + Canal A em LOW  → sentido anti-horário (-1)
//    Canal B em HIGH + Canal A em HIGH → sentido anti-horário (-1)
//    Canal B em HIGH + Canal A em LOW  → sentido horário  (+1)
// ============================================================
void ISR_EncoderA() {
  bool PinB = digitalRead(interruptPinB);
  bool PinA = digitalRead(interruptPinA);

  if (PinB == LOW) {
    if (PinA == HIGH) EncoderCount++;
    else              EncoderCount--;
  } else {
    if (PinA == HIGH) EncoderCount--;
    else              EncoderCount++;
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
  bool PinB = digitalRead(interruptPinA);
  bool PinA = digitalRead(interruptPinB);

  if (PinA == LOW) {
    if (PinB == HIGH) EncoderCount--;
    else              EncoderCount++;
  } else {
    if (PinB == HIGH) EncoderCount++;
    else              EncoderCount--;
  }
}


// ============================================================
//  LEITURA DA TENSÃO DO MOTOR
//
//  Converte o valor bruto do ADC (0–1023) para tensão (V):
//    V = (leitura_ADC × Vref) / 1024
// ============================================================
float readMotorVoltage() {
  int analogValue = analogRead(voltagePin);
  return (analogValue * Vref) / 1024.0;
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
  int PWMval = int(255 * abs(V) / Vmax);
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
  int PWMval = int(255 * abs(V) / Vmax);
  if (PWMval > 255) PWMval = 255;
  return PWMval;
}


// ============================================================
//  PROCESSAMENTO DE COMANDOS DA INTERFACE PYTHON
//
//  Protocolo serial baseado em strings de texto:
//    TEST          → confirma conexão e informa formato dos dados
//    START         → habilita o controlador PID
//    STOP          → desabilita o controlador e para o motor
//    SETPID:kp,ki,kd,sp → atualiza ganhos e setpoint
//    GETPARAMS     → retorna ganhos e setpoint atuais
//    GETMODE       → informa o modo de operação (velocity)
// ============================================================
void processCommand(String command) {
  command.trim();  // Remove espaços e caracteres de controle (\r, \n)

  if (command == "TEST") {
    // Responde à tentativa de conexão da interface
    Serial.println("ARDUINO:Conexao estabelecida com sucesso!");
    Serial.println("ARDUINO:Controlador PID pronto.");
    Serial.println("FORMATO:TEMPO_MS,RPM,SETPOINT,TENSAO_V,PWM,ERRO,DT_MS");
  }
  else if (command == "START") {
    // Zera integradores e habilita o controle
    inte_prev     = 0;
    e_prev        = 0;
    controlActive = true;
    Serial.println("STATUS:Controle PID iniciado");
  }
  else if (command == "STOP") {
    // Desabilita o controle, para o motor e zera integradores
    controlActive = false;
    V             = 0;
    inte_prev     = 0;
    e_prev        = 0;
    WriteDriverVoltage(0, Vmax);
    Serial.println("STATUS:Controle PID parado");
  }
  else if (command.startsWith("SETPID:")) {
    // Extrai os quatro parâmetros separados por vírgula
    // Formato: SETPID:kp,ki,kd,setpoint
    String params = command.substring(7);
    int idx1 = params.indexOf(',');
    int idx2 = params.indexOf(',', idx1 + 1);
    int idx3 = params.indexOf(',', idx2 + 1);

    if (idx1 > 0 && idx2 > 0 && idx3 > 0) {
      kp    = params.substring(0, idx1).toFloat();
      ki    = params.substring(idx1 + 1, idx2).toFloat();
      kd    = params.substring(idx2 + 1, idx3).toFloat();
      RPM_d = params.substring(idx3 + 1).toFloat();

      Serial.print("COMANDO:Parametros atualizados - Kp=");
      Serial.print(kp, 8);
      Serial.print(" Ki=");
      Serial.print(ki, 8);
      Serial.print(" Kd=");
      Serial.print(kd, 8);
      Serial.print(" Setpoint=");
      Serial.println(RPM_d, 1);
    }
  }
  else if (command == "GETPARAMS") {
    // Retorna os parâmetros atuais para a interface
    Serial.print("PARAMS:");
    Serial.print(kp, 8);   Serial.print(",");
    Serial.print(ki, 8);   Serial.print(",");
    Serial.print(kd, 8);   Serial.print(",");
    Serial.println(RPM_d, 1);
  }
  else if (command == "GETMODE") {
    // Informa à interface que este firmware opera em modo velocidade
    Serial.println("MODE:velocity");
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
  pinMode(voltagePin, INPUT);

  // Anexa as rotinas de interrupção aos canais do encoder
  // CHANGE: dispara tanto na subida quanto na descida (quadratura completa)
  attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR_EncoderB, CHANGE);

  // Configura pinos de saída do driver de potência
  pinMode(DirPin1, OUTPUT);
  pinMode(DirPin2, OUTPUT);

  // ---- Configuração do Timer1 no modo CTC (Clear Timer on Compare Match) ----
  cli();          // Desabilita interrupções globais durante a configuração
  TCCR1A = 0;    // Zera registrador de controle A
  TCCR1B = 0;    // Zera registrador de controle B
  TCNT1  = 0;    // Zera contador do Timer1

  // Define o valor de comparação para gerar interrupção a cada 10 ms:
  //   OCR1A = (f_clk / (f_s × prescaler)) - 1 = (16e6 / (100 × 64)) - 1 = 2499
  OCR1A = ((16000000 / ((1 / dt_ideal) * 64)) - 1);

  TCCR1B |= (1 << WGM12);           // Modo CTC: zera o contador ao atingir OCR1A
  TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler = 64 (CS11=1, CS10=1)
  TIMSK1 |= (1 << OCIE1A);          // Habilita interrupção por comparação do Timer1
  sei();          // Reabilita interrupções globais

  delay(1000);   // Aguarda estabilização do sistema

  Serial.println("ARDUINO:Sistema iniciado - Aguardando comandos");
  Serial.println("FORMATO:TEMPO_MS,RPM,SETPOINT,TENSAO_V,PWM,ERRO,DT_MS");

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
    t   = millis();           // Captura o tempo atual
    dt2 = (t - t_prev);      // Intervalo real entre amostras (ms) — usado no envio serial
    dt  = (t - t_prev);      // Intervalo real — usado nos cálculos do PID

    // Calcula a posição angular em revoluções a partir da contagem do encoder
    // Theta = pulsos / (PPR × fator_quadratura)
    Theta = EncoderCount / (PPR * CountPerPulse);

    // ---- Cálculo da velocidade angular (RPM) ----
    // Diferença de posição entre amostras consecutivas dividida pelo intervalo de tempo,
    // convertida de rev/s para RPM multiplicando por 60:
    //   RPMnf = (ΔTheta / Δt) × 60
    RPMnf = (Theta - Theta_prev) / (dt / 1000.0) * 60.0;

    // Aplica filtro passa-baixa exponencial de primeira ordem para atenuar ruído:
    //   RPM[k] = α × RPM[k-1] + (1 - α) × RPMnf[k]
    RPM = alpha * RPM + (1.0 - alpha) * RPMnf;

    // ---- Controlador PID (executado apenas se controle ativo) ----
    if (controlActive) {

      // Erro: diferença entre setpoint e velocidade medida
      e = RPM_d - RPM;

      // Ação integral — método de Tustin (regra dos trapézios):
      //   inte[k] = inte[k-1] + (dt/2) × (e[k] + e[k-1])
      inte = inte_prev + (dt * (e + e_prev) / 2.0);

      // Sinal de controle PID:
      //   V = Kp × e + Ki × inte + Kd × (Δe / Δt)
      V = kp * e + ki * inte + (kd * (e - e_prev) / dt);

      // Saturador: limita o sinal de controle à faixa [Vmin, Vmax]
      // Evita que o controlador solicite tensões além da capacidade do hardware
      if (V > Vmax) V = Vmax;
      if (V < Vmin) V = Vmin;

      // Aplica o sinal de controle ao motor via ponte H
      WriteDriverVoltage(V, Vmax);

      // Atualiza variáveis anteriores para o próximo ciclo
      e_prev    = e;
      inte_prev = inte;
    }

    // ---- Leitura da tensão nos terminais do motor ----
    V_measured = readMotorVoltage();

    // ---- Transmissão de dados para a interface Python ----
    // Formato: TEMPO_MS,RPM,SETPOINT,TENSAO_V,PWM,ERRO,DT_MS
    int pwmValue = getPWMValue(V, Vmax);
    Serial.print(t);              Serial.print(",");
    Serial.print(RPM, 2);         Serial.print(",");
    Serial.print(RPM_d, 1);       Serial.print(",");
    Serial.print(V_measured, 3);  Serial.print(",");
    Serial.print(pwmValue);       Serial.print(",");
    Serial.print(e, 2);           Serial.print(",");
    Serial.println(dt2);

    // ---- Atualiza valores anteriores para o próximo ciclo ----
    Theta_prev = Theta;
    count_prev = count;
    t_prev     = t;

  } // fim do if (count > count_prev)
}


// ============================================================
//  ISR — INTERRUPÇÃO DO TIMER1
//
//  Chamada automaticamente a cada 10 ms pelo hardware.
//  Incrementa o contador 'count', que sinaliza ao loop
//  principal que um novo ciclo de controle deve ser executado.
//  Separar o disparo do timer da lógica de controle garante
//  que o período de amostragem seja independente do tempo
//  de execução do loop.
// ============================================================
ISR(TIMER1_COMPA_vect) {
  count++;
}
