/*
 *  Autor: Diego Amaral Castro
 *  Data: Fevereiro/Março de 2023
 */

/*
 *  O seguinte código eh utilizado num arduino DUE, cujo papel eh a de um controlador de um motor BLDC
 *  O Arduino recebe sinal dos sensores de efeito hall e comuta os mosfets de acordo com os valores lidos
 *  O motor eh ligado apenas sob certas condicoes de interruptores e fins de curso
 *  A entrada para o motor eh uma entrada rampa, com incremento gradual
 */

// Biblioteca Arduino.h adicionada para aumentar compatibilidade com outros editores
#include <Arduino.h>

// Relação de portas Shield Controlador -> Powerdriver (sinais de saida para os gates dos mosfets)
#define MOSFET_A_HIGH 5
#define MOSFET_B_HIGH 6
#define MOSFET_C_HIGH 7

#define MOSFET_A_LOW 9
#define MOSFET_B_LOW 10
#define MOSFET_C_LOW 11

// Duty cycle do PWM maximo no gate
#define PWM_MAX 99

// Portas definidas DUE
#define hallA 4           // Entrada do sinal do sensor de efeito hall A do motor
#define hallB 3           // Entrada do sinal do sensor de efeito hall B do motor
#define hallC 2           // Entrada do sinal do sensor de efeito hall C do motor

#define botao1_pedal 17   // PEDAL INICIO -CHAVE DA FRENTE COM CABO AZUL ACIONAMENTO MOTOR - Acionado pelo pedal de aceleracao. Fim de Curso [Dianteiro]. Entrada do botão 1 de aceleracao (Pedal meio pressionado) 
#define botao2_pedal 18   // PEDAL FIM -  Acionado pelo pedal de aceleracao. Fim de Curso [Traseiro]. Entrada do botão 2 de aceleracao (Pedal Completamente pressionado)

#define chave_controle 16 // Configura o valor inicial do PWM nos mosfets. Sendo o valor mais baixo para arrancada e o maior para ao longo da corrida
#define acelerador 8      // Para ligar o carro sem apertar o pedal. Alternativa para não precisar apertar o pedal ate o fundo
#define sinal_corrente 22 // Desativa a alimentação se a corrente estiver muito alta
#define shutdownIR 1      // Pino que desativa o Gate Driver quando o arduino reseta ou liga

int hallA_estado;                                  // Estado do sensor de efeito hall A do motor BLDC
int hallB_estado;                                  // Estado do sensor de efeito hall B do motor BLDC
int hallC_estado;                                  // Estado do sensor de efeito hall C do motor BLDC
int hall_val = 1;                                  // Estado dos tres halls (Fase do motor)
int botao1_pedal_anterior = 3;                     // Estado anterior do fim de curso dianteiro
int botao2_pedal_anterior = 3;                     // Estado anterior do fim de curso traseiro
int estado_controle, estado_controle_ant = LOW;    // Serve para zerar o duty cycle para o início da rampa
unsigned long tempo_Atual = 0, tempo_Anterior = 0; // Servem para incrementar o PWM depois de x microsegundos

// Variaveis Relevantes
int incremento_rampa = 0; // Duty cycle em % , de 0 a 100%
int inicio_rampa = 0;     // Valor inicial da rampa


int inicio_rampa_HIGH = 30; //Variável que define o valor inicial da rampa em caso de HIGH
int inicio_rampa_LOW = 70; // Variável que define o valor inicial da rampa em caso de LOW
int incremento_tempo = 65; // Variável que permite mudar o valor do tempo de incremento
int incremento_para_rampa = 1; // Variável que permite alterar a amplitude do incremento da rampaSS


int pwwm = 0;             // Armazena o PWM que ira para os MOSFETs (0 a 100%)
int PWM9 = 0;             // Variável que atualiza constantemente o Duty Cycle do pino 9 (0 a 100%)
int PWM10 = 0;            // Variável que atualiza constantemente o Duty Cycle do pino 10 (0 a 100%)
int PWM11 = 0;            // Variável que atualiza constantemente o Duty Cycle do pino 11 (0 a 100%)
int freq = 1000;           // Frequência do PWM em Hz, esse valor não será mudado ao longo da execução do código
int TC_RC = 42000000/freq;// Variável utilizada para definir a frequência dos pinos que usam Timer Counter


void ConfigurePin9(int freq){
  pinMode(9, OUTPUT);
  analogWrite(9, 0);    
  PWMC_ConfigureClocks(freq * PWM_MAX_DUTY_CYCLE , 0, VARIANT_MCK);
}

void  ConfigurePin10(){
  /* Passos para configurar o PWM com o Timer Counter
   * 1º Identifique o Instance Name TIOA7, TIOA8 etc. Você pode encontrar isso no DUE Pinout Diagram
   * 2º 2º Identifique o Instance (TC0, TC1, TC2, TC3, TC4 etc.). O número é o mesmo número do Instance Name (TIOA7 tem instance TC7)
   * 3º Identifique o I/O line. Você também encontra na mesma tabela da página 858
   * 4º Identifique o Peripheral. Você também encontra na mesma tabela da página 858
   * 5º Identifique o Instance ID. Você pode encontrá-lo na tabela da página 38
   * 6º Identifique o TCx->CHANNEL[y]. Você pode encontrá-lo na página 31 do datasheet. TC5 por exemplo, é TC1->CHANNEL[2]
   */

  /*
   * 1º Instance Name: TIOB7
   * 2º Instance: TC7
   * 3º I/O line PC29
   * 4º Peripheral: B
   * 5º Instance ID: 34
   * 6º TC2->TC_CHANNEL[1]
   */ 

                                                            // x significa o grupo do Instance ID, de 0 a 31, temos x=0, de 32 a 63 temos x = 1
//PMC->PMC_PCERx |= PMC_PCERx_PIDyy;                        // yy significa o valor do Instance ID
  PMC->PMC_PCER1 |= PMC_PCER1_PID34;                        // Enable TC7 (TC2 Channel 1)

//PIOx->PIO_PDR |= PIO_PDR_Pyy;                             // x significa a segunda letra do I/O line, yy significa o número do I/O line
  PIOC->PIO_PDR |= PIO_PDR_P29;                             // Disable the GPIO on the corresponding pins

                                                            // xx significa o I/O line
//PIOx->PIO_ABSR |= PIO_xx_yy                               // yy significa o Instance Name
  PIOC->PIO_ABSR |= PIO_PC29B_TIOB7;                        // Switch the multiplexer to peripheral B for TIOB7

//TCx->TC_CHANNEL[y]                                        // Apenas por o que encontrou para o 6º caso
  TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1    // Set the timer clock to TCLK1 (MCK/2 = 84MHz/2 = 42MHz)
                              | TC_CMR_WAVE                 // Enable wave mode
                              | TC_CMR_WAVSEL_UP_RC         // Count up with automatic trigger on RC compare
                              | TC_CMR_BCPB_CLEAR           // Clear TIOB on counter match with RB0t
                              | TC_CMR_BCPC_SET             // Set TIOB on counter match with RC0
                              | TC_CMR_EEVT_XC0;            // Set event selection to XC0 to make TIOB an output (Apenas se for um TIOB, TIOA não precisa)

  TC2->TC_CHANNEL[1].TC_RC = TC_RC;                         // Frequency = (Mck/2)/TC_RC  rate ~ 500 Hz
  TC2->TC_CHANNEL[1].TC_RB = 42000;                         // Duty cycle = (TC_RB/TC_RC) * 100  % = 50%

  TC2->TC_CHANNEL[1].TC_IER = TC_IER_CPCS;                  // Enable interrupts for TC7 (TC2 Channel 1)
  NVIC_EnableIRQ(TC7_IRQn);                                 // Connect TC7 to Nested Vector Interrupt Controller (NVIC)

  TC2->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;  // Enable the timer TC7
  REG_TC2_IER1 |= TC_IER_CPCS;                              // Enable interrupts for TC7 (TC2 Channel 1)
}

void  ConfigurePin11(){
  /* Passos para configurar o PWM com o Timer Counter
   * 1º Identifique o Instance Name TIOA7, TIOA8 etc. Você pode encontrar isso no DUE Pinout Diagram
   * 2º 2º Identifique o Instance (TC0, TC1, TC2, TC3, TC4 etc.). O número é o mesmo número do Instance Name (TIOA7 tem instance TC7)
   * 3º Identifique o I/O line. Você também encontra na mesma tabela da página 858
   * 4º Identifique o Peripheral. Você também encontra na mesma tabela da página 858
   * 5º Identifique o Instance ID. Você pode encontrá-lo na tabela da página 38
   * 6º Identifique o TCx->CHANNEL[y]. Você pode encontrá-lo na página 31 do datasheet. TC5 por exemplo, é TC1->CHANNEL[2]
   */

  /*
   * 1º Instance Name: TIOA8
   * 2º Instance: TC8
   * 3º I/O line PD7
   * 4º Peripheral: B
   * 5º Instance ID: 35
   * 6º TC2->TC_CHANNEL[2]
   */
   
                                                            // x significa o grupo do Instance ID, de 0 a 31, temos x=0, de 32 a 63 temos x = 1
//PMC->PMC_PCERx |= PMC_PCERx_PIDyy;                        // yy significa o valor do Instance ID 
  PMC->PMC_PCER1 |= PMC_PCER1_PID35;                        // Enable TC8 (TC2 Channel 2)

//PIOx->PIO_PDR |= PIO_PDR_Pyy;                             // x significa a segunda letra do I/O line, yy significa o número do I/O line
  PIOD->PIO_PDR |= PIO_PDR_P7;                              // Disable the GPIO on the corresponding pins

                                                            // xx significa o I/O line
//PIOx->PIO_ABSR |= PIO_xx_yy                               // yy significa o Instance Name  
  PIOD->PIO_ABSR |= PIO_PD7B_TIOA8;                         // Switch the multiplexer to peripheral B for TIOA8

//TCx->TC_CHANNEL[y]                                        // Apenas por o que encontrou para o 6º caso
  TC2->TC_CHANNEL[2].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1    // Set the timer clock to TCLK1 (MCK/2 = 84MHz/2 = 42MHz)
                              | TC_CMR_WAVE                 // Enable wave mode
                              | TC_CMR_WAVSEL_UP_RC         // Count up with automatic trigger on RC compare
                              | TC_CMR_ACPA_CLEAR           // Clear TIOA on counter match with RA0
                              | TC_CMR_ACPC_SET;            // Set TIOA on counter match with RC0

  TC2->TC_CHANNEL[2].TC_RC = TC_RC;                         // Frequency = (Mck/2)/TC_RC  rate ~ 500 Hz
  TC2->TC_CHANNEL[2].TC_RA = 42000;                         // Duty cycle = (TC_RA/TC_RC) * 100  % = 50%

  TC2->TC_CHANNEL[2].TC_IER = TC_IER_CPCS;                  // Enable interrupts for TC8 (TC2 Channel 8)
  NVIC_EnableIRQ(TC8_IRQn);                                 // Connect TC8 to Nested Vector Interrupt Controller (NVIC)

  TC2->TC_CHANNEL[2].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;  // Enable the timer TC8
  REG_TC2_IER2 |= TC_IER_CPCS;                              // Enable interrupts for TC8 (TC2 Channel 2)
}

void TC7_Handler() {
  TC2->TC_CHANNEL[1].TC_SR;                                 // Read and clear status register so that the interrupt handler fires again and again
  REG_TC2_RB1 = PWM10;                                      // Update the Duty Cycle
}

void TC8_Handler() {
  TC2->TC_CHANNEL[2].TC_SR;                                 // Read and clear status register so that the interrupt handler fires again and again
  REG_TC2_RA2 = PWM11;                                      // Update the Duty Cycle
}

void DutyCyclePin9(int Duty_Cycle){
  if(Duty_Cycle == 0){
    DisablePin9();      
  } else {
    PWM9 = Duty_Cycle;
  }                                                         // Update the variable that changes the Duty Cycle of pin 9
}

void DutyCyclePin10(int Duty_Cycle){
  int x = map(Duty_Cycle, 0, 100, 0, TC_RC);
  if(Duty_Cycle == 0){
    DisablePin10();
  } else {
    PWM10 = x;                                              // Update the variable that changes the Duty Cycle of pin 10
  }                                              
}

void DutyCyclePin11(int Duty_Cycle){
  int x = map(Duty_Cycle, 0, 100, 0, TC_RC);
  if(Duty_Cycle == 0){
    DisablePin11();                                              // Update the variable that changes the Duty Cycle of pin 11
  } else {
    PWM11 = x;
  }                                          
}

void DisablePin9(){
  PWM9 = 0;
  PWMC_SetDutyCycle(PWM_INTERFACE, PWM_CH4, 0);
}

void DisablePin10(){
  PWM10 = 1;                                                // Update the Duty Cycle to the less value possible
  PIOC->PIO_PER |= PIO_PER_P29;                             // Enable the GPIO               
  PIOC->PIO_OER |= PIO_OER_P29;                             // Configure the GPIO as output
  PIOC->PIO_CODR = 1<<29;                                   // digitalWrite(10, LOW)
}

void DisablePin11(){
  PWM11 = 1;                                                // Update the Duty Cycle to the less value possible
  PIOD->PIO_PER |= PIO_PER_P7;                              // Enable the GPIO
  PIOD->PIO_OER |= PIO_OER_P7;                              // Configure the GPIO as output
  PIOD->PIO_CODR = 1<<7;                                    // digitalWrite(11, LOW)
}

void EnablePin9(){
  int x = map(PWM9, 0, 100, 0, 255);
  PWMC_SetDutyCycle(PWM_INTERFACE, PWM_CH4, x);    
}

void EnablePin10(){
  PIOC->PIO_PDR |= PIO_PDR_P29;                             // Disable the GPIO
}

void EnablePin11(){
  PIOD->PIO_PDR |= PIO_PDR_P7;                              // Disable the GPIO
}

void setup()
{
  // Entradas do Arduino MEGA
  pinMode(hallA, INPUT);
  pinMode(hallB, INPUT);
  pinMode(hallC, INPUT);
  pinMode(sinal_corrente, INPUT);
  pinMode(botao1_pedal, INPUT);
  pinMode(botao2_pedal,INPUT);
  pinMode(chave_controle,INPUT);
  pinMode(acelerador, INPUT);

  // Saídas do Arduino MEGA
  // Saídas para ativação MOSFETs do inversor trifasico
  // Saídas digitais HIGH SIDE
  pinMode(MOSFET_A_HIGH, OUTPUT);
  pinMode(MOSFET_B_HIGH, OUTPUT);
  pinMode(MOSFET_C_HIGH, OUTPUT);

  // Saidas PWM LOW SIDE
  ConfigurePin9(freq);                                      // MOSFET_A_LOW
  ConfigurePin10();                                         // MOSFET_B_LOW
  ConfigurePin11();                                         // MOSFET_C_LOW

  // Saída Shutdown
  pinMode(shutdownIR, OUTPUT);

  // Coloca as saidas dos MOSFETs e o Shutdown em low

  digitalWrite(shutdownIR, LOW);
  digitalWrite(MOSFET_A_HIGH, LOW);
  digitalWrite(MOSFET_B_HIGH, LOW);
  digitalWrite(MOSFET_C_HIGH, LOW);
  DisablePin9();
  DisablePin10();
  DisablePin11();

}
void loop() {
    PIOA->PIO_CODR = 1<<9;                                    // Pino ShutdownIR em 0
    tempo_Atual = millis();
    estado_controle = digitalRead(chave_controle);
  
    // Se houver mudanca da chave controle, entao a rampa eh "zerada". Voltando a incrementar do zero
    if(estado_controle != estado_controle_ant){
        incremento_rampa = 0;
        estado_controle_ant = estado_controle;
    }
  
    // Configura o valor do inicio da rampa de acordo com o selecionado na chave_controle
    // O valor mais baixo eh somente para a arrancada, e o mais alto eh para a pista
    // Os valores podem mudar conforme variar o peso do carro e do piloto
    if(digitalRead(chave_controle) == HIGH){
       inicio_rampa = inicio_rampa_HIGH;
    }
    else{
       inicio_rampa = inicio_rampa_LOW;
    }
  
    // Liga o carro se o pedal estiver completamente pressionado. Ou se o acelerador estiver ligado
    if( digitalRead(acelerador) == LOW || digitalRead(botao2_pedal)){
      if (incremento_rampa < PWM_MAX - inicio_rampa){            // Se o duty cycle do PWM sobre o gate dos MOSFETs ainda eh menor do que PWM_MAX
        if ((tempo_Atual - tempo_Anterior) >= incremento_tempo){  // Incrementa de acordo com o valor estabelecido do tempo de incremento
          incremento_rampa+=incremento_para_rampa; // Aumento gradual da rampa de acordo com a variável incremento para incremento 
          tempo_Anterior = tempo_Atual;
        }  
      }
      pwwm = incremento_rampa + inicio_rampa;
    }
    else{ 
          incremento_rampa = 0;
          pwwm = 0;
    }

  hallA_estado = digitalRead(hallA); // variavel recebe o valor do estado do sensor de efeito Hall A do motor
  hallB_estado = digitalRead(hallB); // variavel recebe o valor do estado do sensor de efeito Hall B do motor
  hallC_estado = digitalRead(hallC); // variavel recebe o valor do estado do sensor de efeito Hall C do motor

  hall_val = (hallA_estado) + (2 * hallB_estado) + (4 * hallC_estado); // converte o valor dos 3 sensores Hall para números decimais

  
  switch (hall_val)
  {
    /*
     * Funcionando em sentido anti-horario, esses valores estão descritos no TCC que esta na biblioteca do Notion sobre o motor BLDC
     *
     * --------------------------------------------------------------------------------------------------------------------------
     *
     * Sequencia das fases para os mosfets ligados:
     * BH AL
     * BH CL
     * AH CL
     * AH BL
     * CH BL
     * CH AL
     *
     * --------------------------------------------------------------------------------------------------------------------------
     *
     * Eh importante garantir que todos os transistores estejam abertos (ou ao menos 5 deles) para fazer a comutacao dos MOSFETs
     * Caso isso nao seja feito, existe o risco de causar um curto-circuito na bateria
     *
     * --------------------------------------------------------------------------------------------------------------------------
     */

    // FASE 1
  case 5:
    PIOC->PIO_CODR = 1<<25;   // Pino 5 (AH) desligado
    PIOC->PIO_CODR = 1<<24;   // Pino 6 (BH) desligado
    PIOC->PIO_CODR = 1<<23;   // Pino 7 (CH) desligado
    DisablePin10();           // Pino 10 (BL) desligado
    DisablePin11();           // Pino 11 (CL) desligado

    PIOC->PIO_SODR = 1<<24;   // Pino 6 (BH) ligado
    DutyCyclePin9(pwwm);
    EnablePin9();             // Pino 9 (AL) com PWM ativo
    break;
    /* ----------------------------------------------------------------------------------------------------------------------- */
    // FASE 2
  case 1:
    PIOC->PIO_CODR = 1<<25;   // Pino 5 (AH) desligado
    PIOC->PIO_CODR = 1<<24;   // Pino 6 (BH) desligado
    PIOC->PIO_CODR = 1<<23;   // Pino 7 (CH) desligado
    DisablePin9();            // Pino 9 (AL) desligado
    DisablePin10();           // Pino 10 (BL) desligado

    PIOC->PIO_SODR = 1<<24;   // Pino 6 (BH) ligado
    DutyCyclePin11(pwwm);
    EnablePin11();            // Pino 11 (CL) com PWM ativo
    break;
    /* ----------------------------------------------------------------------------------------------------------------------- */
    // FASE 3
  case 3:
    PIOC->PIO_CODR = 1<<25;   // Pino 5 (AH) desligado
    PIOC->PIO_CODR = 1<<24;   // Pino 6 (BH) desligado
    PIOC->PIO_CODR = 1<<23;   // Pino 7 (CH) desligado
    DisablePin9();            // Pino 9 (AL) desligado
    DisablePin10();           // Pino 10 (BL) desligado

    PIOC->PIO_SODR = 1<<25;   // Pino 5 (AH) em High
    DutyCyclePin11(pwwm);
    EnablePin11();            // Pino 11 (CL) com PWM ativo    
    break;
    /* ----------------------------------------------------------------------------------------------------------------------- */
    // FASE 4
  case 2:
    PIOC->PIO_CODR = 1<<25;   // Pino 5 (AH) desligado
    PIOC->PIO_CODR = 1<<24;   // Pino 6 (BH) desligado
    PIOC->PIO_CODR = 1<<23;   // Pino 7 (CH) desligado
    DisablePin9();            // Pino 9 (AL) desligado
    DisablePin11();           // Pino 11 (CL) desligado

    PIOC->PIO_SODR = 1<<25;   // Pino 5 (AH) em High
    DutyCyclePin10(pwwm);
    EnablePin10();            // Pino 10 (BL) com PWM ativo
    break;
    /* ----------------------------------------------------------------------------------------------------------------------- */
    // FASE 5
  case 6:
    PIOC->PIO_CODR = 1<<25;   // Pino 5 (AH) desligado
    PIOC->PIO_CODR = 1<<24;   // Pino 6 (BH) desligado
    PIOC->PIO_CODR = 1<<23;   // Pino 7 (CH) desligado
    DisablePin9();            // Pino 9 (AL) desligado
    DisablePin11();           // Pino 11 (CL) desligado

    PIOC->PIO_SODR = 1<<23;   // Pino 7 (CH) ligado
    DutyCyclePin10(pwwm);
    EnablePin10();            // Pino 10 (BL) com PWM ativo
    break;
    /* ----------------------------------------------------------------------------------------------------------------------- */
    // FASE 6
  case 4:
    PIOC->PIO_CODR = 1<<25;   // Pino 5 (AH) desligado
    PIOC->PIO_CODR = 1<<24;   // Pino 6 (BH) desligado
    PIOC->PIO_CODR = 1<<23;   // Pino 7 (CH) desligado
    DisablePin10();           // Pino 10 (BL) desligado
    DisablePin11();           // Pino 11 (CL) desligado

    PIOC->PIO_SODR = 1<<23;   // Pino 7 (CH) ligado
    DutyCyclePin9(pwwm);
    EnablePin9();             // Pino 9 (AL) com PWM ativo
    break;
  }
}
