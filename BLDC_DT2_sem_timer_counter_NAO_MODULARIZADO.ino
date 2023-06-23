/*
 *  Autor: Diego Amaral Castro
 *  Data: Fevereiro/Março de 2023
    Alterações feitas por Joseane Fernandes Miranda em Junho/2023
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
#define MOSFET_A_HIGH 9
#define MOSFET_B_HIGH 10
#define MOSFET_C_HIGH 11

#define MOSFET_A_LOW 6
#define MOSFET_B_LOW 7
#define MOSFET_C_LOW 8

// Duty cycle do PWM maximo no gate
#define PWM_MAX 99

// Portas definidas DUE
#define hallA 4           // Entrada do sinal do sensor de efeito hall A do motor
#define hallB 3           // Entrada do sinal do sensor de efeito hall B do motor
#define hallC 2           // Entrada do sinal do sensor de efeito hall C do motor

#define botao1_pedal 17   // PEDAL INICIO -CHAVE DA FRENTE COM CABO AZUL ACIONAMENTO MOTOR - Acionado pelo pedal de aceleracao. Fim de Curso [Dianteiro]. Entrada do botão 1 de aceleracao (Pedal meio pressionado) 
#define botao2_pedal 18   // PEDAL FIM -  Acionado pelo pedal de aceleracao. Fim de Curso [Traseiro]. Entrada do botão 2 de aceleracao (Pedal Completamente pressionado)

#define chave_controle 16 // Configura o valor inicial do PWM nos mosfets. Sendo o valor mais baixo para arrancada e o maior para ao longo da corrida
#define acelerador 5      // Para ligar o carro sem apertar o pedal. Alternativa para não precisar apertar o pedal ate o fundo
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
int pwwm = 0;             // Armazena o PWM que ira para os MOSFETs (0 a 100%)
int PWM6 = 0;             // Variável que atualiza constantemente o Duty Cycle do pino 6 (0 a 100%)
int PWM7 = 0;            // Variável que atualiza constantemente o Duty Cycle do pino 7 (0 a 100%)
int PWM8 = 0;            // Variável que atualiza constantemente o Duty Cycle do pino 8 (0 a 100%)
int freq = 1000;           // Frequência do PWM em Hz, esse valor não será mudado ao longo da execução do código
int TC_RC = 42000000/freq;// Variável utilizada para definir a frequência dos pinos que usam Timer Counter

//Funções de configuração dos pinos 6,7 e 8
void ConfigurePin6(int freq){
  pinMode(6, OUTPUT);
  analogWrite(6, 0);    
  PWMC_ConfigureClocks(freq * PWM_MAX_DUTY_CYCLE , 0, VARIANT_MCK);
}

void ConfigurePin7(int freq){
  pinMode(7, OUTPUT);
  analogWrite(7, 0);    
  PWMC_ConfigureClocks(freq * PWM_MAX_DUTY_CYCLE , 0, VARIANT_MCK);
}

void ConfigurePin8(int freq){
  pinMode(8, OUTPUT);
  analogWrite(8, 0);    
  PWMC_ConfigureClocks(freq * PWM_MAX_DUTY_CYCLE , 0, VARIANT_MCK);
}

//Funções para ajustar o DutyCycle dos pinos 6, 7 e 8

void DutyCyclePin6(int Duty_Cycle){
if(Duty_Cycle == 0){
    DisablePin6();      
  } else {
    PWM6 = Duty_Cycle;
  }   
}

void DutyCyclePin7(int Duty_Cycle){
if(Duty_Cycle == 0){
    DisablePin7();      
  } else {
    PWM7 = Duty_Cycle;
  }   
}

void DutyCyclePin8(int Duty_Cycle){
if(Duty_Cycle == 0){
    DisablePin8();      
  } else {
    PWM8 = Duty_Cycle;
  }   
}

//Funções de desabilitar os pinos 6, 7 e 8
void DisablePin8(){
  PWM8 = 0;
  PWMC_SetDutyCycle(PWM_INTERFACE, PWM_CH5, 0);
}

void DisablePin7(){
  PWM7 = 0;
  PWMC_SetDutyCycle(PWM_INTERFACE, PWM_CH6, 0);
}

void DisablePin6(){
  PWM6 = 0;
  PWMC_SetDutyCycle(PWM_INTERFACE, PWM_CH7, 0);
}

//Funções de habilitar os pinos 6, 7 e 8
void EnablePin8(){
  int x_8 = map(PWM8, 0, 100, 0, 255);
  PWMC_SetDutyCycle(PWM_INTERFACE, PWM_CH5, x_8);    
}

void EnablePin7(){
  int x_7 = map(PWM7, 0, 100, 0, 255);
  PWMC_SetDutyCycle(PWM_INTERFACE, PWM_CH6, x_7);    
}

void EnablePin6(){
  int x_6 = map(PWM6, 0, 100, 0, 255);
  PWMC_SetDutyCycle(PWM_INTERFACE, PWM_CH7, x_6);    
}

void setup()
{
  // Entradas do Arduino Due
  pinMode(hallA, INPUT);
  pinMode(hallB, INPUT);
  pinMode(hallC, INPUT);
  pinMode(sinal_corrente, INPUT);
  pinMode(botao1_pedal, INPUT);
  pinMode(botao2_pedal,INPUT);
  pinMode(chave_controle,INPUT);
  pinMode(acelerador, INPUT);

  // Saídas do Arduino Due
  // Saídas para ativação MOSFETs do inversor trifasico
  // Saídas digitais HIGH SIDE
  pinMode(MOSFET_A_HIGH, OUTPUT);
  pinMode(MOSFET_B_HIGH, OUTPUT);
  pinMode(MOSFET_C_HIGH, OUTPUT);

  // Saidas PWM LOW SIDE
  ConfigurePin8(freq);                                         // MOSFET_C_LOW
  ConfigurePin7(freq);                                         // MOSFET_B_LOW
  ConfigurePin6(freq);                                         // MOSFET_A_LOW

  // Saída Shutdown
  pinMode(shutdownIR, OUTPUT);

  // Coloca as saidas dos MOSFETs e o Shutdown em low

  digitalWrite(shutdownIR, LOW);
  digitalWrite(MOSFET_A_HIGH, LOW);
  digitalWrite(MOSFET_B_HIGH, LOW);
  digitalWrite(MOSFET_C_HIGH, LOW);
  DisablePin8();
  DisablePin7();
  DisablePin6();

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
       inicio_rampa = 25;
    }
    else{
       inicio_rampa = 50;
    }
  
    // Liga o carro se o pedal estiver completamente pressionado. Ou se o acelerador estiver ligado
    if( digitalRead(acelerador) == LOW || digitalRead(botao1_pedal)){
      if (incremento_rampa < PWM_MAX - inicio_rampa){            // Se o duty cycle do PWM sobre o gate dos MOSFETs ainda eh menor do que PWM_MAX
        if ((tempo_Atual - tempo_Anterior) >= 70){  // Incrementa a cada 70 milisegundos
          incremento_rampa+=1;
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
    PIOC->PIO_CODR = 1<<21;   // Pino 9 (AH) desligado
    PIOC->PIO_CODR = 1<<29;   // Pino 10 (BH) desligado
    PIOD->PIO_CODR = 1<<7;   // Pino 11 (CH) desligado
    DisablePin7();           // Pino 7 (BL) desligado
    DisablePin8();           // Pino 8 (CL) desligado

    PIOC->PIO_SODR = 1<<29;   // Pino 10 (BH) ligado
    DutyCyclePin6(pwwm);
    EnablePin6();             // Pino 6 (AL) com PWM ativo
    break;
    /* ----------------------------------------------------------------------------------------------------------------------- */
    // FASE 2
  case 1:
    PIOC->PIO_CODR = 1<<21;   // Pino 9 (AH) desligado
    PIOC->PIO_CODR = 1<<29;   // Pino 10 (BH) desligado
    PIOD->PIO_CODR = 1<<7;   // Pino 11 (CH) desligado
    DisablePin6();            // Pino 6 (AL) desligado
    DisablePin7();           // Pino 7 (BL) desligado

    PIOC->PIO_SODR = 1<<29;   // Pino 10 (BH) ligado
    DutyCyclePin8(pwwm);
    EnablePin8();            // Pino 8 (CL) com PWM ativo
    break;
    /* ----------------------------------------------------------------------------------------------------------------------- */
    // FASE 3
  case 3:
    PIOC->PIO_CODR = 1<<21;   // Pino 9 (AH) desligado
    PIOC->PIO_CODR = 1<<29;   // Pino 10 (BH) desligado
    PIOD->PIO_CODR = 1<<7;   // Pino 11 (CH) desligado
    DisablePin6();            // Pino 6 (AL) desligado
    DisablePin7();           // Pino 7 (BL) desligado

    PIOC->PIO_SODR = 1<<21;   // Pino 9 (AH) em High
    DutyCyclePin8(pwwm);
    EnablePin8();            // Pino 8 (CL) com PWM ativo    
    break;
    /* ----------------------------------------------------------------------------------------------------------------------- */
    // FASE 4
  case 2:
    PIOC->PIO_CODR = 1<<21;   // Pino 9 (AH) desligado
    PIOC->PIO_CODR = 1<<29;   // Pino 10 (BH) desligado
    PIOD->PIO_CODR = 1<<7;   // Pino 11 (CH) desligado
    DisablePin6();            // Pino 6 (AL) desligado
    DisablePin8();           // Pino 8 (CL) desligado

    PIOC->PIO_SODR = 1<<21;   // Pino 9 (AH) em High
    DutyCyclePin7(pwwm);
    EnablePin7();            // Pino 7 (BL) com PWM ativo
    break;
    /* ----------------------------------------------------------------------------------------------------------------------- */
    // FASE 5
  case 6:
    PIOC->PIO_CODR = 1<<21;   // Pino 9 (AH) desligado
    PIOC->PIO_CODR = 1<<29;   // Pino 10 (BH) desligado
    PIOD->PIO_CODR = 1<<7;   // Pino 11 (CH) desligado
    DisablePin6();            // Pino 6 (AL) desligado
    DisablePin8();           // Pino 8 (CL) desligado

    PIOD->PIO_SODR = 1<<7;   // Pino 11 (CH) ligado
    DutyCyclePin7(pwwm);
    EnablePin7();            // Pino 7 (BL) com PWM ativo
    break;
    /* ----------------------------------------------------------------------------------------------------------------------- */
    // FASE 6
  case 4:
    PIOC->PIO_CODR = 1<<21;   // Pino 9 (AH) desligado
    PIOC->PIO_CODR = 1<<29;   // Pino 10 (BH) desligado
    PIOD->PIO_CODR = 1<<7;   // Pino 11 (CH) desligado
    DisablePin7();           // Pino 7 (BL) desligado
    DisablePin8();           // Pino 8 (CL) desligado

    PIOD->PIO_SODR = 1<<7;   // Pino 11 (CH) ligado
    DutyCyclePin6(pwwm);
    EnablePin6();             // Pino 6 (AL) com PWM ativo
    break;
  }
}