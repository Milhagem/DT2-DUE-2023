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
#define botao2_pedal 18   // PEDAL FIM - Acionado pelo pedal de aceleracao. Fim de Curso [Traseiro]. Entrada do botão 2 de aceleracao (Pedal Completamente pressionado).

#define chave_controle 16 // Configura o valor inicial do PWM nos mosfets. Sendo o valor mais baixo para arrancada e o maior para ao longo da corrida
#define acelerador 8      // Para ligar o carro sem apertar o pedal. Alternativa para não precisar apertar o pedal ate o fundo
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

void setup()
{
  // Entradas do Arduino MEGA
  pinMode(hallA, INPUT);
  pinMode(hallB, INPUT);
  pinMode(hallC, INPUT);
  pinMode(sinal_corrente, INPUT);
  
  // Entradas do sinais da frente
  pinMode(botao1_pedal, INPUT);
  pinMode(botao2_pedal,INPUT);
  pinMode(chave_controle,INPUT);
  
  // Saídas do Arduino MEGA
  // Saídas para ativação MOSFETs do inversor trifasico
  // Saídas digitais HIGH SIDE
  pinMode(MOSFET_A_HIGH, OUTPUT);
  pinMode(MOSFET_B_HIGH, OUTPUT);
  pinMode(MOSFET_C_HIGH, OUTPUT);

  // Saidas PWM LOW SIDE
  pinMode(MOSFET_A_LOW, OUTPUT);
  pinMode(MOSFET_B_LOW, OUTPUT);
  pinMode(MOSFET_C_LOW, OUTPUT);

  // Saída Shutdown
  pinMode(shutdownIR, OUTPUT);

  // Coloca as saidas dos MOSFETs e o Shutdown em low

  digitalWrite(shutdownIR, LOW);
  digitalWrite(MOSFET_A_HIGH, LOW);
  digitalWrite(MOSFET_B_HIGH, LOW);
  digitalWrite(MOSFET_C_HIGH, LOW);
  
  analogWrite(MOSFET_A_LOW, 0);
  analogWrite(MOSFET_B_LOW, 0);
  analogWrite(MOSFET_C_HIGH, 0);
}
void loop() {
    PIOA->PIO_CODR = 1 << 9;                          // Pino ShutdownIR em 0
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
    if( ( (digitalRead(botao2_pedal)== LOW) && (digitalRead(botao1_pedal)== HIGH) ) || (digitalRead(acelerador) == LOW) ){
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
    digitalWrite(MOSFET_A_HIGH, LOW);     // Pino MOSFET_A_HIGH (AH) desligado
    digitalWrite(MOSFET_B_HIGH, LOW);     // Pino MOSFET_B_HIGH (BH) desligado
    digitalWrite(MOSFET_C_HIGH, LOW);     // Pino MOSFET_C_HIGH (CH) desligado
      
    analogWrite(MOSFET_B_LOW, 0);       // Pino 10 (BL) desligado
    analogWrite(MOSFET_C_LOW, 0);       // Pino 11 (CL) desligado

    digitalWrite(MOSFET_B_HIGH, HIGH);    // Pino 6 (BH) ligado
    analogWrite(MOSFET_A_LOW, pwwm);     // Pino 9 (AL) com PWM ativo
    break;
    /* ----------------------------------------------------------------------------------------------------------------------- */
    // FASE 2
  case 1:
    digitalWrite(MOSFET_A_HIGH, LOW);     // Pino MOSFET_A_HIGH (AH) desligado
    digitalWrite(MOSFET_B_HIGH, LOW);     // Pino MOSFET_B_HIGH (BH) desligado
    digitalWrite(MOSFET_C_HIGH, LOW);     // Pino MOSFET_C_HIGH (CH) desligado
      
    analogWrite(MOSFET_A_LOW, 0);        // Pino 9 (AL) desligado
    analogWrite(MOSFET_B_LOW, 0);       // Pino 10 (BL) desligado

    digitalWrite(MOSFET_B_HIGH, HIGH);    // Pino 6 (BH) ligado
    analogWrite(MOSFET_C_LOW, pwwm);    // Pino 11 (CL) com PWM ativo
    break;
    /* ----------------------------------------------------------------------------------------------------------------------- */
    // FASE 3
  case 3:
    digitalWrite(MOSFET_A_HIGH, LOW);     // Pino MOSFET_A_HIGH (AH) desligado
    digitalWrite(MOSFET_B_HIGH, LOW);     // Pino MOSFET_B_HIGH (BH) desligado
    digitalWrite(MOSFET_C_HIGH, LOW);     // Pino MOSFET_C_HIGH (CH) desligado
      
    analogWrite(MOSFET_A_LOW, 0);        // Pino 9 (AL) desligado
    analogWrite(MOSFET_B_LOW, 0);       // Pino 10 (BL) desligado

    digitalWrite(MOSFET_A_HIGH, HIGH);    // Pino 5 (AH) em High
    analogWrite(MOSFET_C_LOW, pwwm);    // Pino 11 (CL) com PWM ativo   
    break;
    /* ----------------------------------------------------------------------------------------------------------------------- */
    // FASE 4
  case 2:
    digitalWrite(MOSFET_A_HIGH, LOW);     // Pino MOSFET_A_HIGH (AH) desligado
    digitalWrite(MOSFET_B_HIGH, LOW);     // Pino MOSFET_B_HIGH (BH) desligado
    digitalWrite(MOSFET_C_HIGH, LOW);     // Pino MOSFET_C_HIGH (CH) desligado
      
    analogWrite(MOSFET_A_LOW, 0);        // Pino 9 (AL) desligado
    analogWrite(MOSFET_C_LOW, 0);       // Pino 11 (CL) desligado

    digitalWrite(MOSFET_A_HIGH, HIGH);    // Pino 5 (AH) em High
    analogWrite(MOSFET_B_LOW, pwwm);    // Pino 10 (BL) com PWM ativo
    break;
    /* ----------------------------------------------------------------------------------------------------------------------- */
    // FASE 5
  case 6:
    digitalWrite(MOSFET_A_HIGH, LOW);     // Pino MOSFET_A_HIGH (AH) desligado
    digitalWrite(MOSFET_B_HIGH, LOW);     // Pino MOSFET_B_HIGH (BH) desligado
    digitalWrite(MOSFET_C_HIGH, LOW);     // Pino MOSFET_C_HIGH (CH) desligado
    
    analogWrite(MOSFET_A_LOW, 0);        // Pino 9 (AL) desligado
    analogWrite(MOSFET_C_LOW, 0);       // Pino 11 (CL) desligado

    digitalWrite(MOSFET_C_HIGH, HIGH);    // Pino 7 (CH) ligado
    analogWrite(MOSFET_B_LOW, pwwm);    // Pino 10 (BL) com PWM ativo
    break;
    /* ----------------------------------------------------------------------------------------------------------------------- */
    // FASE 6
  case 4:
    digitalWrite(MOSFET_A_HIGH, LOW);     // Pino MOSFET_A_HIGH (AH) desligado
    digitalWrite(MOSFET_B_HIGH, LOW);     // Pino MOSFET_B_HIGH (BH) desligado
    digitalWrite(MOSFET_C_HIGH, LOW);     // Pino MOSFET_C_HIGH (CH) desligado
    
    analogWrite(MOSFET_B_LOW, 0);       // Pino 10 (BL) desligado
    analogWrite(MOSFET_C_LOW, 0);       // Pino 11 (CL) desligado

    digitalWrite(MOSFET_C_HIGH, HIGH);    // Pino 7 (CH) ligado
    analogWrite(MOSFET_A_LOW, pwwm);     // Pino 9 (AL) com PWM ativo
    break;
  }
}
