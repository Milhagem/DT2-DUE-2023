/**
 * @file pino.cpp
 * @author Joseane Fernandes Miranda
 * @brief Esse arquivo de código tem o objetivo de escrever as funções que vão controlar os pinos
 * do Arduino Due
 * @date 2023-06-26
 * 
 * 
 */

#include "pino.h"
#include "Arduino.h"

/**
 * @brief Construtor do pino
 * 
 * @param pin 
 * @param PWM 
 */

Pin::Pin(unsigned int pin, int ppwm){
    this->_pin = pin;
    this->_PWM = ppwm;
}

/**
 * @brief Configura o pino de acordo com a frequência que o usuário passa
 * 
 * @param freq 
 */

void Pin::configure_pin(int freq){
    pinMode(this->_pin, OUTPUT);
    analogWrite(this->_pin, 0);
    PWMC_ConfigureClocks(freq * PWM_MAX_DUTY_CYCLE , 0, VARIANT_MCK);
}

/**
 * @brief Configura o Duty Cycle do pino
 * 
 * @param Duty_Cycle valor do Duty Cycle
 */

void Pin::duty_cycle_pin(int Duty_Cycle){
    if(Duty_Cycle == 0){
        this->_PWM = 0;
        if(this->_pin == 6){
            PWMC_SetDutyCycle(PWM_INTERFACE, PWM_CH7, 0);
        } else if(this->_pin == 7){
            PWMC_SetDutyCycle(PWM_INTERFACE, PWM_CH6, 0);
        } else if(this->_pin == 8){
            PWMC_SetDutyCycle(PWM_INTERFACE, PWM_CH5, 0);
        } else if(this->_pin == 9){
            PWMC_SetDutyCycle(PWM_INTERFACE, PWM_CH4, 0);
        }   
        } else {
            this->_PWM = Duty_Cycle;
    }
}

/**
 * @brief Desativa o pino
 * 
 */

void Pin::disable_pin(){
    this->_PWM = 0;
    if(this->_pin == 6){
        PWMC_SetDutyCycle(PWM_INTERFACE, PWM_CH7, 0);
    } else if(this->_pin == 7){
        PWMC_SetDutyCycle(PWM_INTERFACE, PWM_CH6, 0);
    } else if(this->_pin == 8){
        PWMC_SetDutyCycle(PWM_INTERFACE, PWM_CH5, 0);
    } else if(this->_pin == 9){
        PWMC_SetDutyCycle(PWM_INTERFACE, PWM_CH4, 0);   
    }
}

/**
 * @brief Ativa o pino
 * 
 */

void Pin::enable_pin(){
    int x = map(this->_PWM, 0, 100, 0, 255);
    if(this->_pin == 6){
        PWMC_SetDutyCycle(PWM_INTERFACE, PWM_CH7, x); 
    }
    if(this->_pin == 7){
        PWMC_SetDutyCycle(PWM_INTERFACE, PWM_CH6, x);
    }
    if(this->_pin == 8){
        PWMC_SetDutyCycle(PWM_INTERFACE, PWM_CH5, x);
    } else if(this->_pin == 9){
        PWMC_SetDutyCycle(PWM_INTERFACE, PWM_CH4, x);
    }
}