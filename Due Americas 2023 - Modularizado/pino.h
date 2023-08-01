/**
 * @file pino.h
 * @author Joseane Fernandes Miranda
 * @brief Esse arquivo tem o objetivo de criar uma biblioteca capaz de configurar os pinos de um Arduino Due.
 * Cujo papel é controlar um motor BLDC.
 * @date 2023-06-27
 * 
 * 
 */

#ifndef PINO_H
#define PINO_H

#include "Arduino.h"

class Pin{
    private:
        unsigned int _pin;
        int _PWM;

    public:
        Pin(unsigned int pin, int ppwm);
        void configure_pin(int freq);
        void duty_cycle_pin(int Duty_Cycle);
        void disable_pin();
        void enable_pin();
};

#endif