#pragma once

#ifndef EASYCH32_H
#define EASYCH32_H

#include "ch32fun.h"
#include <stdio.h>
#include "ch32v003fun.h"

// Definições de interrupções para CH32V003 (caso não estejam no ch32v003fun.h)
#ifndef I2C1_EV_IRQn
#define I2C1_EV_IRQn 10
#endif
#ifndef I2C1_ER_IRQn
#define I2C1_ER_IRQn 11
#endif

// Verifica se NVIC está disponível, senão define manualmente
#ifndef NVIC_EnableIRQ
#define NVIC_EnableIRQ(irq) NVIC->ISER = (1 << ((irq) & 0x1F))
#endif
#ifndef NVIC_SetPriority
#define NVIC_SetPriority(irq, priority) do { \
    uint8_t shift = ((irq) & 0x03) << 3; \
    NVIC->IP[(irq) >> 2] = (NVIC->IP[(irq) >> 2] & ~(0xFF << shift)) | ((priority) << shift); \
} while (0)
#endif

//flag de saída
extern bool _flagOutPwm = false;

//flag de saída
extern bool _flagOutSw = false;

//flag de entrada
extern bool _flagIn = false;

//flag de alarme
extern bool _flagAlm = false;

//corrente atual
extern uint16_t _current = 0;

//valor atual do pwm
extern uint16_t _pwm = 0;

//variável do tipo volátil que recebe ADC1->RDATAR
extern volatile uint16_t adc_buffer[0];


///////////////////////////////////////////////////////////////////////////////////////////////
class shuntSensor
{
protected:
    void adc_init( void ) const;
    void dma_init( void ) const;
    void opamp_init( void ) const; 

private:
    float shuntResistance;  // Valor do shunt (em ohms)
    float vref;             // Valor de referência (em volts)
    uint16_t under;         // Limite mínimo de corrente (em mA)
    uint16_t over;          // Limite máximo de corrente (em mA)

public:
    // Construtor para inicializar o shunt e vref
    shuntSensor(float defaultShuntResistance = 0.02, float defaultVref = 5);

    // Função para atualizar o valor do shunt
    void setShuntResistance(float resistance);

    // Função para atualizar o valor de vref
    void setVref(float referenceVoltage);

    // Função para calcular a corrente com base no valor lido pelo ADC
    float calculateCurrent() const;

    // Função de recebe o limite mínimo de corrente em mA
    void underCurrent(uint16_t value = 0);

    // Função de recebe o limite máximo de corrente em mA
    void overCurrent(uint16_t value = 5000);

    // Função de Alarme
    bool Alarm() const;

};

///////////////////////////////////////////////////////////////////////////////////////////////
class Pwm
{
protected:

private:

public:
    
};


class Imput
{
protected:

private:

public:
    
};

///////////////////////////////////////////////////////////////////////////////////////////////
class Output
{
protected:

private:

public:
    
};

///////////////////////////////////////////////////////////////////////////////////////////////

// Definições alternativas para NVIC, caso não estejam disponíveis
#ifndef NVIC_EnableIRQ
#define NVIC_EnableIRQ(irqn) NVIC->ISER = (1 << ((uint32_t)(irqn) & 0x1F))
#endif
#ifndef NVIC_SetPriority
#define NVIC_SetPriority(irqn, priority) do { \
    uint32_t shift = ((irqn & 0x0F) << 3); \
    NVIC->IP = (NVIC->IP & ~(0xFF << shift)) | ((priority & 0xFF) << shift); \
} while(0)
#endif
#ifndef I2C1_EV_IRQn
#define I2C1_EV_IRQn 10
#endif
#ifndef I2C1_ER_IRQn
#define I2C1_ER_IRQn 11
#endif

class I2CSlave {
public:
    I2CSlave(uint8_t address, volatile uint8_t* registers, uint8_t size);
    void begin();
    void setSecondaryAddress(uint8_t address, volatile uint8_t* registers, uint8_t size);
    void setReadOnly(bool read_only, bool secondary = false);
    virtual void onWrite(uint8_t reg, uint16_t data, bool secondary = false);
    virtual void onRead(uint8_t reg, uint8_t* data, bool secondary = false);

private:
    struct I2CState {
        uint8_t first_write;
        uint8_t offset;
        uint8_t position;
        volatile uint8_t* registers1;
        uint8_t size1;
        volatile uint8_t* registers2;
        uint8_t size2;
        bool read_only1;
        bool read_only2;
        bool writing;
        bool address2matched;
    } state;

    uint8_t address;

    void handleEventInterrupt();
    void handleErrorInterrupt();

    friend void I2C1_EV_IRQHandler(void);
    friend void I2C1_ER_IRQHandler(void);
};

#endif