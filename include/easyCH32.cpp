/******************************************************************************
* Lightweight and simple CH32V003 generic Library for easyIO single channel
*
* This library provides functions to initialize the hardware
* Read voltage from a shunt resistor
* Initialize the mcu's internal Opamp
* Initialize the ADC (connected to the Opamp output on PD4)
* Initialize the gpio's necessary for the project
* Initialize TMR2 to generate PWM signal for use with monitored power MOSFET 
* output (TIM2 clock prescaler divider and set PWM total cycle width)
* Initialize the i2c peripheral in slave mode (configurable address) for full
* control of the slave device and other features
* Configurable under and over current alarm functions
* Fault indicator LED
* Otional control output (relay, triac, etc.) not monitored.


* Power pins and SWIO for CH32V003F4P6 (TSSOP-20)
* VSS = (pin 7)    VCC = (pin 9)    SWIO = PD1 (pin 18)

* Bus I2C - in Default, and Alternative Pinout Modes.
* Default:	SCL = PC2 (pin 12)		SDA = PC1 (pin 11)
* Alt 1:	SCL = PD1 (pin 18)		SDA = PD0 (pin 8)
* Alt 2:	SCL = PC5 (pin 15)		SDA = PC6 (pin 16)
*
* Opamp and ADC Channels (ADC7 and OPAout internaly connected)
* Default:  OPN0 = PA1 (pin 5)  OPP0 = PA2 (pin 6)  OUT AND ADC = PD4 (pin 1)   
* Alt 1:    OPN1 = PD0 (pin 8)  OPP1 = PD7 (pin 4)  OUT AND ADC = PD4 (pin 1)
*  
* TIMER2 PWM output (configurable prescaler divider and total cycle width)
* Default:  PD3 (pin 20)  //T2CH2
* Alt 1:    PC0 (pin 10)  //T2CH3
* Alt 2:    PD7 (pin  4)  //T2CH4

*INPUT definitions
*Default:  INPUT (AC or DC via HCPL3700) = PD2 (pin 19)
*Alt 1:    IMPUT (AC or DC via HCPL3700) = PC7 (pin 17)
*Alt 2:    IMPUT (AC or DC via HCPL3700) = PD1 (pin 18) **WARNING** SWIO pin

*OUTPUT switch definitions
*Default:  OUTPUT (relay, mosfet, ssr, led, etc) = PD6 (pin 3)
*Alt1:     OUTPUT (relay, mosfet, ssr, led, etc) = PD5 (pin 2)
*Alt2:     OUTPUT (relay, mosfet, ssr, led, etc) = PC0 (pin 10)


* See GitHub Repo for more information: 
* https://github.com/rodrigo-s-lange/easyCH32
*
* Released under the MIT Licence
* Copyright Rodrigo Lange (c) 2024 - 2025
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to
* deal in the Software without restriction, including without limitation the 
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
* sell copies of the Software, and to permit persons to whom the Software is 
* furnished to do so, subject to the following conditions:
* The above copyright notice and this permission notice shall be included in 
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR 
* OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE 
* USE OR OTHER DEALINGS IN THE SOFTWARE.
******************************************************************************/

#include <easyCH32.h>
#include "ch32v003fun.h"

#ifndef I2C1_EV_IRQn
#define I2C1_EV_IRQn 10
#endif
#ifndef I2C1_ER_IRQn
#define I2C1_ER_IRQn 11
#endif

///////////////////////////////////////////////////////////////////////////////////////////////
//flag de saída
bool flagOutPwm = false;

//flag de saída
bool flagOutSw = false;

//flag de entrada
bool flagIn = false;

//flag de alarme
bool flagAlm = false;

//variável do tipo volátil que recebe ADC1->RDATAR
volatile uint16_t adc_buffer[0];

///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////

// Construtor para inicializar o shunt e vref
shuntSensor::shuntSensor(float defaultShuntResistance /*= 0.02*/, float defaultVref /*= 5*/) {
    shuntResistance = defaultShuntResistance;
    vref = defaultVref;
}
///////////////////////////////////////////////////////////////////////////////////////////////

 // Função para atualizar o valor do shunt
void shuntSensor::setShuntResistance(float resistance) {
	shuntResistance = resistance;
}
///////////////////////////////////////////////////////////////////////////////////////////////

// Função para atualizar o valor de vref
void shuntSensor::setVref(float referenceVoltage) {
	vref = referenceVoltage;
}
///////////////////////////////////////////////////////////////////////////////////////////////

 // Função para calcular a corrente com base no valor lido pelo ADC
float shuntSensor::calculateCurrent() const{
	// Calcula a tensão no shunt
	float shuntVoltage = (adc_buffer[0] / 1023.0) * vref;

	// Calcula a corrente usando a Lei de Ohm: I = V / R
	float current = shuntVoltage / shuntResistance;

	return current;
}
///////////////////////////////////////////////////////////////////////////////////////////////

// Função de recebe o limite mínimo de corrente em mA
void shuntSensor::underCurrent(uint16_t value /*= 0*/){
	under = value;
}
///////////////////////////////////////////////////////////////////////////////////////////////

// Função de recebe o limite máximo de corrente em mA
void shuntSensor::overCurrent(uint16_t value /*= 5000*/){
	over = value;
}
///////////////////////////////////////////////////////////////////////////////////////////////

// Função de Alarme
bool shuntSensor::Alarm() const{
	if(flagOutPwm){
		if (calculateCurrent() >= over) return true;
		else return false;
	}
};
///////////////////////////////////////////////////////////////////////////////////////////////

// Função de inicialização dos registradores do ADC
void shuntSensor::adc_init( void ) const
{
	// ADCCLK = 24 MHz => RCC_ADCPRE = 0: divide by 2
	RCC->CFGR0 &= ~(0x1F<<11);
	
	// Enable GPIOD and ADC
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_ADC1;
	
	// PD4 is analog input chl 7
	GPIOD->CFGLR &= ~(0xf<<(4*4));	// CNF = 00: Analog, MODE = 00: Input
	
	// Reset the ADC to init all regs
	RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;
	
	// Set up four conversions on chl 7
	ADC1->RSQR1 = 0;
	ADC1->RSQR2 = 0;
	ADC1->RSQR3 = 7<<(5*0);
	
	// set sampling time for chl 7
	// 0:7 => 3/9/15/30/43/57/73/241 cycles
	ADC1->SAMPTR2 = 7<<(3*7);

	// turn on ADC
	ADC1->CTLR2 |= ADC_ADON;
	
	// Reset calibration
	ADC1->CTLR2 |= ADC_RSTCAL;
	while(ADC1->CTLR2 & ADC_RSTCAL);
	
	// Calibrate
	ADC1->CTLR2 |= ADC_CAL;
	while(ADC1->CTLR2 & ADC_CAL);
}
///////////////////////////////////////////////////////////////////////////////////////////////

// Função que habilita o DMA
void shuntSensor::dma_init( void ) const
{
	// Turn on DMA
	RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;
	
	//DMA1_Channel1 is for ADC
	DMA1_Channel1->PADDR = (uint32_t)&ADC1->RDATAR;
	DMA1_Channel1->MADDR = (uint32_t)adc_buffer; //carrega valor adc
	DMA1_Channel1->CNTR  = 1;
	DMA1_Channel1->CFGR  =
		DMA_M2M_Disable |		 
		DMA_Priority_VeryHigh |
		DMA_MemoryDataSize_HalfWord |
		DMA_PeripheralDataSize_HalfWord |
		DMA_MemoryInc_Enable |
		DMA_Mode_Circular |
		DMA_DIR_PeripheralSRC;
	
	// Turn on DMA channel 1
	DMA1_Channel1->CFGR |= DMA_CFGR1_EN;
	
	// enable scanning
	ADC1->CTLR1 |= ADC_SCAN;
	
	// Enable continuous conversion and DMA
	ADC1->CTLR2 |= ADC_CONT | ADC_DMA | ADC_EXTSEL;
	
	// start conversion
	ADC1->CTLR2 |= ADC_SWSTART;
}
///////////////////////////////////////////////////////////////////////////////////////////////

// Função de inicialização do Opamp
void shuntSensor::opamp_init( void ) const
{
	// turn on the op-amp
	EXTEN->EXTEN_CTR |= EXTEN_OPA_EN;

	// select op-amp pos pin: 0 = PA2, 1 = PD7
	//EXTEN->EXTEN_CTR |= EXTEN_OPA_PSEL;

	// select op-amp neg pin: 0 = PA1, 1 = PD0
	//EXTEN->EXTEN_CTR |= EXTEN_OPA_NSEL;
}
///////////////////////////////////////////////////////////////////////////////////////////////

static I2CSlave* i2c_instance = nullptr;

I2CSlave::I2CSlave(uint8_t address, volatile uint8_t* registers, uint8_t size) {
    this->address = address;
    state.first_write = 1;
    state.offset = 0;
    state.position = 0;
    state.registers1 = registers;
    state.size1 = size;
    state.registers2 = nullptr;
    state.size2 = 0;
    state.read_only1 = false;
    state.read_only2 = false;
    state.writing = false;
    state.address2matched = false;

    i2c_instance = this;
}
///////////////////////////////////////////////////////////////////////////////////////////////

void I2CSlave::begin() {
    RCC->APB1PCENR |= RCC_APB1Periph_I2C1;

    RCC->APB1PRSTR |= RCC_APB1Periph_I2C1;
    RCC->APB1PRSTR &= ~RCC_APB1Periph_I2C1;

    I2C1->CTLR1 |= I2C_CTLR1_SWRST;
    I2C1->CTLR1 &= ~I2C_CTLR1_SWRST;

    uint32_t prerate = 2000000;
    I2C1->CTLR2 |= (FUNCONF_SYSTEM_CORE_CLOCK / prerate) & I2C_CTLR2_FREQ;

    I2C1->CTLR2 |= I2C_CTLR2_ITBUFEN | I2C_CTLR2_ITEVTEN | I2C_CTLR2_ITERREN;

    // Configuração do NVIC
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_SetPriority(I2C1_EV_IRQn, 2); // Prioridade 2 (shift ajustado para CH32V003)
    NVIC_EnableIRQ(I2C1_ER_IRQn);
    NVIC_SetPriority(I2C1_ER_IRQn, 2);

    uint32_t clockrate = 1000000;
    I2C1->CKCFGR = ((FUNCONF_SYSTEM_CORE_CLOCK / (3 * clockrate)) & I2C_CKCFGR_CCR) | I2C_CKCFGR_FS;

    I2C1->OADDR1 = address << 1;
    I2C1->OADDR2 = 0;

    I2C1->CTLR1 |= I2C_CTLR1_PE;
    I2C1->CTLR1 |= I2C_CTLR1_ACK;
}
///////////////////////////////////////////////////////////////////////////////////////////////

void I2CSlave::setSecondaryAddress(uint8_t address, volatile uint8_t* registers, uint8_t size) {
    if (address > 0) {
        I2C1->OADDR2 = (address << 1) | 1;
        state.registers2 = registers;
        state.size2 = size;
    } else {
        I2C1->OADDR2 = 0;
        state.registers2 = nullptr;
        state.size2 = 0;
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////

void I2CSlave::setReadOnly(bool read_only, bool secondary) {
    if (secondary) {
        state.read_only2 = read_only;
    } else {
        state.read_only1 = read_only;
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////

void I2CSlave::onWrite(uint8_t reg, uint16_t data, bool secondary) {
    printf("Escrita no registrador 0x%02X (%s): 0x%04X\n", 
           reg, secondary ? "secundário" : "primário", data);
}
///////////////////////////////////////////////////////////////////////////////////////////////

void I2CSlave::onRead(uint8_t reg, uint8_t* data, bool secondary) {
    data[0] = 0xFF;
    data[1] = 0xFF;
    printf("Leitura no registrador 0x%02X (%s): 0x%02X%02X\n", 
           reg, secondary ? "secundário" : "primário", data[0], data[1]);
}
///////////////////////////////////////////////////////////////////////////////////////////////

void I2CSlave::handleEventInterrupt() {
    uint16_t STAR1 = I2C1->STAR1;
    uint16_t STAR2 = I2C1->STAR2;

    if (STAR1 & I2C_STAR1_ADDR) {
        state.first_write = 1;
        state.position = state.offset;
        state.address2matched = !!(STAR2 & I2C_STAR2_DUALF);
    }

    if (STAR1 & I2C_STAR1_RXNE) {
        if (state.first_write) {
            state.offset = I2C1->DATAR;
            state.position = 0;
            state.first_write = 0;
            state.writing = false;
        } else {
            if (state.address2matched) {
                if (state.position < state.size2 && !state.read_only2) {
                    state.registers2[state.position++] = I2C1->DATAR;
                }
            } else {
                if (state.position < state.size1 && !state.read_only1) {
                    state.registers1[state.position++] = I2C1->DATAR;
                }
            }
            state.writing = true;
        }
    }

    if (STAR1 & I2C_STAR1_TXE) {
        state.writing = false;
        volatile uint8_t* regs = state.address2matched ? state.registers2 : state.registers1;
        uint8_t size = state.address2matched ? state.size2 : state.size1;

        if (state.position < 2) {
            uint8_t data[2];
            onRead(state.offset, data, state.address2matched);
            I2C1->DATAR = (state.position < size) ? data[state.position] : 0x00;
            state.position++;
        } else {
            I2C1->DATAR = 0x00;
        }
    }

    if (STAR1 & I2C_STAR1_STOPF) {
        I2C1->CTLR1 &= ~(I2C_CTLR1_STOP);
        if (state.writing && state.position == 2) {
            volatile uint8_t* regs = state.address2matched ? state.registers2 : state.registers1;
            uint16_t data = (regs[0] << 8) | regs[1];
            onWrite(state.offset, data, state.address2matched);
        }
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////

void I2CSlave::handleErrorInterrupt() {
    uint16_t STAR1 = I2C1->STAR1;

    if (STAR1 & I2C_STAR1_BERR) {
        I2C1->STAR1 &= ~(I2C_STAR1_BERR);
    }

    if (STAR1 & I2C_STAR1_ARLO) {
        I2C1->STAR1 &= ~(I2C_STAR1_ARLO);
    }

    if (STAR1 & I2C_STAR1_AF) {
        I2C1->STAR1 &= ~(I2C_STAR1_AF);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////

extern "C" {
    void I2C1_EV_IRQHandler(void) __attribute__((interrupt));
    void I2C1_EV_IRQHandler(void) {
        if (i2c_instance) {
            i2c_instance->handleEventInterrupt();
        }
    }

    void I2C1_ER_IRQHandler(void) __attribute__((interrupt));
    void I2C1_ER_IRQHandler(void) {
        if (i2c_instance) {
            i2c_instance->handleErrorInterrupt();
        }
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////
