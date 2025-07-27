/***********************************************************************************
 *    (c) Tuxcnc 2025
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 ***********************************************************************************/

/*
If you use Arduino IDE, disable USB in "Tools/USB support" menu.

Uncomment line below if you want use pins PA13 and PA14 as inputs.
This lines works as SWDIO and SWCLK, and if programed as inputs, ST-Link can't reset chip.
In this case you must manually press reset button and hold while start programming, then release.
This affect only programmed chips, new or cleaned are programming normally.
*/

//**********************
// #define USE_ALL_PINS
//**********************

/*
STM32F103 Blue Pill pinout:

PA0	->	out.3
PA1	->	out.4
PA2	->	out.5
PA3	->	out.6
PA4	->	W5500 CS 
PA5	->	W5500 SCK
PA6	->	W5500 MISO
PA7	->	W5500 MOSI
PA8	->	PWM
PA9	->	in.4
PA10	->	in.5
PA11	->	in.6
PA12	->	in.7
PA13	->	in.13	(SWDIO, works as input if defined USE_ALL_PINS)
PA14	->	in.14	(SWCLK, works as input if defined USE_ALL_PINS)
PA15	->	in.8

PB0	->	out.7
PB1	->	out.8
PB2	->	<unavailable>
PB3	->	in.9
PB4	->	in.10
PB5	->	in.11
PB6	->	encoder A
PB7	->	encoder B
PB8	->	encoder Z
PB9	->	in.12
PB10	->	out.9
PB11	->	out.10
PB12	->	in.0
PB13	->	in.1
PB14	->	in.2
PB15	->	in.3

PC0	->	<unavailable>
PC1	->	<unavailable>
PC2	->	<unavailable>
PC3	->	<unavailable>
PC4	->	<unavailable>
PC5	->	<unavailable>
PC6	->	<unavailable>
PC7	->	<unavailable>
PC8	->	<unavailable>
PC9	->	<unavailable>
PC10	->	<unavailable>
PC11	->	<unavailable>
PC12	->	<unavailable>
PC13	->	out.0
PC14	->	out.1
PC15	->	out.2
*/

#include <Ethernet.h>
#include <EthernetUdp.h>

struct cmdPacket {
    uint16_t control;
    uint16_t io;
    uint16_t pwm;
} cmd = { 0, 0, 0 };

struct fbPacket {
    uint16_t control;
    uint16_t io;
    double raw_encoder;
    double encoder_latched;
} fb = { 0, 0, 0, 0 };

uint16_t old_pwm = 0;
volatile double enc4_ovf=0;

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xE1 };
IPAddress ip(10, 10, 10, 88);

unsigned int localPort = 58418;      // local port to listen on

// buffers for receiving and sending data
char packetBuffer[255];  // buffer to hold incoming packet,

#define IO_00 0b0000000000000001
#define IO_01 0b0000000000000010
#define IO_02 0b0000000000000100
#define IO_03 0b0000000000001000
#define IO_04 0b0000000000010000
#define IO_05 0b0000000000100000
#define IO_06 0b0000000001000000
#define IO_07 0b0000000010000000
#define IO_08 0b0000000100000000
#define IO_09 0b0000001000000000
#define IO_10 0b0000010000000000
#define IO_11 0b0000100000000000
#define IO_12 0b0001000000000000
#define IO_13 0b0010000000000000
#define IO_14 0b0100000000000000
#define IO_15 0b1000000000000000

#define CTRL_READY    0b0000000000000001
#define CTRL_ENABLE   0b0000000000000010

#define OUT_00_PIN PC13
#define OUT_00_H GPIOC->ODR |= 1<<13
#define OUT_00_L GPIOC->ODR &= ~(1<<13)

#define OUT_01_PIN PC14
#define OUT_01_H GPIOC->ODR |= 1<<14
#define OUT_01_L GPIOC->ODR &= ~(1<<14)

#define OUT_02_PIN PC15
#define OUT_02_H GPIOC->ODR |= 1<<15
#define OUT_02_L GPIOC->ODR &= ~(1<<15)

#define OUT_03_PIN PA0
#define OUT_03_H GPIOA->ODR |= 1<<0
#define OUT_03_L GPIOA->ODR &= ~(1<<0)

#define OUT_04_PIN PA1
#define OUT_04_H GPIOA->ODR |= 1<<1
#define OUT_04_L GPIOA->ODR &= ~(1<<1)

#define OUT_05_PIN PA2
#define OUT_05_H GPIOA->ODR |= 1<<2
#define OUT_05_L GPIOA->ODR &= ~(1<<2)

#define OUT_06_PIN PA3
#define OUT_06_H GPIOA->ODR |= 1<<3
#define OUT_06_L GPIOA->ODR &= ~(1<<3)

#define OUT_07_PIN PB0
#define OUT_07_H GPIOB->ODR |= 1<<0
#define OUT_07_L GPIOB->ODR &= ~(1<<0)

#define OUT_08_PIN PB1
#define OUT_08_H GPIOB->ODR |= 1<<1
#define OUT_08_L GPIOB->ODR &= ~(1<<1)

#define OUT_09_PIN PB10
#define OUT_09_H GPIOB->ODR |= 1<<10
#define OUT_09_L GPIOB->ODR &= ~(1<<10)

#define OUT_10_PIN PB11
#define OUT_10_H GPIOB->ODR |= 1<<11
#define OUT_10_L GPIOB->ODR &= ~(1<<11)

#define IN_00_PIN PB12
#define IN_00 GPIOB->IDR & (1<<12)

#define IN_01_PIN PB13
#define IN_01 GPIOB->IDR & (1<<13)

#define IN_02_PIN PB14
#define IN_02 GPIOB->IDR & (1<<14)

#define IN_03_PIN PB15
#define IN_03 GPIOB->IDR & (1<<15)

#define IN_04_PIN PA9
#define IN_04 GPIOA->IDR & (1<<9)

#define IN_05_PIN PA10
#define IN_05 GPIOA->IDR & (1<<10)

#define IN_06_PIN PA11
#define IN_06 GPIOA->IDR & (1<<11)

#define IN_07_PIN PA12
#define IN_07 GPIOA->IDR & (1<<12)

#define IN_08_PIN PA15
#define IN_08 GPIOA->IDR & (1<<15)

#define IN_09_PIN PB3
#define IN_09 GPIOB->IDR & (1<<3)

#define IN_10_PIN PB4
#define IN_10 GPIOB->IDR & (1<<4)

#define IN_11_PIN PB5
#define IN_11 GPIOB->IDR & (1<<5)

#define IN_12_PIN PB9
#define IN_12 GPIOB->IDR & (1<<9)

#define IN_13_PIN PA13
#define IN_13 GPIOA->IDR & (1<<13)

#define IN_14_PIN PA14
#define IN_14 GPIOA->IDR & (1<<14)

EthernetUDP Udp;

void OnTimer4Interrupt() {
(TIM4->CR1 & 0x10) ? enc4_ovf-=0x10000 : enc4_ovf+=0x10000;
}

void index_ISR1() {
fb.encoder_latched  = enc4_ovf+TIM4->CNT;
}
void setup() {

// PWM at PA8 pin
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  GPIOA->CRH &= ~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8);  
  GPIOA->CRH |= GPIO_CRH_CNF8_1;
  GPIOA->CRH |= GPIO_CRH_MODE8_1;
  TIM1->BDTR |= TIM_BDTR_MOE;
  TIM1->CR1 &= ~TIM_CR1_CEN;
  TIM1->PSC = 0;
  TIM1->ARR = 65535;
  TIM1->CCMR1 &= ~TIM_CCMR1_CC1S;
  TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
  TIM1->CCER &= ~TIM_CCER_CC1P;
  TIM1->CCR1 = 0;
  TIM1->CCER |= TIM_CCER_CC1E;
  TIM1->CR1 |= TIM_CR1_CEN;
// End PWM

// Spindle index input at pin PB8
  pinMode(PB8, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PB8), index_ISR1, FALLING);
//
  
// configure TIM4 as Encoder input
// Inputs at PB6 and PB7 pins
  pinMode(PB6, INPUT_PULLUP);
  pinMode(PB7, INPUT_PULLUP);
  TIM_TypeDef *Instance4 = TIM4;
  HardwareTimer *MyTim4 = new HardwareTimer(Instance4);
  MyTim4->attachInterrupt(OnTimer4Interrupt);  
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; //0x00000001;  // Enable clock for TIM4
  TIM4->CR1   = 0x0000;     // CEN(Counter ENable)='0'     < TIM control register 1
  TIM4->SMCR  = 0x0003;     // SMS='011' (Encoder mode 3)  < TIM slave mode control register
  TIM4->CCMR1 = 0xF1F1;     // CC1S='01' CC2S='01'         < TIM capture/compare mode register 1
  TIM4->CCMR2 = 0x0000;     //                             < TIM capture/compare mode register 2
  TIM4->CCER  = 0x0011;     // CC1P CC2P                   < TIM capture/compare enable register
  TIM4->PSC   = 0x0000;     // Prescaler = 0               < TIM prescaler
  TIM4->ARR   = 0xffff;     // reload at 0xffff            < TIM auto-reload register  
  TIM4->CNT   = 0x0000;     //reset the counter before we use it 
  TIM4->DIER|=TIM_DIER_UIE; // Turn on interrupt
  TIM4->CR1   = 0x0001;
// End TIM4

  pinMode(OUT_00_PIN, OUTPUT);
  digitalWrite(OUT_00_PIN, 0);
  pinMode(OUT_01_PIN, OUTPUT);
  digitalWrite(OUT_01_PIN, 0);
  pinMode(OUT_02_PIN, OUTPUT);
  digitalWrite(OUT_02_PIN, 0);
  pinMode(OUT_03_PIN, OUTPUT);
  digitalWrite(OUT_03_PIN, 0);
  pinMode(OUT_04_PIN, OUTPUT);
  digitalWrite(OUT_04_PIN, 0);
  pinMode(OUT_05_PIN, OUTPUT);
  digitalWrite(OUT_05_PIN, 0);
  pinMode(OUT_06_PIN, OUTPUT);
  digitalWrite(OUT_06_PIN, 0);
  pinMode(OUT_07_PIN, OUTPUT);
  digitalWrite(OUT_07_PIN, 0);
  pinMode(OUT_08_PIN, OUTPUT);
  digitalWrite(OUT_08_PIN, 0);
  pinMode(OUT_09_PIN, OUTPUT);
  digitalWrite(OUT_09_PIN, 0);
  pinMode(OUT_10_PIN, OUTPUT);
  digitalWrite(OUT_10_PIN, 0);

  pinMode(IN_00_PIN, INPUT_PULLUP);
  pinMode(IN_01_PIN, INPUT_PULLUP);
  pinMode(IN_02_PIN, INPUT_PULLUP);
  pinMode(IN_03_PIN, INPUT_PULLUP);
  pinMode(IN_04_PIN, INPUT_PULLUP);
  pinMode(IN_05_PIN, INPUT_PULLUP);
  pinMode(IN_06_PIN, INPUT_PULLUP);
  pinMode(IN_07_PIN, INPUT_PULLUP);
  pinMode(IN_08_PIN, INPUT_PULLUP);
  pinMode(IN_09_PIN, INPUT_PULLUP);
  pinMode(IN_10_PIN, INPUT_PULLUP);
  pinMode(IN_11_PIN, INPUT_PULLUP);
  pinMode(IN_12_PIN, INPUT_PULLUP);
  #ifdef USE_ALL_PINS
  pinMode(IN_13_PIN, INPUT_PULLUP);
  pinMode(IN_14_PIN, INPUT_PULLUP);
  #endif

  Ethernet.begin(mac, ip);
  Udp.begin(localPort);
}

void loop() {  

  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    // read the packet into packetBufffer
    
    Udp.read(packetBuffer, sizeof(cmd) + 1);

    uint8_t chk = 71;
    for (int i = 0; i < sizeof(cmd); i++){
    chk ^= packetBuffer[i];
    }
    if (packetBuffer[sizeof(cmd)] == chk) {
    memcpy(&cmd, &packetBuffer, sizeof(cmd));

    fb.io = 0;
    fb.control = 0;
    if (!(cmd.control & CTRL_ENABLE)) {
    cmd.pwm = 0;
    OUT_00_L;
    OUT_01_L;
    OUT_02_L;
    OUT_03_L;
    OUT_04_L;
    OUT_05_L;
    OUT_06_L;
    OUT_07_L;
    OUT_08_L;
    OUT_09_L;
    OUT_10_L;
    }
    if (old_pwm != cmd.pwm) {
    TIM1->CCR1 = cmd.pwm;
    old_pwm = cmd.pwm;
    }
    if (cmd.control & CTRL_ENABLE) {
    fb.control |= CTRL_READY; 
    
    (cmd.io & IO_00) ? OUT_00_H : OUT_00_L;
    (cmd.io & IO_01) ? OUT_01_H : OUT_01_L;
    (cmd.io & IO_02) ? OUT_02_H : OUT_02_L;
    (cmd.io & IO_03) ? OUT_03_H : OUT_03_L;
    (cmd.io & IO_04) ? OUT_04_H : OUT_04_L;
    (cmd.io & IO_05) ? OUT_05_H : OUT_05_L;
    (cmd.io & IO_06) ? OUT_06_H : OUT_06_L;
    (cmd.io & IO_07) ? OUT_07_H : OUT_07_L;
    (cmd.io & IO_08) ? OUT_08_H : OUT_08_L;
    (cmd.io & IO_09) ? OUT_09_H : OUT_09_L;
    (cmd.io & IO_10) ? OUT_10_H : OUT_10_L;

    (IN_00) ? (fb.io = IO_00) : (fb.io = 0);
    if (IN_01) fb.io |= IO_01;
    if (IN_02) fb.io |= IO_02;
    if (IN_03) fb.io |= IO_03;
    if (IN_04) fb.io |= IO_04;
    if (IN_05) fb.io |= IO_05;
    if (IN_06) fb.io |= IO_06;
    if (IN_07) fb.io |= IO_07;
    if (IN_08) fb.io |= IO_08;
    if (IN_09) fb.io |= IO_09;
    if (IN_10) fb.io |= IO_10;
    if (IN_11) fb.io |= IO_11;
    if (IN_12) fb.io |= IO_12;
    #ifdef USE_ALL_PINS
    if (IN_13) fb.io |= IO_13;
    if (IN_14) fb.io |= IO_14;
    #endif

    fb.raw_encoder = enc4_ovf+TIM4->CNT;
    }

    // send a reply to the IP address and port that sent us the packet we received
    
    memcpy(&packetBuffer, &fb, sizeof(fb));
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(packetBuffer, sizeof(fb));
    Udp.endPacket();
    }
  }
}
