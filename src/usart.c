/* This file leverages the Lightweight USB device Stack for STM32 microcontrollers
 *
 * Copyright ©2024 Mark Salyzyn <mark.salyzyn[at]gmail[dot]com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "usart.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "config.h"
#include "stm32_compat.h"
#include "flash.h"
#include "crypto.h"

/* address would result in an access or bootloader violation */
extern bool flash_address(uintptr_t address, size_t blksize);
extern bool invalid_address(uintptr_t address, size_t blksize);
extern uint8_t __isr_vector;
extern uint8_t __etext;

#if (DFU_BOOTKEY_ADDR == _AUTO) || (DFU_BOOTKEY_ADDR == _DISABLE)
extern uint8_t __stack;
#define _KEY_ADDR   &__stack
#else
#define _KEY_ADDR   DFU_BOOTKEY_ADDR
#endif

#define STATUS_VAL(x) x

#if defined(DFU_USART) && defined(DFU_UART)
#undef DFU_UART
#endif

#if !defined(DFU_USART) && defined(DFU_UART)
#define DFU_USART DFU_UART
#endif

/* add ~1.6K (1576 STM32F4, 1668 STM32G4) if selected */
#ifdef DFU_USART

#ifndef __XSTRING
#define __STRING(x) #x
#define __XSTRING(x) __STRING(x)
#endif

/* determine which USART DFU_USART is referencing */
#if (~(~DFU_USART + 0) == 0) && (~(~DFU_USART + 1) == 1)
#define USARTx_BASE USART2_BASE
#elif (DFU_USART >= PERIPH_BASE)
#define USARTx_BASE (DFU_USART)
#elif (0 < DFU_USART) && (DFU_USART < 9)
#ifdef DFU_UART
#define __CONCAT_A(x) UART ## x ## _BASE
#else
#define __CONCAT_A(x) USART ## x ## _BASE
#endif
#define __CONCAT_B(x) __CONCAT_A(x)
#define USARTx_BASE __CONCAT_B(DFU_USART)
#else
#error DFU_USART/DFU_UART does not reference an USART/UART
#endif

/* #pragma message "USARTx_BASE = " __XSTRING(USARTx_BASE) */

/* default system code clock rate. See mcu files */
#ifndef SystemCoreClock
#if defined(STM32F0) || defined(STM32F1) || defined(STM32F3) || defined(STM32L4)
#define SystemCoreClock 48000000UL
#elif defined(STM32F4)
#define SystemCoreClock 72000000UL
#elif defined(STM32G4)
#define SystemCoreClock 36000000UL
#elif defined(STM32L0) || defined(STM32L1)
#define SystemCoreClock 24000000UL
#else
#error "Can not determine SystemCoreClock"
#endif
#endif

/* default baud rate */
#ifndef USART_BAUD_RATE
#define USART_BAUD_RATE 921600
#endif
#if (USART_BAUD_RATE == _AUTO) || ((USART_BAUD_RATE + 0) == 0)
#if ((((SystemCoreClock) + (921600 / 2)) / 921600) < 16)
#error "USART_BAUD_RATE:" USART_BAUD_RATE "Impossible"
#endif
#else
#if ((((SystemCoreClock) + ((USART_BAUD_RATE) / 2)) / (USART_BAUD_RATE)) < 16)
#error "USART_BAUD_RATE:" USART_BAUD_RATE "Impossible"
#endif
#endif

/* fill in port pin defaults for known USART references */
#if USARTx_BASE == USART2_BASE
#define USARTx_TX_GPIO_PORT  GPIOA_BASE
#define USARTx_TX_GPIO_PIN   ((uint16_t)1 << 2)  /* GPIO_PIN_2 */
#define USARTx_RX_GPIO_PORT  GPIOA_BASE
#define USARTx_RX_GPIO_PIN   ((uint16_t)1 << 3)  /* GPIO_PIN_3 */
/* Do not specify if hardware flow control not supported */
#define USARTx_CTS_GPIO_PORT GPIOA_BASE
#define USARTx_CTS_GPIO_PIN  ((uint16_t)1 << 0)  /* GPIO_PIN_0 */
#define USARTx_RTS_GPIO_PORT GPIOA_BASE
#define USARTx_RTS_GPIO_PIN  ((uint16_t)1 << 1)  /* GPIO_PIN_1 */
#endif

/* Fill out matrix of port specifications if incomplete */
/* Best to specify all the pins explicitly and not rely on the logic below */
#if !defined(USARTx_RX_GPIO_PORT) && defined(USARTx_TX_GPIO_PORT)
#define USARTx_RX_GPIO_PORT USARTx_TX_GPIO_PORT
#endif
/* Most likely sequence of pins, not always true */
#if !defined(USARTx_RX_GPIO_PIN) && defined(USARTx_TX_GPIO_PIN)
#define USARTx_RX_GPIO_PIN (USARTx_TX_GPIO_PORT << 1)
#endif
#if !defined(USARTx_TX_GPIO_PORT) && defined(USARTx_RX_GPIO_PORT)
#define USARTx_TX_GPIO_PORT USARTx_RX_GPIO_PORT
#endif
/* Most likely sequence of pins, not always true */
#if !defined(USARTx_TX_GPIO_PIN) && defined(USARTx_RX_GPIO_PIN)
#define USARTx_TX_GPIO_PIN (USARTx_RX_GPIO_PORT >> 1)
#endif
/* Eliminate the mirage that one has individual control over CTS and RTS */
#if defined(USARTx_CTS_GPIO_PORT) || defined(USARTx_CTS_GPIO_PIN) \
 || defined(USARTx_RTS_GPIO_PORT) || defined(USARTx_RTS_GPIO_PIN)
#if !defined(USARTx_CTS_GPIO_PORT) && defined(USARTx_RTS_GPIO_PORT)
#define USARTx_CTS_GPIO_PORT USARTx_RTS_GPIO_PORT
#endif
#if !defined(USARTx_RTS_GPIO_PORT) && defined(USARTx_CTS_GPIO_PORT)
#define USARTx_RTS_GPIO_PORT USARTx_CTS_GPIO_PORT
#endif
#if !defined(USARTx_CTS_GPIO_PORT) && defined(USARTx_TX_GPIO_PORT)
#define USARTx_CTS_GPIO_PORT USARTx_TX_GPIO_PORT
#endif
#if !defined(USARTx_RTS_GPIO_PORT) && defined(USARTx_TX_GPIO_PORT)
#define USARTx_RTS_GPIO_PORT USARTx_TX_GPIO_PORT
#endif
/* Most likely sequence of pins, not always true */
#if !defined(USARTx_RTS_GPIO_PIN) && defined(USARTx_CTS_GPIO_PIN)
#define USARTx_RTS_GPIO_PIN (USARTx_CTS_GPIO_PORT << 1)
#endif
/* Most likely sequence of pins, not always true */
#if !defined(USARTx_CTS_GPIO_PIN) && defined(USARTx_RTS_GPIO_PIN)
#define USARTx_CTS_GPIO_PIN (USARTx_RTS_GPIO_PORT >> 1)
#endif
/* Most likely sequence of pins, not always true */
#if !defined(USARTx_RTS_GPIO_PIN) && defined(USARTx_TX_GPIO_PIN)
#define USARTx_RTS_GPIO_PIN (USARTx_TX_GPIO_PORT >> 1)
#endif
/* Most likely sequence of pins, not always true */
#if !defined(USARTx_CTS_GPIO_PIN) && defined(USARTx_TX_GPIO_PIN)
#define USARTx_CTS_GPIO_PIN (USARTx_TX_GPIO_PORT >> 2)
#endif
#endif /* Hardware Flow Control */

/* Arrange to generate clues to which RCC GPIO clocks need to be enabled */
#undef GPIO_AHBENR
#if defined(RCC_AHB1ENR_GPIOAEN)
#define GPIO_AHBENR(pre,post) pre ## AHB1ENR ## post
#elif defined(RCC_AHB2ENR_GPIOAEN)
#define GPIO_AHBENR(pre,post) pre ## AHB2ENR ## post
#endif
#undef GPIOA_RCC
#undef GPIOB_RCC
#undef GPIOC_RCC
#undef GPIOD_RCC
#undef GPIOE_RCC
#undef GPIOF_RCC
#undef GPIOG_RCC
#if (USARTx_TX_GPIO_PORT == GPIOA_BASE)
# define GPIOA_RCC
#elif (USARTx_TX_GPIO_PORT == GPIOB_BASE)
# define GPIOB_RCC
#elif (USARTx_TX_GPIO_PORT == GPIOC_BASE)
# define GPIOC_RCC
#elif (USARTx_TX_GPIO_PORT == GPIOD_BASE)
# define GPIOD_RCC
#elif (USARTx_TX_GPIO_PORT == GPIOE_BASE)
# define GPIOE_RCC
#elif (USARTx_TX_GPIO_PORT == GPIOF_BASE)
# define GPIOF_RCC
#elif (USARTx_TX_GPIO_PORT == GPIOG_BASE)
# define GPIOG_RCC
#endif
#if (USARTx_RX_GPIO_PORT == GPIOA_BASE)
# define GPIOA_RCC
#elif (USARTx_RX_GPIO_PORT == GPIOB_BASE)
# define GPIOB_RCC
#elif (USARTx_RX_GPIO_PORT == GPIOC_BASE)
# define GPIOC_RCC
#elif (USARTx_RX_GPIO_PORT == GPIOD_BASE)
# define GPIOD_RCC
#elif (USARTx_RX_GPIO_PORT == GPIOE_BASE)
# define GPIOE_RCC
#elif (USARTx_RX_GPIO_PORT == GPIOF_BASE)
# define GPIOF_RCC
#elif (USARTx_RX_GPIO_PORT == GPIOG_BASE)
# define GPIOG_RCC
#endif
#ifdef USARTx_RTS_GPIO_PORT
#if (USARTx_RTS_GPIO_PORT == GPIOA_BASE)
# define GPIOA_RCC
#elif (USARTx_RTS_GPIO_PORT == GPIOB_BASE)
# define GPIOB_RCC
#elif (USARTx_RTS_GPIO_PORT == GPIOC_BASE)
# define GPIOC_RCC
#elif (USARTx_RTS_GPIO_PORT == GPIOD_BASE)
# define GPIOD_RCC
#elif (USARTx_RTS_GPIO_PORT == GPIOE_BASE)
# define GPIOE_RCC
#elif (USARTx_RTS_GPIO_PORT == GPIOF_BASE)
# define GPIOF_RCC
#elif (USARTx_RTS_GPIO_PORT == GPIOG_BASE)
# define GPIOG_RCC
#endif
#endif
#ifdef USARTx_CTS_GPIO_PORT
#if (USARTx_CTS_GPIO_PORT == GPIOA_BASE)
# define GPIOA_RCC
#elif (USARTx_CTS_GPIO_PORT == GPIOB_BASE)
# define GPIOB_RCC
#elif (USARTx_CTS_GPIO_PORT == GPIOC_BASE)
# define GPIOC_RCC
#elif (USARTx_CTS_GPIO_PORT == GPIOD_BASE)
# define GPIOD_RCC
#elif (USARTx_CTS_GPIO_PORT == GPIOE_BASE)
# define GPIOE_RCC
#elif (USARTx_CTS_GPIO_PORT == GPIOF_BASE)
# define GPIOF_RCC
#elif (USARTx_CTS_GPIO_PORT == GPIOG_BASE)
# define GPIOG_RCC
#endif
#endif
#ifdef USARTx_DTR_GPIO_PORT
#ifndef USARTx_DTR_GPIO_ACTIVE
#define USARTx_DTR_GPIO_ACTIVE false
#endif
#if (USARTx_DTR_GPIO_PORT == GPIOA_BASE)
# define GPIOA_RCC
#elif (USARTx_DTR_GPIO_PORT == GPIOB_BASE)
# define GPIOB_RCC
#elif (USARTx_DTR_GPIO_PORT == GPIOC_BASE)
# define GPIOC_RCC
#elif (USARTx_DTR_GPIO_PORT == GPIOD_BASE)
# define GPIOD_RCC
#elif (USARTx_DTR_GPIO_PORT == GPIOE_BASE)
# define GPIOE_RCC
#elif (USARTx_DTR_GPIO_PORT == GPIOF_BASE)
# define GPIOF_RCC
#elif (USARTx_DTR_GPIO_PORT == GPIOG_BASE)
# define GPIOG_RCC
#endif
#endif

/* now we have an USARTx */
#define USARTx ((USART_TypeDef*)USARTx_BASE)

/* Send a character */
static inline bool send(uint8_t c) {
#ifdef USART_ISR_TXE
    if (USARTx->ISR & USART_ISR_TXE) {
        USARTx->TDR = c;
        return true;
    }
#else
    if (USARTx->SR & USART_SR_TXE) {
        USARTx->DR = c;
        return true;
    }
#endif
    return false;
}

/* Flush transmitter */
static inline bool complete(void) {
#ifdef USART_ISR_TC
    return USARTx->ISR & USART_ISR_TC;
#else
    return USARTx->SR & USART_SR_TC;
#endif
}

#if (defined(STM32G4))
#define flash_type uint64_t
#else
#define flash_type uint32_t
#endif

static struct {
    uint8_t  blocks_[32];
    uint32_t address_;    /* data transfer pointer               */
    uint8_t  checksum_;
    uint8_t  state_;      /* usart state machine                 */
    uint8_t  size_;       /* last character or transfer counter  */
    uint8_t  offset_;     /* offset into write buffer      */
    uint8_t  write_[256]; /* maximum length write memory command */
    bool     write_init_;
} state;

/* Grab a character */
static inline uint32_t receive(void) {
    uint32_t c = UINT32_MAX, sr;
#ifdef USART_ISR_RXNE
    sr = USARTx->ISR;
    if (sr & USART_ISR_RXNE) {
        c = USARTx->RDR & (uint8_t)UINT8_MAX;
        state.checksum_ ^= c;
    }
    if (!(sr & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE))) {
        return c;
    }
    USARTx->ICR = USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF
                        | USART_ICR_ORECF;
#else
    sr = USARTx->SR;
    if (sr & USART_SR_RXNE) {
        c = USARTx->DR & (uint8_t)UINT8_MAX;
        state.checksum_ ^= c;
    }
    if (!(sr & USART_SR_PE)) {
        return c;
    }
#endif
    if (state.state_ > 2) {  /* running     */
        state.state_ = 6;
    }
    state.checksum_ = 0;
    return UINT32_MAX;
}

/* Input character available */
static inline bool pending(void) {
#ifdef USART_ISR_RXNE
    return USARTx->ISR & USART_ISR_RXNE;
#else
    return USARTx->SR & USART_SR_RXNE;
#endif
}

static inline void initUSART(const uintptr_t base, const uint16_t pin) {
    GPIO_TypeDef *port = (GPIO_TypeDef*)base;
    port->BSRR |= (uint32_t)pin << 16;        /* Output set low           */
    port->OTYPER &= ~pin;                     /* Push Pull                */
    const uint32_t dpin = (uint32_t)pin * pin;
    const uint32_t dmask = ~(3 * dpin);
    port->OSPEEDR &= dmask;                   /* Low speed                */
    port->PUPDR &= dmask;                     /* No pull                  */
    const bool h = pin > ((uint16_t)1 << 7);  /* GPIO_PIN_7               */
    const uint64_t q = (uint64_t)dpin * dpin;
    const uint32_t qpin = h ? (uint32_t)(q >> 32) : (uint32_t)q;
    uint32_t temp = port->AFR[h];
    temp &= ~(15 * qpin);
#if (USARTx_BASE == USART1_BASE) || (USARTx_BASE == USART2_BASE) || (USARTx_BASE == USART3_BASE)
    temp |= 7 * qpin;                         /* Alternate USART1/2/3     */
#else
#if (defined(STM32F4))
    temp |= 8 * qpin;                         /* Alternate USART4/5/6/7/8 */
#else
    if (port >= GPIOC) {
        temp |= 5 * qpin;                     /* Alternate USART4/5       */
    } else {
        temp |= 8 * qpin;                     /* Alternate USART4/5       */
    }
#endif
#endif
    port->AFR[h] = temp;
    temp = port->MODER;
    temp &= dmask;
    temp |= 2 * dpin;                         /* Alternate mode           */
    port->MODER = temp;
}

static inline void deinitUSART(
    const uintptr_t base, const uint16_t pin, const bool active) {
    GPIO_TypeDef *port = (GPIO_TypeDef*)base;
    if (active) {
        port->BSRR |= pin;                    /* Output set high        */
    } else {
        port->BSRR |= (uint32_t)pin << 16;    /* Output set low         */
    }
    port->OTYPER |= pin;                      /* Open collector         */
    const uint32_t dpin = (uint32_t)pin * pin;
    const uint32_t dmask = ~(3 * dpin);
    port->OSPEEDR &= dmask;                   /* Low speed              */
                                              /* pullup / pulldown      */
    port->PUPDR = (port->PUPDR & dmask) | (active ? dpin : (2 * dpin));
    const bool h = pin > ((uint16_t)1 << 7);  /* GPIO_PIN_7             */
    const uint64_t q = (uint64_t)dpin * dpin;
    const uint32_t qpin = h ? (uint32_t)(q >> 32) : (uint32_t)q;
    uint32_t temp = port->AFR[h];
    temp &= ~(15 * qpin);
    port->AFR[h] = temp;                      /* AFR0                   */
    temp = port->MODER;
    temp &= dmask;
    temp &= ~(3 * dpin);                      /* Input mode             */
    port->MODER = temp;
}

void usart_init() {
    /* Enable clock source configuration NB: PCLK is default on reset */
#if (USARTx_BASE == USART1_BASE) && defined(RCC_USART1CLKSOURCE_PCLK2)
    RCC->CCIPR = (RCC->CCIPR & ~(RCC_CCIPR_USART1SEL)) | RCC_USART1CLKSOURCE_PCLK2;
#elif (USARTx_BASE == LPUART1_BASE) && defined(RCC_LPUART1CLKSOURCE_PCLK1)
    RCC->CCIPR = (RCC->CCIPR & ~(RCC_CCIPR_LPUART1SEL)) | RCC_LPUART1CLKSOURCE_PCLK1;
#elif (USARTx_BASE == USART2_BASE) && defined(RCC_USART2CLKSOURCE_PCLK1)
    RCC->CCIPR = (RCC->CCIPR & ~(RCC_CCIPR_USART2SEL)) | RCC_USART2CLKSOURCE_PCLK1;
#elif (USARTx_BASE == USART3_BASE) && defined(RCC_USART3CLKSOURCE_PCLK1)
    RCC->CCIPR = (RCC->CCIPR & ~(RCC_CCIPR_USART3SEL)) | RCC_USART3CLKSOURCE_PCLK1;
#elif (USARTx_BASE == UART4_BASE) && defined(RCC_UART4CLKSOURCE_PCLK1)
    RCC->CCIPR = (RCC->CCIPR & ~(RCC_CCIPR_UART4SEL)) | RCC_UART4CLKSOURCE_PCLK1;
#elif (USARTx_BASE == UART5_BASE) && defined(RCC_UART5CLKSOURCE_PCLK1)
    RCC->CCIPR = (RCC->CCIPR & ~(RCC_CCIPR_UART5SEL)) | RCC_UART5CLKSOURCE_PCLK1;
#endif

    /* Enable clock for USARTx */
#if (USARTx_BASE == USART1_BASE) && defined(RCC_APB2ENR_USART1EN)
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
#elif (USARTx_BASE == LPUART1_BASE) && defined(RCC_APB1ENR2_LPUART1EN)
    RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
#elif (USARTx_BASE == USART2_BASE) && defined(RCC_APB1ENR1_USART2EN)
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
#elif (USARTx_BASE == USART2_BASE) && defined(RCC_APB1ENR_USART2EN)
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
#elif (USARTx_BASE == USART3_BASE) && defined(RCC_APB1ENR1_USART3EN)
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
#elif (USARTx_BASE == USART3_BASE) && defined(RCC_APB1ENR_USART3EN)
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
#elif (USARTx_BASE == USART4_BASE) && defined(RCC_APB1ENR1_USART4EN)
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART4EN;
#elif (USARTx_BASE == UART4_BASE) && defined(RCC_APB1ENR1_UART4EN)
    RCC->APB1ENR1 |= RCC_APB1ENR1_UART4EN;
#elif (USARTx_BASE == UART4_BASE) && defined(RCC_APB1ENR_UART4EN)
    RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
#elif (USARTx_BASE == USART5_BASE) && defined(RCC_APB1ENR1_USART5EN)
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART5EN;
#elif (USARTx_BASE == UART5_BASE) && defined(RCC_APB1ENR_UART5EN)
    RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
#elif (USARTx_BASE == USART6_BASE) && defined(RCC_APB2ENR_USART6EN)
    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
#elif (USARTx_BASE == UART7_BASE) && defined(RCC_APB1ENR_UART7EN)
    RCC->APB1ENR |= RCC_APB1ENR_UART7EN;
#elif (USARTx_BASE == UART8_BASE) && defined(RCC_APB1ENR_UART8EN)
    RCC->APB1ENR |= RCC_APB1ENR_UART8EN;
#else
#error "Select USARTx"
#undef USARTx
#define USARTx ((USART_TypeDef*)0)
#endif

    RCC->GPIO_AHBENR(,) |=
#ifdef GPIOA_RCC
                     GPIO_AHBENR(RCC_,_GPIOAEN) |
#endif
#ifdef GPIOB_RCC
                     GPIO_AHBENR(RCC_,_GPIOBEN) |
#endif
#ifdef GPIOC_RCC
                     GPIO_AHBENR(RCC_,_GPIOCEN) |
#endif
#ifdef GPIOD_RCC
                     GPIO_AHBENR(RCC_,_GPIODEN) |
#endif
#ifdef GPIOE_RCC
                     GPIO_AHBENR(RCC_,_GPIOEEN) |
#endif
#ifdef GPIOF_RCC
                     GPIO_AHBENR(RCC_,_GPIOFEN) |
#endif
#ifdef GPIOG_RCC
                     GPIO_AHBENR(RCC_,_GPIOGEN) |
#endif
                     0;
    initUSART(USARTx_TX_GPIO_PORT, USARTx_TX_GPIO_PIN);
    initUSART(USARTx_RX_GPIO_PORT, USARTx_RX_GPIO_PIN);
#ifdef USARTx_RTS_GPIO_PORT
    initUSART(USARTx_RTS_GPIO_PORT, USARTx_RTS_GPIO_PIN);
#endif
#ifdef USARTx_CTS_GPIO_PORT
    initUSART(USARTx_CTS_GPIO_PORT, USARTx_CTS_GPIO_PIN);
#endif

    /* Configure USARTx */
    USARTx->CR1 = 0; /* Reset configuration */
    USARTx->CR2 = 0;
    USARTx->CR3 = 0;
    USARTx->RTOR = 0;
    USARTx->PRESC = 0; /* divided by 1 */
    /* 9 data bits even parity */
    USARTx->CR1 = USART_CR1_PCE | USART_CR1_RE | USART_CR1_TE
#if defined(USART_CR1_FIFOEN)
                | USART_CR1_FIFOEN  /* Enable FIFO mode */
#endif
#if defined(USART_CR1_M0)
                | USART_CR1_M0;
#else
                | USART_CR1_M;
#endif
    USARTx->CR2 = USART_CR2_STOP_1;  /* 2 stop bits */
#if defined(USARTx_CTS_GPIO_PORT) || defined(USARTx_RTS_GPIO_PORT) \
            || defined(USART_CR1_FIFOEN)
    USARTx->CR3 = 0
#ifdef USARTx_CTS_GPIO_PORT
                   | USART_CR3_CTSE
#endif
#ifdef USARTx_RTS_GPIO_PORT
                   | USART_CR3_RTSE
#endif
#if defined(USART_CR1_FIFOEN)
                   | USART_CR3_TXFTCFG_2 | USART_CR3_RXFTCFG_2
#endif
                   ;
#endif
    /* Oversampling by 16 */
#if (USART_BAUD_RATE == _AUTO) || ((USART_BAUD_RATE + 0) == 0)
    USARTx->BRR = (uint32_t)(((SystemCoreClock) + (921600 / 2)) / 921600);
#else
    USARTx->BRR = (uint32_t)(((SystemCoreClock) + ((USART_BAUD_RATE) / 2))
                / (USART_BAUD_RATE));
#endif
    USARTx->CR1 |= USART_CR1_UE;
    /* DTR asserted now that we are ready */
#ifdef USARTx_DTR_GPIO_PORT
    /* YES we mean deinitUSART!!!, there is no chip H/W for DTR */
    deinitUSART(USARTx_DTR_GPIO_PORT, USARTx_DTR_GPIO_PIN, USARTx_DTR_GPIO_ACTIVE);
#endif
#if ((USART_BAUD_RATE == _AUTO) || (USART_BAUD_RATE == 0)) \
            && defined(IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE)
    if (IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(USARTx)) {
        /* Since we have the FIFO enabled, flush content before enabling ABR */
        while (pending()) {
            receive();
        }
        LL_USART_SetAutoBaudRateMode(
            USARTx, LL_USART_AUTOBAUD_DETECT_ON_7F_FRAME);
        state.state_ = 8;      /* set to autobaud algorithm */
    } else
#endif
        state.state_ = 0;      /* reset and skip read 0x7F */
}

void usart_deinit() {
#ifdef USARTx_DTR_GPIO_PORT
    deinitUSART(USARTx_DTR_GPIO_PORT, USARTx_DTR_GPIO_PIN, !USARTx_DTR_GPIO_ACTIVE);
#endif
    deinitUSART(USARTx_TX_GPIO_PORT, USARTx_TX_GPIO_PIN, true);
    deinitUSART(USARTx_RX_GPIO_PORT, USARTx_RX_GPIO_PIN, true);
#ifdef USARTx_RTS_GPIO_PORT
    deinitUSART(USARTx_RTS_GPIO_PORT, USARTx_RTS_GPIO_PIN, true);
#endif
#ifdef USARTx_CTS_GPIO_PORT
    deinitUSART(USARTx_CTS_GPIO_PORT, USARTx_CTS_GPIO_PIN, true);
#endif
    /* Disable clock for USARTx */
#if (USARTx_BASE == USART1_BASE) && defined(RCC_APB2ENR_USART1EN)
    RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
#elif (USARTx_BASE == LPUART1_BASE) && defined(RCC_APB1ENR2_LPUART1EN)
    RCC->APB1ENR2 &= ~RCC_APB1ENR2_LPUART1EN;
#elif (USARTx_BASE == USART2_BASE) && defined(RCC_APB1ENR1_USART2EN)
    RCC->APB1ENR1 &= ~RCC_APB1ENR1_USART2EN;
#elif (USARTx_BASE == USART2_BASE) && defined(RCC_APB1ENR_USART2EN)
    RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
#elif (USARTx_BASE == USART3_BASE) && defined(RCC_APB1ENR1_USART3EN)
    RCC->APB1ENR1 &= ~RCC_APB1ENR1_USART3EN;
#elif (USARTx_BASE == USART3_BASE) && defined(RCC_APB1ENR_USART3EN)
    RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN;
#elif (USARTx_BASE == USART4_BASE) && defined(RCC_APB1ENR1_USART4EN)
    RCC->APB1ENR1 &= ~RCC_APB1ENR1_USART4EN;
#elif (USARTx_BASE == UART4_BASE) && defined(RCC_APB1ENR1_UART4EN)
    RCC->APB1ENR1 &= ~RCC_APB1ENR1_UART4EN;
#elif (USARTx_BASE == UART4_BASE) && defined(RCC_APB1ENR_UART4EN)
    RCC->APB1ENR &= ~RCC_APB1ENR_UART4EN;
#elif (USARTx_BASE == USART5_BASE) && defined(RCC_APB1ENR1_USART5EN)
    RCC->APB1ENR1 &= ~RCC_APB1ENR1_USART5EN;
#elif (USARTx_BASE == UART5_BASE) && defined(RCC_APB1ENR_UART5EN)
    RCC->APB1ENR &= ~RCC_APB1ENR_UART5EN;
#elif (USARTx_BASE == USART6_BASE) && defined(RCC_APB2ENR_USART6EN)
    RCC->APB2ENR &= ~RCC_APB2ENR_USART6EN;
#elif (USARTx_BASE == UART7_BASE) && defined(RCC_APB1ENR_UART7EN)
    RCC->APB1ENR &= ~RCC_APB1ENR_UART7EN;
#elif (USARTx_BASE == UART8_BASE) && defined(RCC_APB1ENR_UART8EN)
    RCC->APB1ENR &= ~RCC_APB1ENR_UART8EN;
#else
#error "Select USARTx"
#undef USARTx
#define USARTx ((USART_TypeDef*)0)
#endif
    /* do not disable GPIO clock */
}

extern void heartbeat(void);

void usart_poll() {
    /* Prepackaged static content */
#define ACK  0x79
#define NACK 0x1F
#define VER  0x40
#if (DFU_CAN_UPLOAD == _ENABLE)
    static const uint8_t getCommands[11] = {
        ACK, sizeof(getCommands) - 4, VER, 0x00, 0x01, 0x02, 0x11, 0x21,
        0x31, 0x43, ACK
    };
#else
    static const uint8_t getCommands[10] = {
        ACK, sizeof(getCommands) - 4, VER, 0x00, 0x01, 0x02, 0x21,
        0x31, 0x43, ACK
    };
#endif
    static const uint8_t getVersion[5] = {
        ACK, VER, 0x00, 0x00, ACK
    };
    static uint8_t getId[5] = {
        ACK, sizeof(getId) - 4, 0x04, 0x19, ACK
    };
    uint32_t c;

    /* Deal with state machine */
    switch (state.state_) {
    /* reset */
    case 0:
        c = receive();
        if (c != 0x7F) {
            break;
        }
    ack_done:
        state.state_ = 1;
        /* FALLTHRU */

    /* received 0x7F */
    case 1:  /* send ACK */
        if (!send(ACK)) {
            break;
        }
        state.checksum_ = 0;
        state.state_++;
        /* FALLTHRU */

    /* idle */
    case 2:
        c = receive();
        if (c == UINT32_MAX) {
            break;
        }
        if (c == 0x7F) {
            goto ack_done;
        }
        state.size_ = c & UINT8_MAX;
        state.state_++;
        /* FALLTHRU */

    /* idle command checksum (exclusive NOR special) */
    case 3:
        c = receive();
        if (c == UINT32_MAX) {
            break;
        }
        state.state_++;
        state.checksum_ ^= 0xFF;
        if (state.checksum_) {
            /* resync, perhaps _this_ is command */
            state.size_ = state.checksum_ = c;
            /* FALLTHRU */

    /* send NACK and loop to idle command checksum */
    case 4: if (send(NACK)) {
                state.state_--;
            }
            break;
        }
        state.state_++;
        /* FALLTHRU */

    /* Interpret command */
    case 5:
        switch (state.size_) {
        /* Get */
        case 0x00:
            state.address_ = (uintptr_t)getCommands;
            state.size_ = sizeof(getCommands);
            goto read;

        /* Get Version & Read Protection Status */
        case 0x01:
            state.address_ = (uintptr_t)getVersion;
            state.size_ = sizeof(getVersion);
            goto read;

        /* Get ID */
        case 0x02:
            {
                uint16_t devid = DBGMCU->IDCODE & DBGMCU_IDCODE_DEV_ID_Msk;
                getId[2] = (devid >> 8) & UINT8_MAX;
                getId[3] = devid & UINT8_MAX;
            }
            state.address_ = (uintptr_t)getId;
            state.size_ = sizeof(getId);
            goto read;

#if (DFU_CAN_UPLOAD == _ENABLE)
        /* Read Memory */
        case 0x11:
            state.state_ = 10;
            goto ack_command;
#endif

        /* Go */
        case 0x21:
            state.state_ = 30;
            goto ack_command;

        /* Write Memory */
        case 0x31:
            state.state_ = 40;
            goto ack_command;

        /* Erase */
        case 0x43:
            state.state_ = 60;
            goto ack_command;

        default:
            break;
        }
        /* FALLTHRU */

    default:
    nack:
        state.state_ = 6;
        /* FALLTHRU */

    /* send NACK and go back to idle */
    case 6:
        if (send(NACK)) {
            state.state_ = 2;  /* idle */
            state.checksum_ = 0;
            state.write_init_ = false;
        }
        break;

    read:
        heartbeat();
        state.state_ = 7;
        /* FALLTHRU */

    /* send data pointed to by address */
    case 7:
        while (send(*(uint8_t*)(uintptr_t)state.address_)) {
            state.address_++;
            state.size_--;
            if (!state.size_) {
                state.state_ = 2;  /* idle */
                break;
            }
        }
        break;

    /* autobaud */
    case 8:
#if ((USART_BAUD_RATE == _AUTO) || (USART_BAUD_RATE == 0)) \
            && defined(IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE)
        if (IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(USARTx)) {
            state.state_ = 0;
            break;
        }
#endif
        /* TBD our own 0x7F algorithm on the GPIO pin */
        state.state_ = 0;
        break;

    ack_command:
        heartbeat();
#if (DFU_CAN_UPLOAD == _ENABLE)
    case 10:   /* Read Memory       */
#endif
    case 30:   /* Go                */
    case 40:   /* Write Memory      */
    case 60:   /* Erase             */
        if (send(ACK)) {
            state.state_++;
        }
        break;

#if (DFU_CAN_UPLOAD == _ENABLE)  /* receive 5 byte address */
    /* Read Memory: get address field */
    case 11:
#endif
    /* Go: get address field */
    case 31:
    /* Write Memory: get address field */
    case 41:
        state.address_ = 0;
        /* FALLTHRU */

    /* pick up the address field byte-by-byte */
#if (DFU_CAN_UPLOAD == _ENABLE)
    case 12: case 13: case 14:
#endif
    case 32: case 33: case 34:
    case 42: case 43: case 44:
        c = receive();
        if (c == UINT32_MAX) {
            break;
        }
        state.address_ |= (uint32_t)c
                        << ((4 - (state.state_ % 10)) * 8);
        state.state_++;
        break;

#if (DFU_CAN_UPLOAD == _ENABLE)
    /* Read Memory: checksum */
    case 15:
#endif
    /* Go: checksum */
    case 35:
    /* Write Memory: checksum */
    case 45:
        c = receive();
        if (c == UINT32_MAX) {
            break;
        }
        if (state.checksum_) {
            goto nack;
        }
	if ((invalid_address(state.address_, sizeof(state.write_)))
	 && ((state.state_ != 35)  /* GO allowed to point to bootloader */
	  || (state.address_ < ((uintptr_t)&__isr_vector))
	  || (((uintptr_t)&__etext) <= state.address_))) {
            goto nack;
        }
        state.state_++;
        /* FALLTHRU */

#if (DFU_CAN_UPLOAD == _ENABLE)
    /* Read Memory: validate */
    case 16:
#endif
    /* Go: validate */
    case 36:
    /* Write Memory: validate */
    case 46:
        if (!send(ACK)) {
            break;
        }
        state.state_++;
        break;

    case 37:
        if (!complete()) {
            break;
        }
        /* Go */
#if (DFU_BOOTKEY_ADDR == _DISABLE)
        *((uint32_t*)_KEY_ADDR) = 0;
#else
        /* are we requesting reboot into the bootblock? */
        *((uint32_t*)_KEY_ADDR) =
            ((((uintptr_t)&__isr_vector) <= state.address_)
          && (state.address_ < (((uintptr_t)&__isr_vector) + 16 * 1024)))
                ? DFU_BOOTKEY
                : 0;
#endif
        NVIC_SystemReset();
        state.state_ = 0;  /* reset */
        break;

#if (DFU_CAN_UPLOAD == _ENABLE)
    /* Read Memory       */
    case 17:
        c = receive();
        if (c == UINT32_MAX) {
            break;
        }
        state.size_ = c & UINT8_MAX;
        state.state_++;
        break;

    /* ~number of bytes to read */
    case 18:
        c = receive();
        if (c == UINT32_MAX) {
            break;
        }
        if (state.checksum_) {
            goto nack;
        }
        state.state_++;
        /* FALLTHRU */

    case 19:
        if (!send(ACK)) {
            break;
        }
        if (!state.write_init_) {
            aes_init();
            state.write_init_ = true;
        }
        aes_encrypt(
            state.write_,
            (uint8_t*)(uintptr_t)state.address_,
            (size_t)state.size_ + 1);
        state.address_ = (uintptr_t)state.write_;
        state.size_++;  /* yes, uint8_t value of 0 == 256 */
        goto read;
#endif

    /* Write Memory: data */
    case 47:
        c = receive();
        if (c == UINT32_MAX) {
            break;
        }
        state.size_ = c & UINT8_MAX;  /* N - 1 */
        /* size must be aligned to device block size */
        if (((size_t)state.size_ + 1) & (sizeof(flash_type) - 1)) {
            goto nack;
        }
        state.offset_ = 0;
        state.state_++;
        /* FALLTHRU */

    next_write_char:
    /* write one byte to buffer */
    case 48:
        c = receive();
        if (c == UINT32_MAX) {
            break;
        }
        state.write_[state.offset_] = c & UINT8_MAX;
        state.offset_++;
        if (state.size_) {
            state.size_--;
            if (pending()) {
                goto next_write_char;
            }
            break;
        }
        state.state_++;
        /* FALLTHRU */

    case 49:
        c = receive();
        if (c == UINT32_MAX) {
            break;
        }
        if (state.checksum_) {
            goto nack;
        }
        /* Erase/Flash */
        if (!state.write_init_) {
            aes_init();
            state.write_init_ = true;
        }
        aes_decrypt(state.write_, state.write_, state.offset_);
        if (flash_address(state.address_, state.offset_)) {
            c = program_flash(
                (void*)(uintptr_t)state.address_,
                state.write_,
                state.offset_);
            if (c) {
                goto nack;
            }
        } else {
            memcpy(
                (void*)(uintptr_t)state.address_,
                state.write_,
                state.offset_);
        }
        goto ack_done;

    case 61:
        c = receive();
        if (c == UINT32_MAX) {
            break;
        }
        state.state_++;
        if (c != 0xFF) {
                state.size_ = c;
                (void)memset(
                        state.blocks_,
                        0,
                        sizeof(state.blocks_));
                state.state_++;
                goto erase_pages;
        }
        /* FALLTHRU */

    case 62:
        c = receive();
        if (c == UINT32_MAX) {
            break;
        }
        if (c != 0) {
            /* AN3155 says to respond ACK here ... */
            goto nack;
        }
        /* Global erase */
        /* plan is todo nothing, let erase-before-flash function do its job */
        goto ack_done;

    erase_pages:
    case 63:
        c = receive();
        if (c == UINT32_MAX) {
            break;
        }
        state.blocks_[c / (sizeof(state.blocks_[0]) * 8)]
                |= 1 << (c & ((sizeof(state.blocks_[0]) * 8) - 1));
        if (state.size_ != 0) {
            state.size_--;
            break;
        }
        state.state_++;
        /* FALLTHRU */

    case 64:
        c = receive();
        if (c == UINT32_MAX) {
            break;
        }
        if (state.checksum_) {
            goto nack;
        }
        state.state_++;
        /* FALLTHRU */

    case 65:
        /* Erase list of pages */
        /* plan is to do nothing, let erase-before-flash function do its job */
        goto ack_done;
    }
}

#endif  /* DFU_USART */
