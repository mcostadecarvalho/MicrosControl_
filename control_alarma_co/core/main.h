/*
 *  main.h 
 *  Created on: 17 de jun. de 2024
 *  Modelo de parcial para el curso de Microcontroladores y Microprocesadores
 *  Librerias para la gestión de timers, GPIOs, ADC y UART.
 *  
 *  En base al inciso anterior realizar un diagrama de estados que modele el 
 *  comportamiento de la alarma. [20 pts] 
 * 
 *  Empleando los drivers proporcionados en el ANEXO, se solicita escribir 
 *  una API que permita hacer uso del Hardware disponible. [20 pts] 
 *  
 *  Codificar la máquina de estados de inciso  [20 pts] 
 */ 

#include <stdint.h>

/* Timers */ 
#define TIM1 ((TIM_Instance_type) 1) // 8 bit Timer 
#define TIM2 ((TIM_Instance_type) 2) // 8 bit Timer 
#define TIM3 ((TIM_Instance_type) 3) // 16 bit Timer 

#define Timer_CleanFlags(ins,tim) ((tim.instance) &= (~(1<<ins))) // Limpiar bandera de interrupción para la instancia dada 

typedef uint8_t TIM_Instance_type; 

typedef struct{ 
  TIM_Instance_type instance; 
  uint16_t period; 
  uint8_t up_down; 
  uint16_t prescaler; 
}TIM_type; 

void Timer1_ISR(void); 
void Timer2_ISR(void); 
void Timer3_ISR(void); 
void Timer_HwInit(TIM_type*); 
void Timer_HwStart(TIM_Instance_type); 

/* GPIOs */ 
#define GPIOA ((uint8_t) 1) 
#define GPIOB ((uint8_t) 2)  

#define GIO_INPUT ((uint8_t) 1) 
#define GIO_OUTPUT ((uint8_t) 2) 

#define PULL_DOWN ((uint8_t) 1) 
#define PULL_UP ((uint8_t) 2) 
#define PUSH_PULL ((uint8_t) 3) 

#define SET ((uint8_t) 1)
#define CLEAR ((uint8_t) 0)

typedef uint16_t GPIO_Pin_type; 

typedef struct{ 
  GPIO_Pin_type pin; 
  uint8_t port; 
  uint8_t output_type; 
  uint8_t input_output; 
}GPIO_Init_type; 

void GPIO_HwInit(GPIO_Init_type *); 
uint8_t GPIO_ReadPin(uint8_t port, uint8_t pin); 
void GPIO_WritePin(uint8_t port, uint8_t pin, uint8_t value); // value: 0 o 1 omito o no ?

/* ADC */ 

typedef uint8_t ADC_Instance_type; // no se si eso va a ser un typedef, pero lo dejo así por ahora

uint16_t ADC_GetValue(void); 
void ADC_HwInit(ADC_Instance_type *); // no se si eso va a ser un declarado tambien, pero lo pongo

/* UART */ 
#define UART1 ((UART_Instance_type) 1) 
#define UART2 ((UART_Instance_type) 2) 

#define TX_MODE ((uint8_t) 1) 
#define RX_MODE ((uint8_t) 2) 
#define RX_TX_MODE ((uint8_t) 3) 

#define RX_CleanFlags(ins,uart) ((uart.instance) &= (~(1<<ins)))  // Limpiar bandera de recepción para la instancia dada

typedef uint8_t UART_Instance_type; // no se si eso va a ser un typedef, pero lo dejo así por ahora

typedef struct{ 
  UART_Instance_type instance; 
  uint16_t baudrate; 
  uint8_t data_bits; 
  uint8_t parity; 
  uint8_t start_bits; 
  uint8_t stop_bits; 
  uint8_t mode; 
}UART_type; 

void UART_HwInit(UART_type *); 
uint8_t UART_TX_Start(void); 
uint8_t UART_RX_Start(void); 
uint8_t UART_RX_ISR(void); 
void UART_Receive_IT(UART_Instance_type uart, uint8_t *buffer, uint8_t length); 
void UART_Blocking_Transmit(UART_Instance_type uart, uint8_t *buffer, uint8_t length); 