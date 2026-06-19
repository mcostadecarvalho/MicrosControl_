/* 
 * main.c 
 * Problema – 1 – (65p) 
 *
 * Se desea implementar un sistema de alarmas para un domicilio con 4 
 * habitaciones de interés, para lo que se cuenta con los siguientes materiales: 
 *
 * Sensores: 
 * 4 sensores PIR de presencia que actúan como contacto seco NA 
 * 1 pulsador NC 
 * 1 sensor de CO, el cual entrega una señal analógica con una ley de variación 𝑉CO = 𝜆 sqrt(𝑅CO) donde: 
 *      λ es una constante con unidades de [V*ppm]
 *      𝑅CO es la concentración de CO medida en [ppm] 
 *      𝑉CO es la tensión analógica que varía entre 0 – 1 [V] 
 *
 * Actuadores: 
 * Un transistor TIP122. 
 * Un diodo 1N4007. 
 * Resistores varios. 
 * Un LED rojo. 
 * Una sirena de 20 [W] a 12 [VDC]. 
 * Un Relay SPDT con 12VDC y 0.5[W] de bobina. 
 *
 * Comunicación: 
 * Un módulo SIM800L el cual recibe comandos AT mediante 
 * comunicación serie a 9600 [bps]. 
 * 
 * Microcontrolador: 
 * Un microcontrolador de 16 bits con alimentación de 5 [V], que posee: 
 * 1 ADC de 10 bits 
 * 2 puertos de 8 GPIOS con las siguientes características. 
 *  Io < 20 [mA] 
 *  VoH > 4.5 [V] y VoL< 0.3 [V] 
 *  ViH > 4.9 [V] y ViL< 0.1 [V] 
 * 2 puertos USART 
 *  2 Timers de 8 bits 
 *  1 Timer de 16 bits 
 *  Oscilador interno de 16 [MHz] 
 *  
 * Nota (I): Los Timers poseen prescaler configurable como divisor por 8, 16, 32  
 * 
 * Nota (2): Los comandos AT son del tipo: 
 *  “AT” + “Línea del Equipo” 
 *  “AT” + “Línea Destino” 
 *  “AT” + “Mensaje”
 * 
 *  En base al inciso anterior realizar un diagrama de estados que modele el 
 *  comportamiento de la alarma. [20 pts] 
 * 
 *  Empleando los drivers proporcionados en el ANEXO, se solicita escribir 
 *  una API que permita hacer uso del Hardware disponible. [20 pts] 
 *  
 *  Codificar la máquina de estados de inciso  [20 pts] 
 * 
 */

#include "main.h"
#include <stdio.h>
#include <stdint.h>

#define PIN_PIR1        ((uint8_t)0)
#define PIN_PIR2        ((uint8_t)1)
#define PIN_PIR3        ((uint8_t)2)
#define PIN_PIR4        ((uint8_t)3)
#define PIN_BUTTON      ((uint8_t)4)

#define PIN_RELAY        ((uint8_t)5)
#define PIN_LED_ALARMA   ((uint8_t)6)

#define ADC_PIN_CO       ((uint8_t)7) // Pin del ADC para el sensor de CO

#define SWITCH_ON        ((uint8_t)1)
#define SWITCH_OFF       ((uint8_t)0)

#define LED_ON           ((uint8_t)1)
#define LED_OFF          ((uint8_t)0)

#define RELAY_ON         ((uint8_t)1)
#define RELAY_OFF        ((uint8_t)0)

#define CO_ADC_UMBRAL    ((uint16_t)200u)

#define DEBOUNCE_DELAY_MS ((uint8_t)20u)

// #define EVENT_PIR_ANY    ((uint32_t)(1u << 0))
// #define EVENT_BTN_PRESS  ((uint32_t)(1u << 1))

/* 
 * Cálculo de PSC y ARR para Timer de 16 bits (TIM3):
 * Fórmula: ARR = (F_clk / (PSC * F_desired)) - 1
 * 
 * Con F_clk = 16 MHz y PSC = 16:
 *   F_timer = 16 MHz / 16 = 1 MHz
 *   T_tick = 1 us
 * 
 */
#define PSC              ((uint16_t)16u)  // Prescaler: divide 16 MHz a 1 MHz
#define ARR              ((uint16_t)1000u-1u) // Autoreload para 1 ms (1000 us)

TIM_Instance_type tim3 = TIM3;
UART_Instance_type uart1 = UART1;
ADC_Instance_type hadc1;

TIM_type htim3 = { tim3, ARR, 0u, PSC };

UART_type huart1 = { uart1, 9600u, 8u, 0u, 1u, 1u, RX_TX_MODE};
// Configuración UART a 9600 bps, 8 bits, sin paridad, 1 start/stop. RX_TX_MODE para SIM800L (recibir respuestas AT).

ADC_Instance_type hadc1; 
// Configuración para el ADC, si es necesario

GPIO_Init_type pir1 = {PIN_PIR1, GPIOB, PULL_DOWN, GIO_INPUT};
GPIO_Init_type pir2 = {PIN_PIR2, GPIOB, PULL_DOWN, GIO_INPUT};
GPIO_Init_type pir3 = {PIN_PIR3, GPIOB, PULL_DOWN, GIO_INPUT};
GPIO_Init_type pir4 = {PIN_PIR4, GPIOB, PULL_DOWN, GIO_INPUT};

GPIO_Init_type button = {PIN_BUTTON, GPIOB, PULL_DOWN, GIO_INPUT}; // Pulsador NC, por lo que se configura con pull-down para leer 1 lógico cuando no está presionado. 

GPIO_Init_type relay = {PIN_RELAY, GPIOB, PUSH_PULL, GIO_OUTPUT};
GPIO_Init_type led_alarma = {PIN_LED_ALARMA, GPIOB, PUSH_PULL, GIO_OUTPUT};

typedef enum{
	pressed = SWITCH_OFF,
	non_pressed = SWITCH_ON
}button_state;

static volatile uint8_t rg_tick_100ms = 0u; // Tick lógico para FSM/ADC
static volatile uint32_t rg_counter_debounce = 0u; // Contador para debounce del botón

void Timer3_ISR(void);
static uint8_t leer_pir(void);
static button_state switch_debounced(uint8_t GPIO_PORT, uint8_t GPIO_PIN);
static void enviar_sms_uart(const uint8_t *msg);

typedef enum system_state {
    INIT,
    WAIT,
    ALARM,
    ERROR
} system_state_t;

int main(void) {

    system_state_t state = INIT;
    uint16_t co_adc = 0u;
    uint8_t alarm_sms_sent = 0u;
    uint8_t btn_pressed_event = 0u;
    uint8_t pir_flag = 0u;
    uint32_t alarm_elapsed_ms = 0u;  // Tiempo acumulado en estado ALARM para reenvío de SMS

    GPIO_HwInit(&pir1);
    GPIO_HwInit(&pir2);
    GPIO_HwInit(&pir3);
    GPIO_HwInit(&pir4);
    GPIO_HwInit(&button);
    GPIO_HwInit(&led_alarma);
    GPIO_HwInit(&relay);

    ADC_HwInit(&hadc1); // Inicializar ADC para sensor de CO

    Timer_HwInit(&htim3);
    Timer_HwStart(htim3.instance);
    UART_HwInit(&huart1);
    UART_TX_Start();
    UART_RX_Start(); // Habilitar recepción para leer respuestas del SIM800L

    while (1) {
        
        switch (state) {
            case INIT:
                GPIO_WritePin(GPIOB, PIN_LED_ALARMA, LED_OFF);
                
                GPIO_WritePin(GPIOB, PIN_RELAY, RELAY_OFF);
            
                btn_pressed_event = 0u;
                alarm_elapsed_ms = 0u;
                alarm_sms_sent = 0u;
                enviar_sms_uart((uint8_t *)"AT+SMS=\"Sistema Iniciado\"\r\n");

                state = WAIT;
                
                break;

            case WAIT:
                
                if (rg_tick_100ms != 0u) 
                {
                    rg_tick_100ms = 0u;

                    if (leer_pir() != 0u){
                    
                        pir_flag = 1u;    
                    
                    } 
                    else {
                        
                        pir_flag = 0u;
                    }

                    if (pir_flag != 0u) co_adc = ADC_GetValue();
                
                    if (pir_flag != 0u && co_adc > CO_ADC_UMBRAL){
                        state = ALARM;
                        btn_pressed_event = 0u;
                        alarm_elapsed_ms = 0u; 
                        alarm_sms_sent = 0u;
                    }

                    else {

                        state = WAIT;
                    }
        
                }    
                break;

            case ALARM:
                
                GPIO_WritePin(GPIOB, PIN_LED_ALARMA, LED_ON);

                GPIO_WritePin(GPIOB, PIN_RELAY, RELAY_ON);

                /* Muestreo continuo del botón: debounce temporal lo maneja switch_debounced con rg_counter_debounce. */
                if (switch_debounced(GPIOB, PIN_BUTTON) == pressed) {

                    btn_pressed_event = 1u;
                }
                
                else {

                    btn_pressed_event = 0u;
                }

                if (rg_tick_100ms != 0u) {
                    rg_tick_100ms = 0u;

                    /* Enviar SMS inicial de alarma */
                    if (alarm_sms_sent == 0u) {
                        enviar_sms_uart((uint8_t *)"AT+SMS=\"ALARMA\"\r\n");
                        alarm_sms_sent = 1u;
                    }

                    alarm_elapsed_ms += 100u;

                    /* Reenviar SMS cada 2 minutos si botón no está presionado */
                    if (alarm_elapsed_ms >= 120000u && btn_pressed_event != 1u) {
                        enviar_sms_uart((uint8_t *)"AT+SMS=\"ALARMA\"\r\n");
                        alarm_elapsed_ms = 0u;
                    }

                }
                if (btn_pressed_event != 0u) {
                    
                    GPIO_WritePin(GPIOB, PIN_LED_ALARMA, LED_OFF);
                    
                    GPIO_WritePin(GPIOB, PIN_RELAY, RELAY_OFF);

                    alarm_sms_sent = 0u;
                    
                    state = WAIT;
                }

                break;

            case ERROR:
                /* Enviar SMS de error y volver a WAIT */
                enviar_sms_uart((uint8_t *)"AT+SMS=\"Error\"\r\n");
                state = WAIT;
                break;

            default:
                /* Estado desconocido: enviar error y reintentar */
                enviar_sms_uart((uint8_t *)"AT+SMS=\"Error\"\r\n");
                state = WAIT;
                break;
        }
    }

    return 0;
}

void Timer3_ISR(void) {
    static uint8_t div_100ms = 0u;

    div_100ms++;
    if(div_100ms >= 100u){
        div_100ms = 0u;
        rg_tick_100ms = 1u;
    }

    if (rg_counter_debounce > 0u) {

        rg_counter_debounce--;
    }

}

// pressed = 0 lógico (SWITCH_OFF), non_pressed = 1 lógico (SWITCH_ON) configuración pull-down pulsador NC.
button_state switch_debounced (uint8_t GPIO_PORT, uint8_t GPIO_PIN){

    static button_state stable_state = non_pressed; // Estado estable actual del botón
    static button_state last_state = non_pressed;   // Último estado leído del botón
    
    button_state actual_state;
     
	/* Read Pin */
	actual_state = (GPIO_ReadPin(GPIO_PORT, GPIO_PIN) == SWITCH_OFF) ? pressed:non_pressed;

	/* Check if There is a State Change */
	if(actual_state != last_state){
		/* Save previous state */
		last_state = actual_state;
		/* Delay Start */
		rg_counter_debounce = DEBOUNCE_DELAY_MS;
	}
	else{
		/* Delay Ended */
		if((rg_counter_debounce == 0) && (last_state != stable_state)){
			/* If its maintain equal after DEBOUNCE_DELAY_MS */
			stable_state = last_state;
		}
	}

	return(stable_state);
}

static uint8_t leer_pir(void){
    return (uint8_t)(GPIO_ReadPin(GPIOB, PIN_PIR1) | GPIO_ReadPin(GPIOB, PIN_PIR2) | GPIO_ReadPin(GPIOB, PIN_PIR3) | GPIO_ReadPin(GPIOB, PIN_PIR4));
}

/* Envía un mensaje SMS genérico al módulo SIM800L vía UART */
static void enviar_sms_uart(const uint8_t *msg) {
    uint8_t len = 0u;
    /* Calcular longitud hasta null terminator */
    while (msg[len] != '\0' && len < 100u) {
        len++;
    }
    UART_Blocking_Transmit(uart1, (uint8_t *)msg, len);
}

