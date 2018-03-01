#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f3/nvic.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/timer.h>

#include "BAP_task.h"
#include "BAP_UART.h"
#include "BAP_setup.h"
#include "BAP_define.h"

#include <stdio.h>
#include <errno.h>
#include <stddef.h>
#include <sys/types.h>

SemaphoreHandle_t CMDUART_Send_Se;

SemaphoreHandle_t CMDUART_Recv_Se;
char CMDUART_Recv_Buffer[BAP_MAX_UART_MESSAGE_LENGTH_D] = {0};

SemaphoreHandle_t CMDUART_RecvStartMess_Se;
char CMDUART_RecvStartMess_Buffer[BAP_UART_STARTMESSAGE_LENGTH_D] = {0};

TaskSharedVars_s SharedVars;

int main(void)
{
    BAP_SetupClock();
    BAP_SetupGPIO();

    BAP_SetupUSARTWithDMA(USART2, BAP_UART_BAUDRATE_D, 1);
    BAP_SetupUSARTWithDMA(UART5, BAP_UART_BAUDRATE_D, 0);

    BAP_SetupPWM(TIM1, BAP_SYSTEM_CLOCK_HZ_D/1000000, 1000);	//config Timer1 PWM to run at 1kHz
    BAP_SetupPWMOutputEnable(BAP_PWM_TIMER_D, BAP_PWM_MOTOR1_FORWARD_OUT_D);
    BAP_SetupPWMOutputEnable(BAP_PWM_TIMER_D, BAP_PWM_MOTOR1_BACKWARD_OUT_D);
    BAP_SetupPWMOutputEnable(BAP_PWM_TIMER_D, BAP_PWM_MOTOR2_FORWARD_OUT_D);
    BAP_SetupPWMOutputEnable(BAP_PWM_TIMER_D, BAP_PWM_MOTOR2_BACKWARD_OUT_D);

    SharedVars.PWM = 0;
    SharedVars.flag = 0;

    BAP_SetupSemaphoreInit();
    BAP_UART_InterSemaphoreInit();
    BAP_TaskInterSemaphoreInit();

    xTaskCreate(BAP_TaskRecvCmd, "USART2 Recv Command Handler", configMINIMAL_STACK_SIZE, (void*)&SharedVars, 1, NULL);
    xTaskCreate(BAP_TaskMotorControl, "BAP_TaskMotorControl", configMINIMAL_STACK_SIZE, (void*)&SharedVars, 1, NULL);
    vTaskStartScheduler();

    while(1);
    return 0;
}
