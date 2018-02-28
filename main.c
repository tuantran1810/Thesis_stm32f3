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

int main(void)
{
    BAPClockSetup();
    BAPGPIOSetup();
    BAPUSARTWithDMASetup(USART2, 115200, 1);
    BAPUSARTWithDMASetup(UART5, 115200, 0);
    BAPSemaphoreInit();
    BAPPWMSetup(TIM1, 64, 1000, TIM_OC1);
    xTaskCreate(USART2RecvCmdTask_Handler, "USART2 Recv Command Handler", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    vTaskStartScheduler();

    while(1);
    return 0;
}
