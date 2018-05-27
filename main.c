#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <stddef.h>
#include <sys/types.h>

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
#include "BAP_motor.h"

SemaphoreHandle_t CMDUART_Send_Se;
SemaphoreHandle_t DEBUGUART_Send_Se;
SemaphoreHandle_t CMDUART_Recv_Se;
SemaphoreHandle_t DEBUGUART_Recv_Se;
TaskSharedVars_S SharedVars;

int main(void)
{
    BAP_SetupModuleInit();
    BAP_UARTModuleInit();
    BAP_MotorModuleInit();
    BAP_TaskModuleInit();

    BAP_LOG_DEBUG("System Start!!!\n\r");

    memset(&SharedVars, 0, sizeof(TaskSharedVars_S));

    xTaskCreate(BAP_TaskRecvCmd, "BAP_TaskRecvCmd", 300, (void*)&SharedVars, 3, NULL);
    xTaskCreate(BAP_TaskPlateControl, "BAP_TaskPlateControl", 500, (void*)&SharedVars, 3, NULL);
    xTaskCreate(BAP_TaskMotorControl, "BAP_TaskMotorControl", 300, (void*)&SharedVars, 2, NULL);
    xTaskCreate(BAP_TaskCommunicate, "BAP_TaskCommunicate", 300, (void*)&SharedVars, 1, NULL);
    xTaskCreate(BAP_TaskTrajectoryControl, "BAP_TaskTrajectoryControl", 300, (void*)&SharedVars, 1, NULL);
    vTaskStartScheduler();

    while(1);
    return 0;
}
