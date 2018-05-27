#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>

#include "BAP_define.h"
#include "BAP_UART.h"

SemaphoreHandle_t InterDEBUGUART_Send_Se;
SemaphoreHandle_t InterDEBUGUART_Recv_Se;
SemaphoreHandle_t InterCMDUART_Send_Se;
SemaphoreHandle_t InterCMDUART_Recv_Se;

BAP_RESULT_E BAP_UARTModuleInit(void)
{
    BAP_SemCreateBin(InterDEBUGUART_Send_Se);
    BAP_SemCreateBin(InterDEBUGUART_Recv_Se);
    BAP_SemCreateBin(InterCMDUART_Send_Se);
    BAP_SemCreateBin(InterCMDUART_Recv_Se);
    BAP_SemGive(InterDEBUGUART_Send_Se);
    BAP_SemGive(InterDEBUGUART_Recv_Se);
    BAP_SemGive(InterCMDUART_Send_Se);
    BAP_SemGive(InterCMDUART_Recv_Se);
    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_UART_SendString(uint32_t uart, char* str, int strlen)
{
    if (str == NULL)
        return BAP_FAILED_NULL_PTR;

    if (strlen <= 0)
        return BAP_FAILED_WRONG_PAR;

    if ((uart != USART1) && (uart != USART2) && (uart != USART3) && (uart != UART4) && (uart != UART5))
        return BAP_FAILED_WRONG_PAR;

    if (uart == BAP_UART_CMD_CH_D)
    {
        BAP_SemTakeMax(InterCMDUART_Send_Se);
    }
    else if (uart == BAP_UART_DEBUG_CH_D)
    {
        BAP_SemTakeMax(InterDEBUGUART_Send_Se);
    }

    for (int i = 0; i < strlen; i++)
    {
        usart_send_blocking(uart, str[i]);
    }

    if (uart == BAP_UART_CMD_CH_D)
    {
        BAP_SemGive(InterCMDUART_Send_Se);
    }
    else if (uart == BAP_UART_DEBUG_CH_D)
    {
        BAP_SemGive(InterDEBUGUART_Send_Se);
    }

    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_UART_RecvString(uint32_t uart, char* str, int strlen)
{
    if (str == NULL)
        return BAP_FAILED_NULL_PTR;

    if ((uart != USART1) && (uart != USART2) && (uart != USART3) && (uart != UART4) && (uart != UART5))
        return BAP_FAILED_WRONG_PAR;
    
    int count = 0;

    if (uart == BAP_UART_CMD_CH_D)
    {
        BAP_SemTakeMax(InterCMDUART_Recv_Se);
    }
    else if (uart == BAP_UART_DEBUG_CH_D)
    {
        BAP_SemTakeMax(InterDEBUGUART_Recv_Se);
    }

    for (int i=0; i<strlen; i++)
    {
        str[count] = usart_recv_blocking(uart);
        count++;
    }

    if (uart == BAP_UART_CMD_CH_D)
    {
        BAP_SemGive(InterCMDUART_Recv_Se);
    }
    else if (uart == BAP_UART_DEBUG_CH_D) 
    {
        BAP_SemGive(InterDEBUGUART_Recv_Se);
    }

    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_UARTSendDMA(uint32_t uart, char *data, int size)
{
    if ((uart != USART1) && (uart != USART2) && (uart != USART3) && (uart != UART4))
        return BAP_FAILED_WRONG_PAR;

    uint32_t DMANum = 0, PerAdd = 0;
    uint8_t DMACH = 0;

    switch (uart)
    {
        case USART1:
            DMANum = DMA1;
            DMACH = DMA_CHANNEL4;
            PerAdd = (uint32_t)&USART1_TDR;
            break;
        case USART2:
            DMANum = DMA1;
            DMACH = DMA_CHANNEL7;
            PerAdd = (uint32_t)&USART2_TDR;
            break;
        case USART3:
            DMANum = DMA1;
            DMACH = DMA_CHANNEL2;
            PerAdd = (uint32_t)&USART3_TDR;
            break;
        case UART4:
            DMANum = DMA2;
            DMACH = DMA_CHANNEL5;
            PerAdd = (uint32_t)&UART4_TDR;
            break;
        default:
            break;
    }

    usart_disable_tx_dma(uart);
    dma_disable_channel(DMANum, DMACH);
    dma_channel_reset(DMANum, DMACH);

    dma_set_peripheral_address(DMANum, DMACH, PerAdd);
    dma_set_memory_address(DMANum, DMACH, (uint32_t)data);
    dma_set_number_of_data(DMANum, DMACH, size);
    dma_set_read_from_memory(DMANum, DMACH);

    dma_enable_memory_increment_mode(DMANum, DMACH);
    dma_disable_peripheral_increment_mode(DMANum, DMACH);

    dma_set_peripheral_size(DMANum, DMACH, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMANum, DMACH, DMA_CCR_MSIZE_8BIT);

    dma_set_priority(DMANum, DMACH, DMA_CCR_PL_VERY_HIGH);

    dma_enable_transfer_complete_interrupt(DMANum, DMACH);

    dma_enable_channel(DMANum, DMACH);

    usart_enable_tx_dma(uart);
    return BAP_SUCCESS;
}

void dma1_channel7_isr(void)
{
    if ((DMA1_ISR & DMA_ISR_TCIF7) != 0) 
    {
        DMA1_IFCR |= DMA_IFCR_CTCIF7;
    }

    dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL7);

    usart_disable_tx_dma(USART2);

    dma_disable_channel(DMA1, DMA_CHANNEL7);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(CMDUART_Send_Se, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void dma1_channel2_isr(void)
{
    if ((DMA1_ISR & DMA_ISR_TCIF2) != 0) 
    {
        DMA1_IFCR |= DMA_IFCR_CTCIF2;
    }

    dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL2);

    usart_disable_tx_dma(USART3);

    dma_disable_channel(DMA1, DMA_CHANNEL2);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(DEBUGUART_Send_Se, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

BAP_RESULT_E BAP_UARTRecvDMA(uint32_t uart, char *data, int size)
{
    if ((uart != USART1) && (uart != USART2) && (uart != USART3) && (uart != UART4))
        return BAP_FAILED_WRONG_PAR;

    uint32_t DMANum = 0, PerAdd = 0;
    uint8_t DMACH = 0;
    switch (uart)
    {
        case USART1:
            DMANum = DMA1;
            DMACH = DMA_CHANNEL5;
            PerAdd = (uint32_t)&USART1_RDR;
            break;
        case USART2:
            DMANum = DMA1;
            DMACH = DMA_CHANNEL6;
            PerAdd = (uint32_t)&USART2_RDR;
            break;
        case USART3:
            DMANum = DMA1;
            DMACH = DMA_CHANNEL3;
            PerAdd = (uint32_t)&USART3_RDR;
            break;
        case UART4:
            DMANum = DMA2;
            DMACH = DMA_CHANNEL3;
            PerAdd = (uint32_t)&UART4_RDR;
            break;
        default:
            break;
    }

    usart_disable_rx_dma(uart);
    dma_disable_channel(DMANum, DMACH);
    dma_channel_reset(DMANum, DMACH);

    dma_set_peripheral_address(DMANum, DMACH, PerAdd);
    dma_set_memory_address(DMANum, DMACH, (uint32_t)data);
    dma_set_number_of_data(DMANum, DMACH, size);
    dma_set_read_from_peripheral(DMANum, DMACH);

    dma_enable_memory_increment_mode(DMANum, DMACH);
    dma_disable_peripheral_increment_mode(DMANum, DMACH);

    dma_set_peripheral_size(DMANum, DMACH, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMANum, DMACH, DMA_CCR_MSIZE_8BIT);

    dma_set_priority(DMANum, DMACH, DMA_CCR_PL_VERY_HIGH);

    dma_enable_transfer_complete_interrupt(DMANum, DMACH);

    dma_enable_channel(DMANum, DMACH);

    usart_enable_rx_dma(uart);
    return BAP_SUCCESS;
}

void dma1_channel6_isr(void)
{
    if ((DMA1_ISR & DMA_ISR_TCIF6) != 0)
    {
        DMA1_IFCR |= DMA_IFCR_CTCIF6;
    }
    
    dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL6);

    usart_disable_rx_dma(USART2);

    dma_disable_channel(DMA1, DMA_CHANNEL6);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(CMDUART_Recv_Se, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void dma1_channel3_isr(void)
{
    if ((DMA1_ISR & DMA_ISR_TCIF3) != 0)
    {
        DMA1_IFCR |= DMA_IFCR_CTCIF3;
    }
    
    dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL3);

    usart_disable_rx_dma(USART3);

    dma_disable_channel(DMA1, DMA_CHANNEL3);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(DEBUGUART_Recv_Se, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
