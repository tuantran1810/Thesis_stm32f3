#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/f3/nvic.h>
#include <libopencm3/stm32/timer.h>

#include "BAP_setup.h"
#include "BAP_define.h"

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void BAP_SetupClock(void)
{
    rcc_clock_setup_hsi(&rcc_hsi_8mhz[RCC_CLOCK_64MHZ]);
    rcc_periph_clock_enable(RCC_GPIOE); // for PWM
    rcc_periph_clock_enable(RCC_GPIOA); // for USART2
    rcc_periph_clock_enable(RCC_GPIOC); // for UART5
    rcc_periph_clock_enable(RCC_GPIOD); // for UART5

    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_UART5);
    rcc_periph_clock_enable(RCC_DMA1);
    rcc_periph_clock_enable(RCC_TIM1);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void BAP_SetupGPIO(void)
{
    //GPIO for USART2
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3); //PA2 = USART2_TX  PA3 = USART2_RX
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);

    //GPIO for UART5
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12);
    gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
    gpio_set_af(GPIOC, GPIO_AF5, GPIO12);//PC12 = UART5_TX
    gpio_set_af(GPIOD, GPIO_AF5, GPIO2);//PD2 = UART5_RX

    //GPIO for PWM => use timer 1 | PE9 = TIM1_CH1 | PE11 = TIM1_CH2 | PE13 = TIM1_CH3 | PE14 = TIM1_CH4
    gpio_mode_setup(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO11 | GPIO13 | GPIO14);
    gpio_set_af(GPIOE, GPIO_AF2, GPIO9 | GPIO11 | GPIO13 | GPIO14);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//Run BAP_SetupPWM() first, then run BAP_SetupPWMOutputEnable() to enable output pins
//Use BAP_SetupPWMChangePeriod() to change PWM period after running two functions above
BAP_RESULT_E BAP_SetupPWMTimer(uint32_t tim, uint32_t prescaler, uint32_t period)
{
    // tim should be TIM1 - TIM17
    // prescaler should be uint16_t
    timer_reset(tim);
    timer_set_mode(tim, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(tim, prescaler);
    timer_set_repetition_counter(tim, 0);
    timer_enable_preload(tim);
    timer_continuous_mode(tim);
    timer_set_period(tim, period);
    timer_enable_break_main_output(tim);
    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_SetupPWMOutput(uint32_t tim)
{
    for (int i = 0; i < 7; i++)
    {
        timer_disable_oc_output(tim, i);
        timer_set_oc_mode(tim, i, TIM_OCM_PWM1);
        timer_set_oc_value(tim, i, 0);
    }
    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_SetupPWM(uint32_t tim, uint32_t prescaler, uint32_t period)
{
    BAP_SetupPWMTimer(tim, prescaler, period);
    BAP_SetupPWMOutput(tim);
    timer_enable_counter(tim);
    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_SetupPWMOutputEnable(uint32_t tim, enum tim_oc_id oc_id)
{
    timer_set_oc_value(tim, oc_id, 0);
    timer_enable_oc_output(tim, oc_id);
    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_SetupPWMChangePeriod(uint32_t tim, enum tim_oc_id oc_id, int period)
{
    timer_set_oc_value(tim, oc_id, period);
    return BAP_SUCCESS;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
BAP_RESULT_E BAP_SetupUSARTWithDMA(uint32_t uart, int baudrate,bool withDMA)
{
    if ((uart != USART1) && (uart != USART2) && (uart != USART3) && (uart != UART4) && (uart != UART5))
        return BAP_FAILED_WRONG_PAR;

    if ((uart == UART5) && withDMA)
        return BAP_FAILED_WRONG_PAR;

	usart_set_baudrate(uart, baudrate);
	usart_set_databits(uart, 8);
	usart_set_stopbits(uart, USART_STOPBITS_1);
	usart_set_mode(uart, USART_MODE_TX_RX);
	usart_set_parity(uart, USART_PARITY_NONE);
	usart_set_flow_control(uart, USART_FLOWCONTROL_NONE);

	usart_enable(uart);

    uint8_t DMARxCH = 0, DMATxCH = 0;
    if (withDMA)
    {
        switch (uart)
        {
            case USART1:
                DMARxCH = NVIC_DMA1_CHANNEL5_IRQ;
                DMATxCH = NVIC_DMA1_CHANNEL4_IRQ;
                break;
            case USART2:
                DMARxCH = NVIC_DMA1_CHANNEL6_IRQ;
                DMATxCH = NVIC_DMA1_CHANNEL7_IRQ;
                break;
            case USART3:
                DMARxCH = NVIC_DMA1_CHANNEL3_IRQ;
                DMATxCH = NVIC_DMA1_CHANNEL2_IRQ;
                break;
            case UART4:
                DMARxCH = NVIC_DMA2_CHANNEL3_IRQ;
                DMATxCH = NVIC_DMA2_CHANNEL5_IRQ;
                break;
        }
        nvic_set_priority(DMARxCH, 0xE0);
        nvic_enable_irq(DMARxCH);

        nvic_set_priority(DMATxCH, 0xD0);
        nvic_enable_irq(DMATxCH);
    }
    return BAP_SUCCESS;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void BAP_SetupSemaphoreInit(void)
{
    BAP_SemCreateBin(CMDUART_Send_Se);
    BAP_SemCreateBin(CMDUART_Recv_Se);
    BAP_SemCreateBin(CMDUART_RecvStartMess_Se);

    BAP_SemGive(CMDUART_RecvStartMess_Se);
}
