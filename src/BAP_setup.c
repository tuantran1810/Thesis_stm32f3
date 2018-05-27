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

BAP_RESULT_E BAP_SetupPWMTimer(uint32_t tim, uint32_t prescaler, uint32_t period);
BAP_RESULT_E BAP_SetupPWMOutput(uint32_t tim);
BAP_RESULT_E BAP_SetupEncoderInput(uint32_t tim);
BAP_RESULT_E BAP_SetupEncoderTimer(uint32_t tim);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void BAP_SetupClock(void)
{
    rcc_clock_setup_hsi(&rcc_hsi_8mhz[RCC_CLOCK_64MHZ]);
    rcc_periph_clock_enable(RCC_GPIOE); // for PWM
    rcc_periph_clock_enable(RCC_GPIOA); // for USART2
    rcc_periph_clock_enable(RCC_GPIOC); // for UART5, TIM8 Encoder input
    rcc_periph_clock_enable(RCC_GPIOD); // for UART5
    rcc_periph_clock_enable(RCC_GPIOB); // for USART3

    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_USART3);
    rcc_periph_clock_enable(RCC_DMA1);
    rcc_periph_clock_enable(RCC_TIM1);
    rcc_periph_clock_enable(RCC_TIM8);
    rcc_periph_clock_enable(RCC_TIM4);
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

    //GPIO for UART4
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10 | GPIO11);
    gpio_set_af(GPIOC, GPIO_AF5, GPIO10 | GPIO11); //PC10 = UART4_TX    PC11 = UART4_RX

    //GPIO for USART3
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10 | GPIO11);
    gpio_set_af(GPIOB, GPIO_AF7, GPIO10 | GPIO11); //PB10 = USART3_TX    PB11 = USART3_RX

    //GPIO for PWM => use timer 4 | PD12 = TIM4_CH1 (in 1 -> red wire motor) | PD13 = TIM4_CH2 (in 2 -> black wire motor) 
    //                            | PD14 = TIM4_CH3 (in 3 -> red wire motor) | PD15 = TIM4_CH4 (in 4 -> black wire motor)
    gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12 | GPIO13 | GPIO14 | GPIO15);
    gpio_set_af(GPIOD, GPIO_AF2, GPIO12 | GPIO13 | GPIO14 | GPIO15);

    //GPIO for TIM1 Encoder input for x axist PE9 = TIM1_CH1 (green wire)| PE11 = TIM1_CH2 (blue wire)
    gpio_mode_setup(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO11);
    gpio_set_output_options(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO9 | GPIO11);
    gpio_set_af(GPIOE, GPIO_AF2, GPIO9 | GPIO11);

    //GPIO for TIM8 Encoder input for y axist PC6 = TIM8_CH1 (green wire)| PC7 = TIM8_CH2 (blue wire)
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO6 | GPIO7);
    gpio_set_af(GPIOC, GPIO_AF4, GPIO6 | GPIO7);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
BAP_RESULT_E BAP_SetupEncoderTimer(uint32_t tim)
{
    timer_reset(tim);
    timer_set_mode(tim, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(tim, 0);
    timer_set_repetition_counter(tim, 0);
    timer_set_counter(tim, 0);
    timer_enable_preload(tim);
    timer_continuous_mode(tim);
    timer_set_period(tim, 0xFFFF);
    timer_enable_break_main_output(tim);
    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_SetupEncoderInput(uint32_t tim)
{
    timer_slave_set_mode(tim, TIM_SMCR_SMS_EM3);
    timer_ic_set_input(tim, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_input(tim, TIM_IC2, TIM_IC_IN_TI2);
    timer_set_oc_polarity_high(tim, TIM_OC1);
    timer_set_oc_polarity_high(tim, TIM_OC2);
    timer_enable_counter(tim);
    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_SetupEncoder(uint32_t tim)
{
    BAP_SetupEncoderTimer(tim);
    BAP_SetupEncoderInput(tim);
    return BAP_SUCCESS;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//Run BAP_SetupPWM() first, then run BAP_SetupPWMOutputEnable() to enable output pins
//Use BAP_MotorChangePWMPeriod() to change PWM period after running two functions above
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
    timer_set_period(tim, period-1);
    timer_enable_break_main_output(tim);
    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_SetupPWMOutput(uint32_t tim)
{
    const int NumOfOutput = 7;
    for (int i = 0; i < NumOfOutput; i++)
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
void BAP_SetupModuleInit(void)
{
    BAP_SemCreateBin(CMDUART_Send_Se);
    BAP_SemCreateBin(DEBUGUART_Send_Se);
    BAP_SemCreateBin(CMDUART_Recv_Se);
    BAP_SemCreateBin(DEBUGUART_Recv_Se);

    BAP_SetupClock();
    BAP_SetupGPIO();

    BAP_SetupUSARTWithDMA(BAP_UART_CMD_CH_D, BAP_UART_BAUDRATE_D, 1);
    // BAP_SetupUSARTWithDMA(BAP_UART_DEBUG_CH_D, BAP_UART_BAUDRATE_D, 0);
    BAP_SetupUSARTWithDMA(BAP_UART_DEBUG_CH_D, BAP_UART_BAUDRATE_D, 1);

    //config Timer1 PWM to run at 1kHz, resolution 1000 (0 to 999)
    BAP_SetupPWM(BAP_PWM_TIMER_D, BAP_SYSTEM_CLOCK_HZ_D/1000000, 1000); 
    BAP_SetupPWMOutputEnable(BAP_PWM_TIMER_D, BAP_PWM_XAXISTMOTOR_FORWARD_OUT_D);
    BAP_SetupPWMOutputEnable(BAP_PWM_TIMER_D, BAP_PWM_XAXISTMOTOR_BACKWARD_OUT_D);
    BAP_SetupPWMOutputEnable(BAP_PWM_TIMER_D, BAP_PWM_YAXISTMOTOR_FORWARD_OUT_D);
    BAP_SetupPWMOutputEnable(BAP_PWM_TIMER_D, BAP_PWM_YAXISTMOTOR_BACKWARD_OUT_D);

    BAP_SetupEncoder(BAP_XAXISTMOTOR_ENCODER_TIMER_D);
    BAP_SetupEncoder(BAP_YAXISTMOTOR_ENCODER_TIMER_D);
}
