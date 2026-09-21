/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef MCUCONF_H
#define MCUCONF_H

/*
 * STM32F4xx drivers configuration.
 * The following settings override the default settings present in
 * the various device driver implementation headers.
 * Note that the settings for each driver only have effect if the whole
 * driver is enabled in halconf.h.
 *
 * IRQ priorities:
 * 15...0       Lowest...Highest.
 *
 * DMA priorities:
 * 0...3        Lowest...Highest.
 */

#define STM32F4xx_MCUCONF
#define STM32F405_MCUCONF
#define STM32F415_MCUCONF
#define STM32F407_MCUCONF
#define STM32F417_MCUCONF

/*
 * HAL driver system settings.
 */
#define STM32_NO_INIT                       FALSE
#define STM32_PVD_ENABLE                    FALSE
#define STM32_PLS                           STM32_PLS_LEV0
#define STM32_BKPRAM_ENABLE                 FALSE
#define STM32_HSI_ENABLED                   TRUE
#ifndef STM32_LSI_ENABLED
#define STM32_LSI_ENABLED                   FALSE
#endif
#define STM32_HSE_ENABLED                   TRUE
#define STM32_LSE_ENABLED                   FALSE
#define STM32_CLOCK48_REQUIRED              TRUE
#define STM32_SW                            STM32_SW_PLL
#define STM32_PLLSRC                        STM32_PLLSRC_HSE

/* Matek F405-TE-SD HSE is 8MHz */
#define STM32_PLLM_VALUE                    8
#define STM32_PLLN_VALUE                    336
#define STM32_PLLP_VALUE                    2
#define STM32_PLLQ_VALUE                    7

#define STM32_HPRE                          STM32_HPRE_DIV1
#define STM32_PPRE1                         STM32_PPRE1_DIV4
#define STM32_PPRE2                         STM32_PPRE2_DIV2
#define STM32_RTCSEL                        STM32_RTCSEL_LSI
#define STM32_RTCPRE_VALUE                  8
#define STM32_MCO1SEL                       STM32_MCO1SEL_HSI
#define STM32_MCO1PRE                       STM32_MCO1PRE_DIV1
#define STM32_MCO2SEL                       STM32_MCO2SEL_SYSCLK
#define STM32_MCO2PRE                       STM32_MCO2PRE_DIV5
#define STM32_I2SSRC                        STM32_I2SSRC_CKIN
#define STM32_PLLI2SN_VALUE                 192
#define STM32_PLLI2SR_VALUE                 5

/*
 * IRQ system settings.
 */
#define STM32_IRQ_EXTI0_PRIORITY            6
#define STM32_IRQ_EXTI1_PRIORITY            6
#define STM32_IRQ_EXTI2_PRIORITY            6
#define STM32_IRQ_EXTI3_PRIORITY            6
#define STM32_IRQ_EXTI4_PRIORITY            6
#define STM32_IRQ_EXTI5_9_PRIORITY          6
#define STM32_IRQ_EXTI10_15_PRIORITY        6
#define STM32_IRQ_EXTI16_PRIORITY           6
#define STM32_IRQ_EXTI17_PRIORITY           15
#define STM32_IRQ_EXTI18_PRIORITY           6
#define STM32_IRQ_EXTI19_PRIORITY           6
#define STM32_IRQ_EXTI20_PRIORITY           6
#define STM32_IRQ_EXTI21_PRIORITY           15
#define STM32_IRQ_EXTI22_PRIORITY           15

#define STM32_IRQ_TIM1_BRK_TIM9_PRIORITY    7
#define STM32_IRQ_TIM1_UP_TIM10_PRIORITY    7
#define STM32_IRQ_TIM1_TRGCO_TIM11_PRIORITY 7
#define STM32_IRQ_TIM1_CC_PRIORITY          7
#define STM32_IRQ_TIM2_PRIORITY             7
#define STM32_IRQ_TIM3_PRIORITY             7
#define STM32_IRQ_TIM4_PRIORITY             7
#define STM32_IRQ_TIM5_PRIORITY             7
#define STM32_IRQ_TIM6_PRIORITY             7
#define STM32_IRQ_TIM7_PRIORITY             7
#define STM32_IRQ_TIM8_BRK_TIM12_PRIORITY   7
#define STM32_IRQ_TIM8_UP_TIM13_PRIORITY    7
#define STM32_IRQ_TIM8_TRGCO_TIM14_PRIORITY 7
#define STM32_IRQ_TIM8_CC_PRIORITY          7

#define STM32_IRQ_USART1_PRIORITY           12
#define STM32_IRQ_USART2_PRIORITY           12
#define STM32_IRQ_USART3_PRIORITY           12
#define STM32_IRQ_UART4_PRIORITY            12
#define STM32_IRQ_UART5_PRIORITY            12
#define STM32_IRQ_USART6_PRIORITY           12

/*
 * ADC driver system settings.
 */
#define STM32_ADC_ADCPRE                    ADC_CCR_ADCPRE_DIV8
#define STM32_ADC_USE_ADC1                  TRUE
#define STM32_ADC_USE_ADC2                  FALSE
#define STM32_ADC_USE_ADC3                  FALSE
#define STM32_ADC_ADC1_DMA_STREAM           STM32_DMA_STREAM_ID(2, 4)
#define STM32_ADC_ADC2_DMA_STREAM           STM32_DMA_STREAM_ID(2, 2)
#define STM32_ADC_ADC3_DMA_STREAM           STM32_DMA_STREAM_ID(2, 1)
#define STM32_ADC_ADC1_DMA_PRIORITY         2
#define STM32_ADC_ADC2_DMA_PRIORITY         2
#define STM32_ADC_ADC3_DMA_PRIORITY         2
#define STM32_ADC_IRQ_PRIORITY              6
#define STM32_ADC_ADC1_DMA_IRQ_PRIORITY     6
#define STM32_ADC_ADC2_DMA_IRQ_PRIORITY     6
#define STM32_ADC_ADC3_DMA_IRQ_PRIORITY     6

/*
 * GPT driver system settings.
 */
#define STM32_GPT_USE_TIM1                  FALSE
#define STM32_GPT_USE_TIM2                  FALSE
#define STM32_GPT_USE_TIM3                  FALSE
#define STM32_GPT_USE_TIM4                  FALSE
#define STM32_GPT_USE_TIM5                  FALSE
#define STM32_GPT_USE_TIM6                  FALSE
#define STM32_GPT_USE_TIM7                  FALSE
#define STM32_GPT_USE_TIM8                  FALSE
#define STM32_GPT_USE_TIM9                  FALSE
#define STM32_GPT_USE_TIM11                 FALSE
#define STM32_GPT_USE_TIM12                 FALSE
#define STM32_GPT_USE_TIM14                 FALSE

/*
 * I2C driver system settings.
 */
#if USE_I2C1
#define STM32_I2C_USE_I2C1                  TRUE
#else
#define STM32_I2C_USE_I2C1                  FALSE
#endif
#define STM32_I2C_USE_I2C2                  FALSE
#define STM32_I2C_USE_I2C3                  FALSE

#define STM32_I2C_ISR_LIMIT                 6
#define STM32_I2C_BUSY_TIMEOUT              0
#define STM32_I2C_I2C1_RX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 0)
#define STM32_I2C_I2C1_TX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 6)
#define STM32_I2C_I2C1_IRQ_PRIORITY         5
#define STM32_I2C_I2C1_DMA_PRIORITY         3
#define STM32_I2C_DMA_ERROR_HOOK(i2cp)      osalSysHalt("DMA failure")


/*
 * ICU driver system settings.
 */
#define STM32_ICU_USE_TIM1                  FALSE
#define STM32_ICU_USE_TIM2                  FALSE
#define STM32_ICU_USE_TIM3                  FALSE
#define STM32_ICU_USE_TIM4                  FALSE
#define STM32_ICU_USE_TIM5                  FALSE
#define STM32_ICU_USE_TIM8                  FALSE
#define STM32_ICU_USE_TIM9                  FALSE
#define STM32_ICU_TIM5_IRQ_PRIORITY         7

/*
 * PWM driver system settings.
 */
#define STM32_PWM_USE_ADVANCED              FALSE

#ifndef STM32_PWM_USE_TIM1
#define STM32_PWM_USE_TIM1                  TRUE
#endif
#ifndef STM32_PWM_USE_TIM2
#define STM32_PWM_USE_TIM2                  TRUE
#endif
#ifndef STM32_PWM_USE_TIM3
#define STM32_PWM_USE_TIM3                  TRUE
#endif
#ifndef STM32_PWM_USE_TIM4
#define STM32_PWM_USE_TIM4                  TRUE
#endif
#define STM32_PWM_USE_TIM5                  FALSE
#ifndef STM32_PWM_USE_TIM8
#define STM32_PWM_USE_TIM8                  TRUE
#endif
#define STM32_PWM_USE_TIM9                  FALSE

#ifndef STM32_PWM_USE_TIM12
#define STM32_PWM_USE_TIM12                  TRUE
#endif
#ifndef STM32_PWM_USE_TIM13
#define STM32_PWM_USE_TIM13                  TRUE
#endif

#define STM32_PWM_TIM1_IRQ_PRIORITY         7
#define STM32_PWM_TIM2_IRQ_PRIORITY         7
#define STM32_PWM_TIM3_IRQ_PRIORITY         7
#define STM32_PWM_TIM4_IRQ_PRIORITY         7
#define STM32_PWM_TIM8_IRQ_PRIORITY         7

/*
 * SERIAL driver system settings.
 */
#if USE_UART1
#define STM32_SERIAL_USE_USART1             TRUE
#else
#define STM32_SERIAL_USE_USART1             FALSE
#endif
#if USE_UART2
#define STM32_SERIAL_USE_USART2             TRUE
#else
#define STM32_SERIAL_USE_USART2             FALSE
#endif
#if USE_UART3
#define STM32_SERIAL_USE_USART3             TRUE
#else
#define STM32_SERIAL_USE_USART3             FALSE
#endif
#if USE_UART4
#define STM32_SERIAL_USE_UART4              TRUE
#else
#define STM32_SERIAL_USE_UART4              FALSE
#endif
#if USE_UART5
#define STM32_SERIAL_USE_UART5              TRUE
#else
#define STM32_SERIAL_USE_UART5              FALSE
#endif
#if USE_UART6
#define STM32_SERIAL_USE_USART6             TRUE
#else
#define STM32_SERIAL_USE_USART6             FALSE
#endif

#define STM32_SERIAL_USART1_PRIORITY        12
#define STM32_SERIAL_USART2_PRIORITY        12
#define STM32_SERIAL_USART3_PRIORITY        12
#define STM32_SERIAL_UART4_PRIORITY         12
#define STM32_SERIAL_UART5_PRIORITY         12
#define STM32_SERIAL_USART6_PRIORITY        12

/*
 * SPI driver system settings.
 */
#if USE_SPI1
#define STM32_SPI_USE_SPI1                  TRUE
#else
#define STM32_SPI_USE_SPI1                  FALSE
#endif
#if USE_SPI2
#define STM32_SPI_USE_SPI2                  TRUE
#else
#define STM32_SPI_USE_SPI2                  FALSE
#endif
#if USE_SPI3
#define STM32_SPI_USE_SPI3                  TRUE
#else
#define STM32_SPI_USE_SPI3                  FALSE
#endif

#define STM32_SPI_SPI1_RX_DMA_STREAM        STM32_DMA_STREAM_ID(2, 0)
#define STM32_SPI_SPI1_TX_DMA_STREAM        STM32_DMA_STREAM_ID(2, 5)
#define STM32_SPI_SPI2_RX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 3)
#define STM32_SPI_SPI2_TX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 4)
#define STM32_SPI_SPI3_RX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 2)
#define STM32_SPI_SPI3_TX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 7)

#define STM32_SPI_SPI1_DMA_PRIORITY         1
#define STM32_SPI_SPI2_DMA_PRIORITY         1
#define STM32_SPI_SPI3_DMA_PRIORITY         1
#define STM32_SPI_SPI1_IRQ_PRIORITY         10
#define STM32_SPI_SPI2_IRQ_PRIORITY         10
#define STM32_SPI_SPI3_IRQ_PRIORITY         10
#define STM32_SPI_DMA_ERROR_HOOK(spip)      osalSysHalt("DMA failure")

/*
 * ST driver system settings.
 */
#define STM32_ST_IRQ_PRIORITY               8
#define STM32_ST_USE_TIMER                  5

/*
 * USB driver system settings.
 */
#define STM32_USB_USE_OTG1                  TRUE   // FS, DFU_BOOT
#define STM32_USB_USE_OTG2                  FALSE  // HS
#define STM32_USB_OTG1_IRQ_PRIORITY         14
#define STM32_USB_OTG2_IRQ_PRIORITY         14
#define STM32_USB_OTG1_RX_FIFO_SIZE         512
#define STM32_USB_OTG2_RX_FIFO_SIZE         512
#define STM32_USB_HOST_WAKEUP_DURATION      2

#endif /* _MCUCONF_H_ */
