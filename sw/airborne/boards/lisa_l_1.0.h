#ifndef CONFIG_LISA_L_1_0_H
#define CONFIG_LISA_L_1_0_H

#define BOARD_LISA_L

/* Lisa/L has an 8MHZ external clock and 72MHz internal. */
#define EXT_CLK 8000000
#define AHB_CLK 72000000

/* Onboard LEDs */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_STP08

// FIXME, this is just to make it compile
#define POWER_SWITCH_LED 5

/* SPI slave mapping */

#define SPI_SELECT_SLAVE0_PERIPH RCC_APB2ENR_IOPAEN
#define SPI_SELECT_SLAVE0_PORT GPIOA
#define SPI_SELECT_SLAVE0_PIN GPIO15

#define SPI_SELECT_SLAVE1_PERIPH RCC_APB2ENR_IOPAEN
#define SPI_SELECT_SLAVE1_PORT GPIOA
#define SPI_SELECT_SLAVE1_PIN GPIO4

#define SPI_SELECT_SLAVE2_PERIPH RCC_APB2ENR_IOPBEN
#define SPI_SELECT_SLAVE2_PORT GPIOB
#define SPI_SELECT_SLAVE2_PIN GPIO12

#define SPI_SELECT_SLAVE3_PERIPH RCC_APB2ENR_IOPCEN
#define SPI_SELECT_SLAVE3_PORT GPIOC
#define SPI_SELECT_SLAVE3_PIN GPIO13

#define SPI_SELECT_SLAVE4_PERIPH RCC_APB2ENR_IOPCEN
#define SPI_SELECT_SLAVE4_PORT GPIOC
#define SPI_SELECT_SLAVE4_PIN GPIO12


/*
 * I2C
 *
 */
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO6
#define I2C1_GPIO_SDA GPIO7

#define I2C2_GPIO_PORT GPIOB
#define I2C2_GPIO_SCL GPIO10
#define I2C2_GPIO_SDA GPIO11


/*
 * UART pin configuration
 *
 * sets on which pins the UARTs are connected
 */
#define UART1_GPIO_AF 0
#define UART1_GPIO_PORT_RX GPIO_BANK_USART1_RX
#define UART1_GPIO_RX GPIO_USART1_RX
#define UART1_GPIO_PORT_TX GPIO_BANK_USART1_TX
#define UART1_GPIO_TX GPIO_USART1_TX

#define UART2_GPIO_AF 0
#define UART2_GPIO_PORT_RX GPIO_BANK_USART2_RX
#define UART2_GPIO_RX GPIO_USART2_RX
#define UART2_GPIO_PORT_TX GPIO_BANK_USART2_TX
#define UART2_GPIO_TX GPIO_USART2_TX

#define UART3_GPIO_AF AFIO_MAPR_USART3_REMAP_PARTIAL_REMAP
#define UART3_GPIO_PORT_RX GPIO_BANK_USART3_PR_RX
#define UART3_GPIO_RX GPIO_USART3_PR_RX
#define UART3_GPIO_PORT_TX GPIO_BANK_USART3_PR_TX
#define UART3_GPIO_TX GPIO_USART3_PR_TX

#define UART5_GPIO_AF 0
#define UART5_GPIO_PORT_RX GPIO_BANK_UART5_RX
#define UART5_GPIO_RX GPIO_UART5_RX
#define UART5_GPIO_PORT_TX GPIO_BANK_UART5_TX
#define UART5_GPIO_TX GPIO_UART5_TX


/*
 * Spektrum
 */
/* The line that is pulled low at power up to initiate the bind process */
#define SPEKTRUM_BIND_PIN GPIO3
#define SPEKTRUM_BIND_PIN_PORT GPIOC

#define SPEKTRUM_UART1_RCC_REG &RCC_APB2ENR
#define SPEKTRUM_UART1_RCC_DEV RCC_APB2ENR_USART1EN
#define SPEKTRUM_UART1_BANK GPIO_BANK_USART1_RX
#define SPEKTRUM_UART1_PIN GPIO_USART1_RX
#define SPEKTRUM_UART1_AF 0
#define SPEKTRUM_UART1_IRQ NVIC_USART1_IRQ
#define SPEKTRUM_UART1_ISR usart1_isr
#define SPEKTRUM_UART1_DEV USART1

#define SPEKTRUM_UART3_RCC_REG &RCC_APB1ENR
#define SPEKTRUM_UART3_RCC_DEV RCC_APB1ENR_USART3EN
#define SPEKTRUM_UART3_BANK GPIO_BANK_USART3_PR_RX
#define SPEKTRUM_UART3_PIN GPIO_USART3_PR_RX
#define SPEKTRUM_UART3_AF AFIO_MAPR_USART3_REMAP_PARTIAL_REMAP
#define SPEKTRUM_UART3_IRQ NVIC_USART3_IRQ
#define SPEKTRUM_UART3_ISR usart3_isr
#define SPEKTRUM_UART3_DEV USART3

#define SPEKTRUM_UART5_RCC_REG &RCC_APB1ENR
#define SPEKTRUM_UART5_RCC_DEV RCC_APB1ENR_UART5EN
#define SPEKTRUM_UART5_BANK GPIO_BANK_UART5_RX
#define SPEKTRUM_UART5_PIN GPIO_UART5_RX
#define SPEKTRUM_UART5_AF 0
#define SPEKTRUM_UART5_IRQ NVIC_UART5_IRQ
#define SPEKTRUM_UART5_ISR uart5_isr
#define SPEKTRUM_UART5_DEV UART5


/*
 * PPM input
 */
#define USE_PPM_TIM2 1
#define PPM_CHANNEL         TIM_IC2
#define PPM_TIMER_INPUT     TIM_IC_IN_TI2
#define PPM_IRQ             NVIC_TIM2_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC2IE
#define PPM_CC_IF           TIM_SR_CC2IF
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO1
#define PPM_GPIO_AF         0


/* ADC */
// active ADC
#define USE_AD1 1
#define USE_AD1_1 1
#define USE_AD1_2 1
#define USE_AD1_3 1
#define USE_AD1_4 1

#define USE_AD_TIM1 1

/* PA0 - ADC0 */
/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY 2
#endif
#define DefaultVoltageOfAdc(adc) (0.0059*adc)
/* Onboard ADCs */
#define BOARD_ADC_CHANNEL_1 8
#define BOARD_ADC_CHANNEL_2 9
// FIXME - removed for now and used for battery monitoring
//#define BOARD_ADC_CHANNEL_3 13
#define BOARD_ADC_CHANNEL_3 0
#define BOARD_ADC_CHANNEL_4 15

#define USE_BARO_BOARD 1


/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

/*
 * PWM
 *
 */
#define PWM_USE_TIM3 1
#define PWM_USE_TIM4 1

#define USE_PWM1 1
#define USE_PWM2 1
#define USE_PWM3 1
#define USE_PWM4 1
#define USE_PWM5 1
#define USE_PWM6 1

#define ACTUATORS_PWM_NB 6

// PWM_SERVO_x is the index of the servo in the actuators_pwm_values array
#if USE_PWM1
#define PWM_SERVO_1 0
#define PWM_SERVO_1_TIMER TIM3
#define PWM_SERVO_1_RCC_IOP RCC_APB2ENR_IOPCEN
#define PWM_SERVO_1_GPIO GPIOC
#define PWM_SERVO_1_PIN GPIO6
#define PWM_SERVO_1_AF 0
#define PWM_SERVO_1_OC TIM_OC1
#define PWM_SERVO_1_OC_BIT (1<<0)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#if USE_PWM2
#define PWM_SERVO_2 1
#define PWM_SERVO_2_TIMER TIM3
#define PWM_SERVO_2_RCC_IOP RCC_APB2ENR_IOPCEN
#define PWM_SERVO_2_GPIO GPIOC
#define PWM_SERVO_2_PIN GPIO7
#define PWM_SERVO_2_AF 0
#define PWM_SERVO_2_OC TIM_OC2
#define PWM_SERVO_2_OC_BIT (1<<1)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#if USE_PWM3
#define PWM_SERVO_3 2
#define PWM_SERVO_3_TIMER TIM3
#define PWM_SERVO_3_RCC_IOP RCC_APB2ENR_IOPCEN
#define PWM_SERVO_3_GPIO GPIOC
#define PWM_SERVO_3_PIN GPIO8
#define PWM_SERVO_3_AF 0
#define PWM_SERVO_3_OC TIM_OC3
#define PWM_SERVO_3_OC_BIT (1<<2)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

#if USE_PWM4
#define PWM_SERVO_4 3
#define PWM_SERVO_4_TIMER TIM3
#define PWM_SERVO_4_RCC_IOP RCC_APB2ENR_IOPCEN
#define PWM_SERVO_4_GPIO GPIOC
#define PWM_SERVO_4_PIN GPIO9
#define PWM_SERVO_4_AF 0
#define PWM_SERVO_4_OC TIM_OC4
#define PWM_SERVO_4_OC_BIT (1<<3)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

#if USE_PWM5
#define PWM_SERVO_5 4
#define PWM_SERVO_5_TIMER TIM4
#define PWM_SERVO_5_RCC_IOP RCC_APB2ENR_IOPBEN
#define PWM_SERVO_5_GPIO GPIOB
#define PWM_SERVO_5_PIN GPIO8
#define PWM_SERVO_5_AF 0
#define PWM_SERVO_5_OC TIM_OC3
#define PWM_SERVO_5_OC_BIT (1<<2)
#else
#define PWM_SERVO_5_OC_BIT 0
#endif

#if USE_PWM6
#define PWM_SERVO_6 5
#define PWM_SERVO_6_TIMER TIM4
#define PWM_SERVO_6_RCC_IOP RCC_APB2ENR_IOPBEN
#define PWM_SERVO_6_GPIO GPIOB
#define PWM_SERVO_6_PIN GPIO9
#define PWM_SERVO_6_AF 0
#define PWM_SERVO_6_OC TIM_OC4
#define PWM_SERVO_6_OC_BIT (1<<4)
#else
#define PWM_SERVO_6_OC_BIT 0
#endif

/* servos 1-4 on TIM3 */
#define PWM_TIM3_CHAN_MASK (PWM_SERVO_1_OC_BIT|PWM_SERVO_2_OC_BIT|PWM_SERVO_3_OC_BIT|PWM_SERVO_4_OC_BIT)
/* servos 5-6 on TIM4 */
#define PWM_TIM4_CHAN_MASK (PWM_SERVO_5_OC_BIT|PWM_SERVO_6_OC_BIT)

#endif /* CONFIG_LISA_L_1_0_H */
