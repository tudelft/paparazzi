#ifndef CONFIG_BETH_H
#define CONFIG_BETH_H

#define AHB_CLK 72000000

/* this board uses a crystal for HSE */
//#define HSE_TYPE RCC_HSE_ON

/* Onboard LEDs */
#define USE_LED_1
#define LED_1_GPIO GPIOC
#define LED_1_GPIO_CLK RCC_APB2Periph_GPIOC
#define LED_1_GPIO_PIN GPIO_Pin_12

#define USE_LED_4
#define LED_4_GPIO GPIOA
#define LED_4_GPIO_CLK RCC_APB2Periph_GPIOA
#define LED_4_GPIO_PIN GPIO_Pin_6

#define USE_LED_5
#define LED_5_GPIO GPIOA
#define LED_5_GPIO_CLK RCC_APB2Periph_GPIOA
#define LED_5_GPIO_PIN GPIO_Pin_7

#define USE_LED_6
#define LED_6_GPIO GPIOB
#define LED_6_GPIO_CLK RCC_APB2Periph_GPIOB
#define LED_6_GPIO_PIN GPIO_Pin_0

#define USE_LED_7
#define LED_7_GPIO GPIOB
#define LED_7_GPIO_CLK RCC_APB2Periph_GPIOB
#define LED_7_GPIO_PIN GPIO_Pin_1

#endif /* CONFIG_BETH_H */
