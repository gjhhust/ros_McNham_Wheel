#ifndef _LED_TASK_H_
#define _LED_TASK_H_

#define RCC_GPIO RCC_APB2Periph_GPIOC
#define LED_PIN1 GPIO_Pin_13
#define LED_PIN2 GPIO_Pin_14
#define LED_GPIO GPIOC
void LED_Configuration(void);
void LedTask(void);
#endif
