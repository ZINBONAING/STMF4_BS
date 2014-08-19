
#include <stdio.h>
#include <stm32f4xx.h>
#include <stm32f4xx_usart.h> // under Libraries/STM32F4xx_StdPeriph_Driver/inc and src
#include <stdio.h>
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include <stm32f4xx_i2c.h>
#include <MPU9150.h>
#include "stm32f4xx_rcc.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <misc.h>			 // I recommend you have a look at these in the ST firmware folder
#include <math.h>
#include "stm32f4xx_tim.h"
#include <stm32f4xx_it.h>
#include "uart.h"
#include "stm32f4xx_exti.h"




#include <stdio.h>

#include "uart.h"

#define USARTx                           USART3


/********************************* Globals ************************************/
/********************************* Prototypes *********************************/
/*******************************  Function ************************************/

void USART_Config(void)
{
	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 *
	 * They make our life easier because we don't have to mess around with
	 * the low level stuff of setting bits in the correct registers
	 */
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* enable APB2 peripheral clock for USART1
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	/* --------------------------- System Clocks Configuration -----------------*/
	  /* USART3 clock enable */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	  /* GPIOB clock enable */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* This sequence sets up the TX and RX pins
	GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Connect USART pins to AF */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	/* Now the USART_InitStruct is used to define the
	 * properties of USART1
	 */
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART3, &USART_InitStruct);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

        	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;		 // we want to configure the USART1 interrupts
        	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;// this sets the priority group of the USART1 interrupts
        	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		 // this sets the subpriority inside the group
        	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
        	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff
    USART_Cmd(USART3, ENABLE);


}

int fputc(int ch, FILE *f)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USARTx, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

int fputchar(int ch )
{
  //* Place your implementation of fputc here 
  //* e.g. write a character to the USART 
  USART_SendData(USARTx, (uint8_t) ch);

  //* Loop until the end of transmission 
  while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
  {}

  return ch;
}
