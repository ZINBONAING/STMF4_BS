/*******************************************************************************
File    : i2c.c
Purpose : I2c 3 to communicate with the sensors
Author  : 
********************************** Includes ***********************************/
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
/********************************* Defines ************************************/
#define I2Cx_FLAG_TIMEOUT             ((uint32_t)0x1000)
#define I2Cx_LONG_TIMEOUT             ((uint32_t)(300 * I2Cx_FLAG_TIMEOUT))

#define SENSORS_I2C_SCL_GPIO_PORT         GPIOB
#define SENSORS_I2C_SCL_GPIO_CLK          RCC_AHB1Periph_GPIOB

#define SENSORS_I2C_SCL_GPIO_PIN          GPIO_Pin_6
#define SENSORS_I2C_SCL_GPIO_PINSOURCE    GPIO_PinSource6
 
#define SENSORS_I2C_SDA_GPIO_PORT         GPIOB
#define SENSORS_I2C_SDA_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define SENSORS_I2C_SDA_GPIO_PIN          GPIO_Pin_9
#define SENSORS_I2C_SDA_GPIO_PINSOURCE    GPIO_PinSource9

#define SENSORS_I2C_RCC_CLK               RCC_APB1Periph_I2C1
#define SENSORS_I2C_AF                    GPIO_AF_I2C1


#define WAIT_FOR_FLAG(flag, value, timeout, errorcode)  I2CTimeout = timeout;\
          while(I2C_GetFlagStatus(I2C1, flag) != value) {\
            if((I2CTimeout--) == 0) return I2Cx_TIMEOUT_UserCallback(errorcode); \
          }\
  
#define CLEAR_ADDR_BIT      I2C_ReadRegister(I2C1, I2C_Register_SR1);\
                            I2C_ReadRegister(I2C1, I2C_Register_SR2);\
                               
/********************************* Globals ************************************/

/********************************* Prototypes *********************************/

/*******************************  Function ************************************/

void I2cMaster_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef I2C_InitStructure;

    /* Enable I2Cx clock */
  RCC_APB1PeriphClockCmd(SENSORS_I2C_RCC_CLK, ENABLE);

  /* Enable I2C GPIO clock */
  RCC_AHB1PeriphClockCmd(SENSORS_I2C_SCL_GPIO_CLK | SENSORS_I2C_SDA_GPIO_CLK, ENABLE);

  /* Configure I2Cx pin: SCL ----------------------------------------*/
  GPIO_InitStructure.GPIO_Pin =  SENSORS_I2C_SCL_GPIO_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

  /* Connect pins to Periph */
  GPIO_PinAFConfig(SENSORS_I2C_SCL_GPIO_PORT, SENSORS_I2C_SCL_GPIO_PINSOURCE, SENSORS_I2C_AF);  
  GPIO_Init(SENSORS_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

  /* Configure I2Cx pin: SDA ----------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = SENSORS_I2C_SDA_GPIO_PIN; 

  /* Connect pins to Periph */
  GPIO_PinAFConfig(SENSORS_I2C_SDA_GPIO_PORT, SENSORS_I2C_SDA_GPIO_PINSOURCE, SENSORS_I2C_AF);  
  GPIO_Init(SENSORS_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);  
  
  I2C_DeInit(I2C1);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = 400000;
    
  /* Enable the I2C peripheral */
  I2C_Cmd(I2C1, ENABLE);
    
  /* Initialize the I2C peripheral */
  I2C_Init(I2C1, &I2C_InitStructure);
}

/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
static uint32_t I2Cx_TIMEOUT_UserCallback(char value)
{
  
  /* The following code allows I2C error recovery and return to normal communication
     if the error source doesn’t still exist (ie. hardware issue..) */
  I2C_InitTypeDef I2C_InitStructure;
  
  I2C_GenerateSTOP(I2C1, ENABLE);
  I2C_SoftwareResetCmd(I2C1, ENABLE);
  I2C_SoftwareResetCmd(I2C1, DISABLE);
  
   I2C_DeInit(I2C1);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = 400000;
    
  /* Enable the I2C peripheral */
  I2C_Cmd(I2C1, ENABLE);
    
  /* Initialize the I2C peripheral */
  I2C_Init(I2C1, &I2C_InitStructure);

  return 1;
}



/**
  * @brief  Writes a Byte to a given register to the sensors through the 
            control interface (I2C)
  * @param  RegisterAddr: The address (location) of the register to be written.
  * @param  RegisterValue: the Byte value to be written into destination register.
  * @retval 0 if correct communication, else wrong communication
  */
unsigned long Sensors_I2C_WriteRegister(unsigned char Address, unsigned char RegisterAddr, unsigned char RegisterLen, unsigned char *RegisterValue)
{
  uint32_t result = 0;
  uint32_t i = RegisterLen;
  __IO uint32_t  I2CTimeout = I2Cx_LONG_TIMEOUT;
  
  RegisterValue = RegisterValue + (RegisterLen - 1);

  /* Wait for the busy flag to be cleared */
  WAIT_FOR_FLAG (I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 1);
  
  /* Start the config sequence */
  I2C_GenerateSTART(I2C1, ENABLE);

  /* Wait for the start bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 2);

  /* Transmit the slave address and enable writing operation */
  I2C_Send7bitAddress(I2C1, (Address<<1), I2C_Direction_Transmitter);
  
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 3);
  
  /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
  CLEAR_ADDR_BIT
  
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 4);

  /* Transmit the first address for write operation */
  I2C_SendData(I2C1, RegisterAddr);

  while (i--)
  {
    /* Wait for address bit to be set */
    WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 5);
  
    /* Prepare the register value to be sent */
    I2C_SendData(I2C1, *RegisterValue--);
  }
  
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 6);
  
  /* End the configuration sequence */
  I2C_GenerateSTOP(I2C1, ENABLE);
  
  /* Return the verifying value: 0 (Passed) or 1 (Failed) */
  return result;  
}

unsigned long Sensors_I2C_ReadRegister(unsigned char Address, unsigned char RegisterAddr, unsigned char RegisterLen, unsigned char *RegisterValue)
{
  uint32_t result = 0;
  uint8_t  bytesRemaining = RegisterLen;
  __IO uint32_t  I2CTimeout = I2Cx_LONG_TIMEOUT;
  
     
  /* Wait for the busy flag to be cleared */
  WAIT_FOR_FLAG (I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 7);
  
  /* Start the config sequence */
  I2C_GenerateSTART(I2C1, ENABLE);

  /* Wait for the start bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 8);
  
  /* Transmit the slave address and enable writing operation */
  I2C_Send7bitAddress(I2C1, (Address<<1), I2C_Direction_Transmitter);

  /* Wait for the start bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 9);

  /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
  CLEAR_ADDR_BIT;
  
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 10);
  
  /* Transmit the register address to be read */
  I2C_SendData(I2C1, RegisterAddr);
  
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 11);  

  /*!< Send START condition a second time */  
  I2C_GenerateSTART(I2C1, ENABLE);
  
  /* Wait for the start bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 12);
  
  /*!< Send address for read */
  I2C_Send7bitAddress(I2C1, (Address<<1), I2C_Direction_Receiver);
  
  /* Wait for the start bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 13);
  
  if (RegisterLen == 1) {
    /*!< Disable Acknowledgment */
    I2C_AcknowledgeConfig(I2C1, DISABLE);
    
    /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
    CLEAR_ADDR_BIT;
    
    /*!< Send STOP Condition */
    I2C_GenerateSTOP(I2C1, ENABLE);
    
    /* Wait for the RXNE bit to be set */
    WAIT_FOR_FLAG (I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 14);
    
    bytesRemaining--;
    RegisterValue[bytesRemaining] = I2C_ReceiveData(I2C1);
    
  } else if( RegisterLen == 2) {
   
     /*!< Disable Acknowledgment */
    I2C_AcknowledgeConfig(I2C1, DISABLE);
    
   /* Set POS bit */ 
    I2C1->CR1 |= I2C_CR1_POS;
   
   /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
   CLEAR_ADDR_BIT; 
   
   /* Wait for the buffer full bit to be set */
   WAIT_FOR_FLAG (I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 15);
   
   /*!< Send STOP Condition */
   I2C_GenerateSTOP(I2C1, ENABLE);

   /* Read 2 bytes */
   bytesRemaining--;
   RegisterValue[bytesRemaining] = I2C_ReceiveData(I2C1);
   bytesRemaining--;
   RegisterValue[bytesRemaining] = I2C_ReceiveData(I2C1);
    
  } else { /* more than 2 bytes */
    
    /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
    CLEAR_ADDR_BIT;
    
    while(bytesRemaining)
    {
      if( bytesRemaining == 3)
      {
        /* Wait for the buffer full bit to be set */
        WAIT_FOR_FLAG (I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 16);
        
        /*!< Disable Acknowledgment */
        I2C_AcknowledgeConfig(I2C1, DISABLE);
        
        /* Read 1 bytes */
        bytesRemaining--;
        RegisterValue[bytesRemaining] = I2C_ReceiveData(I2C1);
        
        /*!< Send STOP Condition */
        I2C_GenerateSTOP(I2C1, ENABLE);
        
        /* Read 1 bytes */
        bytesRemaining--;
        RegisterValue[bytesRemaining] = I2C_ReceiveData(I2C1);
        
        /* Wait for the buffer full bit to be set */
        WAIT_FOR_FLAG (I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 17);
        
        /* Read 1 bytes */
        bytesRemaining--;
        RegisterValue[bytesRemaining] = I2C_ReceiveData(I2C1);
      }
      else
      {
        /* Wait for the RXNE bit to be set */
        WAIT_FOR_FLAG (I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 18);
        
        /* Read 2 bytes */
        bytesRemaining--;
        RegisterValue[bytesRemaining] = I2C_ReceiveData(I2C1);
      }
    }
  } 
  /* Clear BTF flag */
  I2C_ClearFlag(I2C1, I2C_FLAG_BTF);
  /* Wait for the busy flag to be cleared */
  WAIT_FOR_FLAG (I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 19);  
  /*!< Re-Enable Acknowledgment to be ready for another reception */
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  //Disable POS -- TODO
  I2C1->CR1 &= ~I2C_CR1_POS;
     
  /* Return the byte read from sensor */
  return result;
}

int Sensors_I2C_WriteRegister_swap(unsigned char slave_addr,unsigned char reg_addr,unsigned char len, unsigned char *data_ptr)
{
  int i;
  unsigned char swap_array[32];
  
  for (i = 0; i < len; i++) 
    swap_array[len-1-i] = *data_ptr++;
                              
  return Sensors_I2C_WriteRegister((slave_addr),reg_addr, len, (unsigned char *)swap_array); 
   
}

int Sensors_I2C_ReadRegister_swap(unsigned char slave_addr,unsigned char reg_addr,unsigned char len, unsigned char *data_ptr)
{
  int i;
  volatile unsigned char swap_array[32];
  int ret = 0;
   
   ret = Sensors_I2C_ReadRegister((slave_addr),reg_addr, len, (unsigned char *)swap_array);
   if(ret)
     return ret;
   
   for (i = 0; i < len; i++) 
      *data_ptr++ = swap_array[len-1-i];

   return ret;
}


