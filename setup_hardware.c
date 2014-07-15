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
#include <Global_variables.h>
int Buf_counter=0;
int expect_received=0,receivedmsg[20],received_msg=0,bufcount=0;






void init_USART3(uint32_t baudrate){

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
    USART_InitStruct.USART_BaudRate = 57600;
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


//----------------------------UART 4 for 3DM----------------------------------------------------------
void init_USART2(uint32_t baudrate){

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
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	  /* GPIOB clock enable */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* This sequence sets up the TX and RX pins
	GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Connect USART pins to AF */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource5, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART2);

	/* Now the USART_InitStruct is used to define the
	 * properties of USART1
	 */
    USART_InitStruct.USART_BaudRate = 38400;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART2, &USART_InitStruct);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

        	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;		 // we want to configure the USART1 interrupts
        	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;// this sets the priority group of the USART1 interrupts
        	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		 // this sets the subpriority inside the group
        	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
        	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff
    USART_Cmd(USART2, ENABLE);





}



//----------------------------END UART4 for 3DM-------------------------------------------------------
/* This function is used to transmit a string of characters via
 * the USART specified in USARTx.
 *
 * It takes two arguments: USARTx --> can be any of the USARTs e.g. USART1, USART2 etc.
 * 						   (volatile) char *s is the string you want to send
 *
 * Note: The string has to be passed to the function as a pointer because
 * 		 the compiler doesn't know the 'string' data type. In standard
 * 		 C a string is just an array of characters
 *
 * Note 2: At the moment it takes a volatile char because the received_string variable
 * 		   declared as volatile char --> otherwise the compiler will spit out warnings
 * */
void USART_puts(USART_TypeDef* USARTx, volatile char *s){

	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) );
		USART_SendData(USARTx, *s);
		*s++;
	}
}

void Transmit_UART(USART_TypeDef* USARTx,int bufcounter){

	char txchar;
	txchar = serial_buffer[ Buf_counter++ ] ;
	//	txchar ='K';
		if ( txchar )
		{
			while( !(USARTx->SR & 0x00000040) );
					USART_SendData(USARTx,txchar);
		}
	   else
		{
			sb_index = 0 ;
			end_index = 0 ;

		}



}

// this is the interrupt request handler (IRQ) for ALL USART1 interrupts
void USART3_IRQHandler(void){

	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART3, USART_IT_RXNE) ){

		static uint8_t cnt = 0; // this counter is used to determine the string length
		char t = USART3->DR; // the character from the USART1 data register is saved in t

		/* check if the received character is not the LF character (used to determine end of string)
		 * or the if the maximum string length has been been reached
		 */
		/*
		if( (t != '\n') && (cnt < MAX_STRLEN) ){
			received_string[cnt] = t;
			cnt++;
		}
		else{ // otherwise reset the character counter and print the received string
			cnt = 0;
			USART_puts(USART3, received_string);
		}
		/*
		 *
		 *
		 */
		if(t == 'u'){
			int k;
			float sampletime=0.0;

			 Delay1(1000000L);
			 Delay1(1000000L);
			 Delay1(1000000L);
			 Delay1(1000000L);

				            StartMotor=0;
				            TIM_SetCompare1(TIM1, 450);
				            			 	TIM_SetCompare2(TIM1, 450);
				            			 	TIM_SetCompare3(TIM1, 450);
				            	            TIM_SetCompare4(TIM1, 450);


			stoptimer=timercount+250000;
		}


		else if(t == 's')
		{
			StartMotor=0;
			USART_puts(USART3, "O");
				}
/*
		else if(t == '+')
				{
			trottle_manual=trottle_manual+step;
			trottleintrrupt=1;
					USART_puts(USART3, "+");
						}

		else if(t == '-')
						{
			        trottleintrrupt=1;
					trottle_manual=trottle_manual-step;
							USART_puts(USART3, "-");
								}

		else if(t == 'c')
								{


			setmotorspeed=1;
			USART_puts(USART3, "c");
										}
        if(state1==1){
        	if(t == '1'){        }
        	else if(t =='2'){        }
        	else if(t =='3'){        }
        	else if(t =='4'){        }


        }
*/
		}


	}
void USART2_IRQHandler(void){

	// check if the USART4 receive interrupt flag was set
	if( USART_GetITStatus(USART2, USART_IT_RXNE) ){

		static uint8_t cnt = 0; // this counter is used to determine the string length
		char t = USART2->DR; // the character from the USART1 data register is saved in t

		if(expect_received==1){
			if(t == 0x0E ){
			 bufcount=1;
			 receivedmsg[bufcount]=t;
			}
			expect_received=2;

		}
		if(expect_received==2){
			bufcount=bufcount+1;
			receivedmsg[bufcount]=t;

		}

		if(bufcount>10){

			expect_received==0;
			received_msg=1;

		}



		}


	}


//------------------End UART setup---------------------------------



//-----------------Start Printf function for serial port----------
//-- Zin Bo 3-Feb-2014-------------------------------------------
void serial_output(char* format, ... )
{
	va_list arglist ;

	va_start(arglist, format) ;

	int start_index = end_index ;
	int remaining = SERIAL_BUFFER_SIZE - start_index ;

	if (remaining > 1)
	{
	int wrote = vsnprintf( (char*)(&serial_buffer[start_index]), (size_t)remaining, format, arglist) ;

	end_index = start_index + wrote;
    if(end_index>512){

    	sb_index = 0 ;
		end_index = 0 ;

    }


	}

	Buf_counter=0;
	while(Buf_counter<=end_index){

		Transmit_UART(USART3,Buf_counter);


	}



		// IFS1bits.U2TXIF =1 ; // trigger the TX interrupt



	va_end(arglist);

	return ;

}



//---------------- End printf function for serial port------------



void I2C1_init(void){

GPIO_InitTypeDef GPIO_InitStruct;
I2C_InitTypeDef I2C_InitStruct;
NVIC_InitTypeDef NVIC_InitStructure;
// enable APB1 peripheral clock for I2C1
RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
// enable clock for SCL and SDA pins
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

/* setup SCL and SDA pins
* You can connect the I2C1 functions to two different
* pins:
* 1. SCL on PB6 or PB8
* 2. SDA on PB7 or PB9
*/
GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9; // we are going to use PB6 and PB9
GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;	// set pins to alternate function
GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;	// set GPIO speed
GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;	// set output to open drain --> the line has to be only pulled low, not driven high
GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;//GPIO_PuPd_UP;	// enable pull up resistors
GPIO_Init(GPIOB, &GPIO_InitStruct);	// init GPIOB

// Connect I2C1 pins to AF
GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);	// SCL
GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1); // SDA

// configure I2C1
I2C_InitStruct.I2C_ClockSpeed = 400000; // 100kHz original 400 000
I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;	// I2C mode
I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
I2C_InitStruct.I2C_OwnAddress1 = 0x00;	// own address, not relevant in master mode
I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;	// disable acknowledge when reading (can be changed later on)
I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
I2C_Init(I2C1, &I2C_InitStruct);	// init I2C1

/*Configure the SPI interrupt priority*/
NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);

NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
NVIC_Init(&NVIC_InitStructure);

// enable I2C1
I2C_Cmd(I2C1, ENABLE);
//I2C_ITConfig(I2C1, I2C_IT_BUF | I2C_IT_EVT, ENABLE);
}

/* This function issues a start condition and
* transmits the slave address + R/W bit
*
* Parameters:
* I2Cx --> the I2C peripheral e.g. I2C1
* address --> the 7 bit slave address
* direction --> the transmission direction can be:
* I2C_Direction_Tranmitter for Master transmitter mode
* I2C_Direction_Receiver for Master receiver
*/
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){

	int timeout;
	  //-----------------------------------------------------------------------------
	  timeout = I2C_TIMEOUT;
// wait until I2C1 is not busy any more
while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
	 if((timeout--)==0)   // wait until I2C-Bus is not busy anymore
	    {
	      return ERROR;
	    }

// Send I2C1 START condition
I2C_GenerateSTART(I2Cx, ENABLE);

// wait for I2C1 EV5 --> Slave has acknowledged start condition
while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

// Send slave Address for write
I2C_Send7bitAddress(I2Cx, address, direction);

/* wait for I2Cx EV6, check if
* either Slave has acknowledged Master transmitter or
* Master receiver mode, depending on the transmission
* direction
*/
if(direction == I2C_Direction_Transmitter){
while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	if((timeout--)==0)   // wait until I2C-Bus is not busy anymore
		    {
		      return ERROR;
		    }
}
else if(direction == I2C_Direction_Receiver){
while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	if((timeout--)==0)   // wait until I2C-Bus is not busy anymore
		    {
		      return ERROR;
		    }
}
}

/* This function transmits one byte to the slave device
* Parameters:
* I2Cx --> the I2C peripheral e.g. I2C1
* data --> the data byte to be transmitted
*/
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
	int timeout;
		  //-----------------------------------------------------------------------------
		  timeout = I2C_TIMEOUT;
// wait for I2C1 EV8 --> last byte is still being transmitted (last byte in SR, buffer empty), next byte can already be written
while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
	if((timeout--)==0)   // wait until I2C-Bus is not busy anymore
			    {
			      return ERROR;
			    }
I2C_SendData(I2Cx, data);
}

/* This function reads one byte from the slave device
* and acknowledges the byte (requests another byte)
*/
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
// enable acknowledge of received data
I2C_AcknowledgeConfig(I2Cx, ENABLE);
// wait until one byte has been received
while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
// read data from I2C data register and return data byte
uint8_t data = I2C_ReceiveData(I2Cx);
return data;
}

/* This function reads one byte from the slave device
* and doesn't acknowledge the received data
* after that a STOP condition is transmitted
*/
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
	int timeout;
		  //-----------------------------------------------------------------------------
		  timeout = I2C_TIMEOUT;
// disable acknowledge of received data
// nack also generates stop condition after last byte received
// see reference manual for more info
I2C_AcknowledgeConfig(I2Cx, DISABLE);
I2C_GenerateSTOP(I2Cx, ENABLE);
// wait until one byte has been received
while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) )
	if((timeout--)==0)   // wait until I2C-Bus is not busy anymore
				    {
				      return ERROR;
				    }
// read data from I2C data register and return data byte
uint8_t data = I2C_ReceiveData(I2Cx);
return data;
}

/* This function issues a stop condition and therefore
* releases the bus
*/
void I2C_stop(I2C_TypeDef* I2Cx){

	int timeout;
			  //-----------------------------------------------------------------------------
			  timeout = I2C_TIMEOUT;

// Send I2C1 STOP Condition after last byte has been transmitted
I2C_GenerateSTOP(I2Cx, ENABLE);
// wait for I2C1 EV8_2 --> byte has been transmitted
while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	if((timeout--)==0)   // wait until I2C-Bus is not busy anymore
				    {
				      return ERROR;
				    }
}
void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

void MPUIni_init16(void)
{
	// need at least 60 msec delay here
	Delay(20);
	MPU9150_writeSensor(MPU9150_PWR_MGMT_1, 0x0);
	Delay(20);
	MPU9150_writeSensor(MPU9150_ACCEL_CONFIG, 0x04); //+-8G 0.63Hz
    Delay(20);
    MPU9150_writeSensor(MPU9150_GYRO_CONFIG, 0x00); //+/1 500 deg per sec
     Delay(20);

}



void  MPU9150_writeSensor(uint16_t addr, uint16_t data)
{

	I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Transmitter);
	I2C_write(I2C1,addr);
	//I2C_stop(I2C1);
	Delay(50);
	//I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Transmitter);
	I2C_write(I2C1,data);
	I2C_stop(I2C1);
	//Delay(100);
	// this delay is necessary; it appears that SS must be deasserted for one or
	// more SPI clock cycles between writes
	// __delay_us(1);

	//-------------
	//  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
	//  Wire.write(addr);
	//  Wire.write(data);
	//  Wire.endTransmission(true);






	//-------------


}

int MPU9150_read1byte(int address){
	int Read_reg;
	I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Transmitter);
	I2C_write(I2C1, address);
	I2C_stop(I2C1);
	 // Wire.beginTransmission(MPU9150_I2C_ADDRESS);
	 // Wire.write(addrL);
	 // Wire.endTransmission(false);
	Delay(50);
	I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
	Read_reg = I2C_read_nack(I2C1);
	return Read_reg;

}

int MPU9150_readSensor(int addrL, int addrH){
 int byte_L =0,byte_H=0 ;

I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Transmitter);
I2C_write(I2C1, addrL);
I2C_stop(I2C1);
 // Wire.beginTransmission(MPU9150_I2C_ADDRESS);
 // Wire.write(addrL);
 // Wire.endTransmission(false);
Delay(50);
I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
byte_L = I2C_read_nack(I2C1);
//I2C_stop(I2C1);


I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Transmitter);
I2C_write(I2C1, addrH);
I2C_stop(I2C1);
I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
byte_H = I2C_read_nack(I2C1);
//I2C_stop(I2C1);

  return (byte_H<<8)+byte_L;
}

int toSignedInt(unsigned int value, int bitLength)
{
    int signedValue = value;
    if (value >> (bitLength - 1))
        signedValue |= -1 << bitLength;

    return signedValue;
}

int16_t silly_convert (int16_t i)
{
	i=~i;
	i=i-1;
  return i;
}


void InitializeTimer2()
{

	/*
	The comments don't jive with the values, hence you are rightly confused

	Going from 1 MHz to 1 KHz you'd expect a 1000 -1 value

	For a counter to have N states, it starts at 0 and counts up to N-1, the counter recognizes the N-1 state and the next state will be 0. The register is thus programed with N-1 rather than N.

	TIM Update Frequency = TIM Clock / (P * Q)

	Where Prescaler = P - 1, and Period = Q - 1

	The base TIM Clock with depend on the AHB divider for the bus to which it's attached.

	TIM2 on on APB1, the typical prescaler for that is 4, where the input for the timer is 2. See the Clock Tree diagram for this relationship. Thus the TIM Clock will nominally be 84 MHz (168 MHz / 2)

	Therefore to get the time base ticking at 1 MHz, the prescaler needs to be 84 - 1, a period of 1000 - 1 will get the update down to 1 KHz.

	Your example would generate 25 Hz in my estimation, a division of 3360000, whose factors include 336 and 10000

	*/
	 NVIC_InitTypeDef NVIC_InitStructure;
	 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;            /* Enable the TIM2 gloabal Interrupt */
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  /* value = 0 - 15*/
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         /* value = 0 - 15*/
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);


	  /* TIM2 clock enable */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	  /* Time base configuration */

	  TIM_TimeBaseStructure.TIM_Period = 48 - 1;  // this will generate timer interrrupt every 1ms
	  TIM_TimeBaseStructure.TIM_Prescaler = 335 - 1; // 24 MHz Clock down to 1 MHz (adjust per your clock)
	  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	  /* TIM IT enable */
	  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	  /* TIM2 enable counter */
	  TIM_Cmd(TIM2, ENABLE);



}

void InitPWM(){

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* TIM1 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	/* GPIOE clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/* TIM1 channel 2 pin (PE.9) configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* Connect TIM pins to AF2 */
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);

	TIM_TimeBaseStructInit (&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock/150000000); // pers 33.6
	TIM_TimeBaseStructure.TIM_Period = 40000; // 20ms for servo period- 51Hz -ZinBo 12-May-2014
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1 , &TIM_TimeBaseStructure);
	// PWM1 Mode configuration: Channel1
	// Edge -aligned; not single pulse mode
	TIM_OCStructInit (& TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC1Init(TIM1 , &TIM_OCInitStructure);

	TIM_BDTRInitTypeDef bdtr;
	TIM_BDTRStructInit(&bdtr);
	bdtr.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM1, &bdtr);


	// Enable Timer Interrupt and Timer
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //
	TIM_Cmd(TIM1 , ENABLE);

	TIM_SetCompare1(TIM1, 8100);    // 500- 1ms , 1000- 2ms ; thus range is between 2.


}

void InitPWM2(){

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* TIM1 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	/* GPIOE clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/* TIM1 channel 2 pin (PE.11) configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* Connect TIM pins to AF2 */
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);


	// PWM1 Mode configuration: Channel1
	// Edge -aligned; not single pulse mode
	TIM_OCStructInit (& TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC2Init(TIM1 , &TIM_OCInitStructure);

	TIM_BDTRInitTypeDef bdtr;
	TIM_BDTRStructInit(&bdtr);
	bdtr.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM1, &bdtr);


	// Enable Timer Interrupt and Timer
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //
	TIM_Cmd(TIM1 , ENABLE);

	TIM_SetCompare2(TIM1, 8100);    // 500- 1ms , 1000- 2ms ; thus range is between 2.


}

void InitPWM3(){

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* TIM1 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	/* GPIOE clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/* TIM1 channel 2 pin (PE.13) configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* Connect TIM pins to AF2 */
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);


	// PWM1 Mode configuration: Channel1
	// Edge -aligned; not single pulse mode
	TIM_OCStructInit (& TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC3Init(TIM1 , &TIM_OCInitStructure);

	TIM_BDTRInitTypeDef bdtr;
	TIM_BDTRStructInit(&bdtr);
	bdtr.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM1, &bdtr);


	// Enable Timer Interrupt and Timer
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //
	TIM_Cmd(TIM1 , ENABLE);

	TIM_SetCompare3(TIM1, 8100);    // 500- 1ms , 1000- 2ms ; thus range is between 2.


}
void InitPWM4(){

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* TIM1 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	/* GPIOE clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/* TIM1 channel 2 pin (PE.13) configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* Connect TIM pins to AF2 */
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);


	// PWM1 Mode configuration: Channel1
	// Edge -aligned; not single pulse mode
	TIM_OCStructInit (& TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC4Init(TIM1 , &TIM_OCInitStructure);

	TIM_BDTRInitTypeDef bdtr;
	TIM_BDTRStructInit(&bdtr);
	bdtr.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM1, &bdtr);


	// Enable Timer Interrupt and Timer
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //
	TIM_Cmd(TIM1 , ENABLE);

	TIM_SetCompare4(TIM1, 8100);    // 500- 1ms , 1000- 2ms ; thus range is between 2.


}


void PWMinput_sound(void)
{

	TIM_ICInitTypeDef  TIM_ICInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  /* GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  /* TIM4 chennel2 configuration : PB.07 */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Connect TIM pin to AF2 */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);


  //	TIM_TimeBaseStructure.TIM_Period = 40000; // 20ms for servo period- 51Hz -ZinBo 12-May-2014
  //	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  //	TIM_TimeBaseInit(TIM4 , &TIM_TimeBaseStructure);



  /* Enable the TIM4 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* TIM4 configuration: PWM Input mode ------------------------
     The external signal is connected to TIM4 CH2 pin (PB.07),
     The Rising edge is used as active edge,
     The TIM4 CCR2 is used to compute the frequency value
     The TIM4 CCR1 is used to compute the duty cycle value

     TIM5_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
  TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM5_ICInitStructure.TIM_ICFilter = 0x0;

  ------------------------------------------------------------ */

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV2;//TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

  TIM_PrescalerConfig(TIM4, 9, TIM_PSCReloadMode_Immediate);   //TIM2CLK/£¨20£©=3.6MHz

  TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);

  /* Select the TIM4 Input Trigger: TI2FP2 */
  TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2);

  /* Select the slave Mode: Reset Mode */
  TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM4,TIM_MasterSlaveMode_Enable);

  /* TIM enable counter */
  TIM_Cmd(TIM4, ENABLE);

  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);

}





void PWMinput_radioCH3(void)
{

	TIM_ICInitTypeDef  TIM_ICInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  /* TIM4 chennel2 configuration : PB.07 */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Connect TIM pin to AF2 */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);

  /* Enable the TIM4 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4; //0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* TIM4 configuration: PWM Input mode ------------------------
     The external signal is connected to TIM4 CH2 pin (PB.07),
     The Rising edge is used as active edge,
     The TIM4 CCR2 is used to compute the frequency value
     The TIM4 CCR1 is used to compute the duty cycle value
  ------------------------------------------------------------ */

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = 4;//TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

  TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);

  /* Select the TIM4 Input Trigger: TI2FP2 */
  TIM_SelectInputTrigger(TIM3, TIM_TS_TI2FP2);

  /* Select the slave Mode: Reset Mode */
  TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Enable);

  /* TIM enable counter */
  TIM_Cmd(TIM3, ENABLE);

  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);

}
void PWMinput_radioCH6(void)
{

	TIM_ICInitTypeDef  TIM_ICInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

  /* GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* TIM4 chennel2 configuration : PB.07 */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect TIM pin to AF2 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);

  /* Enable the TIM4 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* TIM4 configuration: PWM Input mode ------------------------
     The external signal is connected to TIM4 CH2 pin (PB.07),
     The Rising edge is used as active edge,
     The TIM4 CCR2 is used to compute the frequency value
     The TIM4 CCR1 is used to compute the duty cycle value
  ------------------------------------------------------------ */

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;//TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_PrescalerConfig(TIM5, 9, TIM_PSCReloadMode_Immediate);   //TIM2CLK/£¨20£©=3.6MHz
  TIM_PWMIConfig(TIM5, &TIM_ICInitStructure);

  /* Select the TIM4 Input Trigger: TI2FP2 */
  TIM_SelectInputTrigger(TIM5, TIM_TS_TI2FP2);

  /* Select the slave Mode: Reset Mode */
  TIM_SelectSlaveMode(TIM5, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM5,TIM_MasterSlaveMode_Enable);

  /* TIM enable counter */
  TIM_Cmd(TIM5, ENABLE);

  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM5, TIM_IT_CC2, ENABLE);

}





void init_GPIO(void){

	/* This TypeDef is a structure defined in the
	 * ST's library and it contains all the properties
	 * the corresponding peripheral has, such as output mode,
	 * pullup / pulldown resistors etc.
	 *
	 * These structures are defined for every peripheral so
	 * every peripheral has it's own TypeDef. The good news is
	 * they always work the same so once you've got a hang
	 * of it you can initialize any peripheral.
	 *
	 * The properties of the periperals can be found in the corresponding
	 * header file e.g. stm32f4xx_gpio.h and the source file stm32f4xx_gpio.c
	 */
	GPIO_InitTypeDef GPIO_InitStruct;

	/* This enables the peripheral clock to the GPIOD IO module
	 * Every peripheral's clock has to be enabled
	 *
	 * The STM32F4 Discovery's User Manual and the STM32F407VGT6's
	 * datasheet contain the information which peripheral clock has to be used.
	 *
	 * It is also mentioned at the beginning of the peripheral library's
	 * source file, e.g. stm32f4xx_gpio.c
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* In this block of instructions all the properties
	 * of the peripheral, the GPIO port in this case,
	 * are filled with actual information and then
	 * given to the Init function which takes care of
	 * the low level stuff (setting the correct bits in the
	 * peripheral's control register)
	 *
	 *
	 * The LEDs on the STM324F Discovery are connected to the
	 * pins PD12 thru PD15
	 */
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12; // we want to configure all LED GPIO pins // we want to configure all LED GPIO pins
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOD, &GPIO_InitStruct); 			// this finally passes all the values to the GPIO_Init function which takes care of setting the corresponding bits.

}


void Delay1(__IO uint32_t nCount) {
  while(nCount--) {
  }
}
