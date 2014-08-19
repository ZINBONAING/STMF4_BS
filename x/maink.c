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

// Update Result here ---

//----ok now is to test out moving average and higher sampling---

// End update resilt here.---


	int main(void){

/*

		 for(ki=0;ki<Logbuf;ki++){

					  M1x[ki]=-99;
					  M2x[ki]=-99;
					  M3x[ki]=-99;
					  M4x[ki]=-99;

					 Ptermx[ki]=-99;
					 Dtermx[ki]=-99;
					 Itermx[ki]=-99;
					  PIDx[ki]=-99;
					  Radioinx[ki]=-99;
					  XErrbuffer[ki]=-99;

				 			                }

*/










I2C1_init(); // initialize I2C peripheral










//MPUIni_init16();
sensor_value=0;
//sensor_value=MPU9150_read1byte(MPU9150_PWR_MGMT_1);
serial_output("PWRMG1 = %x ",sensor_value);
sensor_value=MPU9150_read1byte(MPU9150_WHO_AM_I);
//serial_output("I am MPU = %x ",sensor_value);
int cntG;
for(cntG=0;cntG<3;cntG++){




    //	TIM_SetCompare1(TIM1, 9500); //M1 --3153 rpm
   //  TIM_SetCompare2(TIM1, 9500); //M2 -- 1289 rpm
   //  TIM_SetCompare3(TIM1, 9500); //M3 -- 3163 rpm
  //   TIM_SetCompare4(TIM1, 9500); //M4 -- 3417 rpm


 // 	TIM_SetCompare1(TIM1, 11000); //M1 --4812 rpm
  //   TIM_SetCompare2(TIM1, 11000); //M2 -- 4475 rpm
   //  TIM_SetCompare3(TIM1, 11000); //M3 -- 4747 rpm
   //    TIM_SetCompare4(TIM1, 11000); //M4 --   rpm




while(1){






//RxGyroR
//	M1Radio_in,M2Radio_in,M3Radio_in,M4Radio_in;
//	serial_output("SetX\t%c%d.%d\t",Csign(setX),C1(setX),C2(setX));
//	serial_output("SetY\t%c%d.%d\t",Csign(setY),C1(setY),C2(setY));
	serial_output("SetZ\t%c%d.%d\t",Csign(setZ),C1(setZ),C2(setZ));
    serial_output("ErrZ:\t%c%d.%d\t",Csign(ErrorZ),C1(ErrorZ),C2(ErrorZ));
    serial_output("Voltage \t%c%d.%d,",Csign(Batter_V),C1(Batter_V),C2(Batter_V));
  //  serial_output("%c%d.%d,",Csign(RzAccR),C1(RzAccR),C2(RzAccR));
	serial_output("Roll:\t%c%d.%d\t",Csign(DM_roll),C1(DM_roll),C2(DM_roll));
	 serial_output("Pitch:\t%c%d.%d\t",Csign(DM_pitch),C1(DM_pitch),C2(DM_pitch));
	 serial_output("Raw:\t%c%d.%d\t",Csign(DM_raw),C1(DM_raw),C2(DM_raw));
//	 serial_output("raw:\t%c%d.%d\t",Csign(DM_raw),C1(DM_raw),C2(DM_raw));
  // serial_output("%c%d.%d\t",Csign(M1Radio_in),C1(M1Radio_in),C2(M1Radio_in));
  // serial_output("%c%d.%d\t",Csign(M2Radio_in),C1(M2Radio_in),C2(M2Radio_in));
  // serial_output("%c%d.%d\t",Csign(M3Radio_in),C1(M3Radio_in),C2(M3Radio_in));
  // serial_output("%c%d.%d\t",Csign(M4Radio_in),C1(M4Radio_in),C2(M4Radio_in));

	   serial_output("%c%d.%d\t",Csign(M1),C1(M1),C2(M1));
	   serial_output("%c%d.%d\t",Csign(M2),C1(M2),C2(M2));
	   serial_output("%c%d.%d\t",Csign(M3),C1(M3),C2(M3));
	   serial_output("%c%d.%d\t",Csign(M4),C1(M4),C2(M4));

 //  serial_output("GyroX:\t%c%d.%d\t",Csign(RxGyroR),C1(RxGyroR),C2(RxGyroR));
 //  serial_output("GyroXRAW:\t%c%d.%d\t",Csign(GyroXvalue),C1(GyroXvalue),C2(GyroXvalue));
 //  serial_output("ErrRateX :\t%c%d.%d\t",Csign(ErrRateX),C1(ErrRateX),C2(ErrRateX));
   serial_output("XAngle PID:\t%c%d.%d\t",Csign(pidx),C1(pidx),C2(pidx));
   serial_output("YAngle PID:\t%c%d.%d\t",Csign(pidy),C1(pidy),C2(pidy));
   serial_output("ZAngle PID:\t%c%d.%d\t",Csign(pidz),C1(pidz),C2(pidz));
   //serial_output("XRatePID :\t%c%d.%d\t",Csign(PIDRateX),C1(PIDRateX),C2(PIDRateX));


 //serial_output("DM_CompAngRateX:\t%c%d.%d\t",Csign(DM_CompAngRateX),C1(DM_CompAngRateX),C2(DM_CompAngRateX));

 //serial_output("Gyro Sensitivity %d,",Gyro_sensitivity);


/*
 *
 *
 *
 * 	ErrRateX=pidx-RxGyroR;
								 	PtermRateX=	ErrRateX*RateXPG;
								 	DiffErrRateX=ErrRateX-PreviousErrRateX;
								 	DtermRateX=DiffErrRateX*RateXDG;
								 	IntErrRateX=IntErrRateX+ErrRateX;
								 	ItermRateX=RateXIG*IntErrRateX;
								 	PIDRateX=PtermRateX+DtermRateX+ItermRateX;
 *
 *
 *
 */


  // float RateYPG=200,RateYDG=0,RateYIG=50,SetYRate=-5;
  // float PreviousErrRateY,ErrRateY,DiffErrRateY,IntErrRateY,PtermRateY,DtermRateY,ItermRateY;
  /*
   ErrRateY=SetYRate-RyGyroR;
   	PtermRateY=	ErrRateY*RateYPG;
   	DiffErrRateY=ErrRateY-PreviousErrRateY;
   	DtermRateY=DiffErrRateY*RateYDG;
   	IntErrRateY=IntErrRateY+ErrRateY;
   	ItermRateY=RateYIG*IntErrRateY;
   	*/
  //  serial_output("%d,",M1);
  //  serial_output("%d,",M2);
  //  serial_output("%d,",M3);
  //  serial_output("%d,",M4);
 //   serial_output("%d,",sensorheight);
  serial_output("Height\t%c%d.%d cm\t",Csign(sensorheight),C1(sensorheight),C2(sensorheight));
//   serial_output("AC\t %c%d.%d\t cm,",Csign(actualclimbrate),C1(actualclimbrate),C2(actualclimbrate));
//   serial_output(" EAC\t %c%d.%d\t cm,",Csign( ErrorClimbrate),C1( ErrorClimbrate),C2( ErrorClimbrate));
//

   // serial_output("motor=%d,",StartMotor);
 //   serial_output("FlightMode=%d\t",flightmode);
   //serial_output("%c%d.%d,",Csign(ErrorH),C1(ErrorH),C2(ErrorH));
  // temperrX=ErrorX*100;
   // temperrY=ErrorY*100;
   // serial_output("%c%d.%d,",Csign(temperrX),C1(temperrX),C2(temperrX));
   // serial_output("%c%d.%d",Csign(temperrY),C1(temperrY),C2(temperrY));

   // serial_output("%c%d.%d,",Csign(RzAccR),C1(RzAccR),C2(RzAccR));
   // serial_output("%c%d.%d,",Csign(RzAccR),C1(RzAccR),C2(RzAccR));
   // serial_output("%c%d.%d,",Csign(RzAccR),C1(RzAccR),C2(RzAccR));

  // serial_output("P:%c%d.%d\t",Csign(p_termx),C1(p_termx),C2(p_termx));
  // serial_output("I:%c%d.%d\t",Csign(i_termx),C1(i_termx),C2(i_termx));
  // serial_output("D:%c%d.%d\t",Csign(d_termx),C1(d_termx),C2(d_termx));
 // serial_output("Radio PWM:%c%d.%d\t",Csign(radioin),C1(radioin),C2(radioin));
 //   ErrorH=setheight

  //  serial_output("ErrorH:%c%d.%d\t",Csign(ErrorH),C1(ErrorH),C2(ErrorH));
   // serial_output("setheight:\t%c%d.%d cm \t",Csign(setheight),C1(setheight),C2(setheight));


  //  serial_output("Xangle= \t%c%d.%d\t",Csign(Axr),C1(Axr),C2(Axr));
  //  serial_output("AccAngleX= \t%c%d.%d\t",Csign(AccAngleX),C1(AccAngleX),C2(AccAngleX));
  //  serial_output("GyroAngleX= \t%c%d.%d\t",Csign(Axz*180/PI),C1(Axz*180/PI),C2(Axz*180/PI));

 //   serial_output("ErrX=\t%c%d.%d\t",Csign(ErrorX),C1(ErrorX),C2(ErrorX));

   // serial_output("Yangle= \t%c%d.%d\t",Csign(Ayr),C1(Ayr),C2(Ayr));
  //  serial_output("ErrY=\t%c%d.%d\t",Csign(ErrorY),C1(ErrorY),C2(ErrorY));

 //   serial_output("PIDRateY=\t%c%d.%d\t",Csign(PIDRateY),C1(PIDRateY),C2(PIDRateY));
 //   serial_output("PIDRateX=\t%c%d.%d\t",Csign(PIDRateX),C1(PIDRateX),C2(PIDRateX));

 //   serial_output("IntErrRateX=\t%c%d.%d\t",Csign(IntErrRateX),C1(IntErrRateX),C2(IntErrRateX));
 //   serial_output("IntErrRateY=\t%c%d.%d\t",Csign(IntErrRateY),C1(IntErrRateY),C2(IntErrRateY));
   serial_output("ChangeinRadio=\t%c%d.%d\t",Csign( ChangeinRadio),C1( ChangeinRadio),C2( ChangeinRadio));



  //  PIDRateY+PIDRateX
  // serial_output("SM:%d\t",StartMotor);
  // serial_output("RS:%d\t",Radio_status);

 //   serial_output("StableMode:%d\t",StableMode);
 //   serial_output("FlightMode:%d\t",flightmode);

 //   serial_output("Radio=%c%d.%d\t",Csign(DutyCycle2_radio),C1(DutyCycle2_radio),C2(DutyCycle2_radio));

    //DutyCycle2_radio5
 //   serial_output("Radio6=%c%d.%d\t",Csign(DutyCycle2_radio5),C1(DutyCycle2_radio5),C2(DutyCycle2_radio5));
//StableMode
    //


//    serial_output("Ayr : %c%d.%d",Csign(Ayr),C1(Ayr),C2(Ayr));
    //serial_output("%5d,",AccYvalue);
   // serial_output("#Est%f<",RyEst);
   // serial_output("%5d,",AccZvalue);
   // serial_output("#Est %f<",RzEst);

  //  serial_output("Radio1=%5d\t ",DutyCycle2_radio1);
    serial_output("\n");

}


}
return 0;
}

/* @brief  This function handles TIM4 global interrupt request.
* @param  None
* @retval None
*/

