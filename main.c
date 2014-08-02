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
int firstime=0,rawmanualcontrol=1,initialbearing=0;
float Batter_V=0.0;

#define SERIAL_BUFFER_SIZE 512
char serial_buffer[SERIAL_BUFFER_SIZE] ;
int sb_index = 0 ;
int end_index = 0 ;
int togglebit=0;
int timercount=0;
int his_timercount=0;
int stoptimer=0;
int serialflag=0;
int movavgcounter=0;
int XErrbuf;
int expect_received=0,received_msg=0;
int16_t receivedmsg[25];
int takeoff=0;
float xtrim=0,ytrim=0.0;
// P=0.8 , I=0.1 , D=0.225
// P=3, I=1, D=0.3
//P=0.17 D=0.16 when upper limit set to 600
//0.34 and 0.28 under control for 2x prop
//small battery 0.23 and 0.22 seems ok. slow thouggh,0.33,0.34
//small batter 0.165 , 0.17 ok. 4 props
//4 x prop small battery
//1.9 and Dgain 1.6 is ok

//Rate PID PG=4.4 , D=5.4 , Kp=0.7
//Rate PID PG=4.2 , D=5.4 , Kp=0.9 ,IGain=0
//Rate PID PG=4.2 , D=7 , Kp=0.9 ,IGain=0
//best value pG=5.2 Dgain=7.2 , Kp=0.8 , Ki=0.005

//high trottle gain
//Pagin =4.6, Xgain =7 ,
int manualradio=14000;
#define Logbuf 500 //
int PIDoption=1;  // 0= Gyro , 1 =Angle
float PGain=15.3,PgainX=15.3,ErrorX=0,ErrorY=0,setX=0,setY=0,setZ,setheight=300,ErrorH=0,GH=100;
float IGain=0,Dgain=11,err_diffX=0.0,err_diffY=0.0,int_errX=0.0,int_errY=0.0,PreviousErrX=0.0,PreviousErrY=0.0;
float PgainZ=20,IgainZ=0,DgainZ=4;
//--------------------------------------------- Rate PID ---------------------------------------------------------
float RateYPG=0.8,RateYDG=0,RateYIG=0,SetYRate=5,RateZPG=0.8,RateZDG=0,RateZIG=0;
float PreviousErrRateY,ErrRateY,ErrRateZ,PtermRateZ,DiffErrRateY,IntErrRateY,PtermRateY,DtermRateY,ItermRateY,DiffErrRateZ,PreviousErrRateZ=0,IntErrRateZ,ItermRateZ,PIDRateZ;
float PIDRateY,DtermRateZ,PIDRateZ;
float PreviousAlt_M1=13000,PreviousAlt_M2=13000,PreviousAlt_M3=13000,PreviousAlt_M4=13000;

/*
ErrRateZ=setZ-DM_CompAngRateZ
							 	PtermRateZ=	ErrRateZ*RateZPG;
							 	DiffErrRateZ=ErrRateZ-PreviousErrRateZ;
							 	DtermRateZ=DiffErrRateZ*RateZDG;
							 	IntErrRateZ=IntErrRateZ+ErrRateZ;
							 	ItermRateZ=RateZIG*IntErrRateZ;
							 	PIDRateZ=PtermRateZ+DtermRateZ+ItermRateZ;
*/



float RateXPG=0.8,RateXDG=0,RateXIG=0.000,SetXRate=5;
float PreviousErrRateX,ErrRateX,DiffErrRateX,IntErrRateX,PtermRateX,DtermRateX,ItermRateX;
float PIDRateX,Axz,Ayz,RateAyzpast;

float p_termz=0.0,i_termz=0.0,d_termz=0.0,pidz,PreviousErrZ,ErrorZ,err_diffZ,int_errZ;
float p_termx=0.0,i_termx=0.0,d_termx=0.0,p_termy=0.0,i_termy=0.0,d_termy=0.0;
float pidx=0.0,pidy=0.0;
float M1=500,M2=500,M3=500,M4=500;
int16_t AccXA[20],AccYA[20],AccZA[20],GyroXA[20],GyroYA[20],GyroZA[20];
int16_t mutex=0;
int trottle_manual=0,trottleintrrupt=0,conttrolflag,setmotorspeed=0,state1=0;
float AccAngleX,AccAngleY,RateAyz,RateAxz;
float temperrX,temperrY;
int flightmode=0;
#define I2C_TIMEOUT  (0x5)
#define PI 3.14159265358979
float MXlimit=15000;
float MNlimit=9900;
#define step 100

int IC2Value_radio, DutyCycle_radio,DutyCycle2_radio,Frequency_radio,curr_ch3,prev_ch3;
int IC2Value_radioCh5,DutyCycle_radio5,DutyCycle2_radio5,Frequency_radio5;
int IC2Value_radioCh1,DutyCycle_radio1,DutyCycle2_radio1,Frequency_radio1;
int IC2Value_radioCh2,DutyCycle_radio2,DutyCycle2_radio2,Frequency_radio2;
int IC2Value_radioCh4,DutyCycle_radio4,DutyCycle2_radio4,Frequency_radio4;

int IC2Value_radioCh9,DutyCycle_radio9,DutyCycle2_radio9,Frequency_radio9;
float sensorheight=0;


float PGainH=0.5,IGainH=0,DgainH=4,CRateGain=25;
float setclimbrate=15,actualclimbrate=0,PreviousHeight=0,ErrorClimbrate=0,PidClimbRate=0;






float err_diffH,int_errH,p_termh,i_termh,d_termh,pidh,PreviousErrH;
int PreviousFlightMode=0,StableMode=0;
float 	RxAcc,RyAcc,RzAcc,RxGyro,RyGyro,RzGyro;
//Radio status state machine
int Radio_status=0;
int Rstate=0;
int ns_radio=0;
float radioin=0;
int tempradio_status=0;
int ConvertedValue = 0; //Converted value readed from ADC
float XErrbuffer[Logbuf];
float M1x[Logbuf];
float M2x[Logbuf];
float M3x[Logbuf];
float M4x[Logbuf];
float Ptermx[Logbuf];
float Dtermx[Logbuf];
float Itermx[Logbuf];
float PIDx[Logbuf];
float Radioinx[Logbuf];
float CurrentX[Logbuf];
int PID_Start=0;



float m1m3_rpm,m2m4_rpm,average_rpm,m1Rcompensate,m2Rcompensate,m3Rcompensate,m4Rcompensate;
float M1Radio_in,M2Radio_in,M3Radio_in,M4Radio_in;

float DM_roll,DM_pitch,DM_raw,DM_CompAngRateX,GyroGain,DM_CompAngRateY,DM_CompAngRateZ;
int16_t  DM_cmd,DM_roll_cal,DM_pitch_cal,DM_raw_cal,DM_AccelX_cal,DM_AccelY_cal,DM_AccelZ_cal,DM_CompAngRateX_cal,DM_CompAngRateY_cal,DM_CompAngRateZ_cal,DM_TimerTicks_cal,checksum;



//int16_t  DM_cmd;
int16_t CRCvalidation=0,Gyro_sensitivity=0;
//end Radio status state machine
void PWMinput_radioCH3(void);
void  UART2_sendbyte(uint8_t);
void radio_in(void);
//Version Number : Major.Minor.Complete/Progress
// Revision : ZIO1.M1.2 # added PWM for motor


// this slave address belongs to the STM32F4-Discovery board's
// CS43L22 Audio DAC
// connect PD4 to VDD in order to get the DAC out of reset and test the I2C
// interface
#define SLAVE_ADDRESS 0x68 // the slave address (example)

__IO uint16_t IC2Value = 0;
__IO uint16_t DutyCycle = 0;
__IO uint16_t StartMotor = 0;
__IO uint16_t DutyCycle2 = 0;
__IO uint32_t Frequency = 0;



int16_t AccXvalue=0;
int16_t AccYvalue=0;
int16_t  AccZvalue=0;
int16_t GyroXvalue=0;
int16_t GyroYvalue=0;
int16_t GyroZvalue=0;


float RxEst,RyEst,RzEst,RxEstpast,RyEstpast,RzEstpast,Axr,Ayr,Azr;
float 	RxAccR,RyAccR,RzAccR,RxGyroR,RyGyroR,RzGyroR;
float RateAxzpast,AyzPastpast;



int32_t RatePIDY(int DesiredRate){


	return(PIDRateY);

}












//-----------------------------------------End Rate PID -------------------------------------------------------------

#define SENSITIVITY 4096
int T=0;


#define MAX_STRLEN 5 // this is the maximum string length of our string in characters
volatile char received_string[MAX_STRLEN+1]; // this will hold the recieved string
//------------------Start UART Setup------------------------------
/*
* This funcion initializes the USART1 peripheral
 *
 * Arguments: baudrate --> the baudrate at which the USART is
 * 						   supposed to operate
 */

void fuseGyroAcc(int,int,int,int,int,int);

float squaredz(float x){
  return x*x;
}
void fuseGyroAcc(int RxAcc0,int RyAcc0,int RzAcc0,int RxGyro0,int RyGyro0,int RzGyro0){
	//    GPIOD->BSRRL = 0xF000; // set PD1


	float RaccModulus,RaccNormalize,SignRzGyro;


	float   AyzPast,AxzPast,R;
	float Wgyro=10.0;
	int i,j;
	float totalx=0.0,totaly=0.0,totalz=0.0;


	RxAccR=(((float)RxAcc0))/4096;
	RyAccR=(((float)RyAcc0))/4096;
	RzAccR=(((float)RzAcc0))/4096;




	// Avgerage last 20 sample


/*
	int16_t AccXA[20],AccYA[20],AccZA[20],GyroXA[20],GyroYA[20],GyroZA[20];

	AccXA[movavgcounter]=RxAccR;
	AccYA[movavgcounter]=RyAccR;
	AccZA[movavgcounter]=RzAccR;
	if(movavgcounter>10){
		movavgcounter=0;
	}

	for(i=0;i<11;i++){
		totalx=AccXA[i]+totalx;
		totaly=AccYA[i]+totaly;
		totalz=AccZA[i]+totalz;

	}

	//RxAccR=totalx/10.0;
	//RyAccR=totaly/10.0;
	//RzAccR=totalz/10.0;
	totalx=0;totaly=0;totalz=0;

	movavgcounter=movavgcounter+1;
	//end Avergae last 20 sample
*/
	RxGyroR=((((float)RxGyro0)+120)/131.0);
	RyGyroR=(((float)RyGyro0)+120)/131.0;
	RzGyroR=(((float)RzGyro0)+120)/131.0;
	//--up to here from eginning is 10us----
	RaccModulus=sqrt((RxAccR*RxAccR)+(RyAccR*RyAccR)+(RzAccR*RzAccR));
	RxAcc=RxAccR/RaccModulus;
	RyAcc=RyAccR/RaccModulus;
	RzAcc=RzAccR/RaccModulus;
	//AccAngleX,AccAngleY;

	AccAngleX=acos(RxAcc)*180/PI-90;
	AccAngleY=acos(RyAcc)*180/PI-90;
	//--up to here from eginning is 600us----
	 GPIOD->BSRRH = 0xF000; // reset PD1
	 //
    if(T==0){  //time is 0, first data point , define REst at 0 time.
	   RxEstpast= RxAcc;
	   RyEstpast= RyAcc;
       RzEstpast= RzAcc;

    }

    if( (RzEstpast<0.1) && (RzEstpast>-0.1)    ) {
    	RxGyro= RxAcc;
    	RyGyro= RyAcc;
    	RzGyro= RzAcc;

    }
    else
    {
    	RateAxz=1*RyGyroR*PI/180.0;
    	RateAyz=1*RxGyroR*PI/180.0;


    	AxzPast=atan2(RxEstpast,RzEstpast);
    	Axz=AxzPast+((RateAxz+RateAxzpast)/2.0)*0.007;//new angle based on Gyro data
    	AyzPast=atan2(RyEstpast,RzEstpast);
    	Ayz=AyzPast+((RateAyz+RateAyzpast)/2.0)*0.007;


    	if(cos(Axz)>=0)
    		{
    		SignRzGyro=1;
    		}
    	else
    	{
    		SignRzGyro=-1;
    	}


    	RxGyro=sin(Axz)/sqrt((1+ squaredz(cos(Axz))*squaredz(tan(Ayz))   ));
    	RyGyro=sin(Ayz)/sqrt((1+squaredz(cos(Ayz))*squaredz(tan(Axz))   ));
    	//0.9428 / (1+(-0.77)x4.21

    	//RzGyro=1;
    	RzGyro=SignRzGyro*sqrt(1-(RxGyro*RxGyro)-(RyGyro*RyGyro));



    }
    T=2; //make sure Tis not Zero so it won't initialize again
  // ------------------------------GYRO----------------------------------------------------



RxEst=(RxAcc+RxGyro*Wgyro)/(1+Wgyro);
RyEst=(RyAcc+RyGyro*Wgyro)/(1+Wgyro);
RzEst=(RzAcc+RzGyro*Wgyro)/(1+Wgyro);

R=sqrt((RxEst*RxEst)+(RyEst*RyEst)+(RzEst*RzEst));
RxEst=RxEst/R;
RyEst=RyEst/R;
RzEst=RzEst/R;


RxEstpast= RxEst;
RyEstpast= RyEst;
RzEstpast= RzEst;


Axr=acos(RxEst)*180/PI-90;
Ayr=acos(RyEst)*180/PI-90;
Azr=acos(RzEst)*180/PI-90;
RateAxzpast=RateAxz;
RateAyzpast=AyzPast;
}

char Csign(float Zee){
	char signn;

	if((Zee<0)&(Zee>-1)){
		signn='-';

	}
	else {
		signn=32;
	}
	return signn;
}

int C1(float Zee){
	int int1;
	float int2;

	int1=(int)Zee;

	return int1;
}
int C2(float Zee){
	int int1,ans;
	float int2,int3;
	if(Zee<0){
		Zee=Zee*(-1);

		}
	int1=(int)Zee;
	int2=(Zee-int1)*1000;
	ans=(int)int2;

	return ans;
}

int ki;

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
int sensor_value=0;

float heading,lat1=1.344026,lon1,lat2,lon2,headb;
adc_configure();//Start configuration



    	 ConvertedValue = adc_convert();//Read the ADC converted value
    	 //   delay(100);
    	    Batter_V=(((ConvertedValue/4096.0)*2.96)-0.20)*10.0;







I2C1_init(); // initialize I2C peripheral
InitPWM();
InitPWM2();
InitPWM3();
InitPWM4();

PWMinput_sound();
PWMinput_radioCH3();

PWMinput_radioCH6();
PWMinput_radioCH1();
PWMinput_radioCH2();
PWMinput_radioCH4();

InitializeTimer2();


Delay(50000);
	Delay(50000);
	Delay(50000);Delay(50000);
	Delay(50000);
	Delay(50000);Delay(50000);
	Delay(50000);
	Delay(50000);
init_USART3(9600);
init_USART2(38400);
init_USART4();
init_GPIO();
uint8_t received_data2=0xF0;
int sf;
for(sf=0;sf<24;sf++){
	receivedmsg[sf]=0;
}






//MPUIni_init16();
sensor_value=0;
//sensor_value=MPU9150_read1byte(MPU9150_PWR_MGMT_1);
serial_output("PWRMG1 = %x ",sensor_value);
sensor_value=MPU9150_read1byte(MPU9150_WHO_AM_I);
//serial_output("I am MPU = %x ",sensor_value);
int cntG;
for(cntG=0;cntG<3;cntG++){
UART2_sendbyte(0x28);
UART2_sendbyte(0x82);
expect_received=6;
Delay(50000);
Delay(50000);
Delay(50000);
Delay(50000);
Delay(50000);
Delay(50000);
Delay(50000);
Delay(50000);

}
GyroGain=(32768000/ Gyro_sensitivity);
expect_received=0;
  /* This flashed the LEDs on the board once
   * Two registers are used to set the pins (pin level is VCC)
   * or to reset the pins (pin level is GND)
   *
   * BSRR stands for bit set/reset register
   * it is seperated into a high and a low word (each of 16 bit size)
   *
   * A logical 1 in BSRRL will set the pin and a logical 1 in BSRRH will
   * reset the pin. A logical 0 in either register has no effect
   */
 // GPIOD->BSRRL = 0xF000; // set PD1
  Delay1(1000000L);		 // wait a short period of time
  //GPIOD->BSRRH = 0xF000; // reset PD1


  StartMotor=0;



    //	TIM_SetCompare1(TIM1, 9500); //M1 --3153 rpm
   //  TIM_SetCompare2(TIM1, 9500); //M2 -- 1289 rpm
   //  TIM_SetCompare3(TIM1, 9500); //M3 -- 3163 rpm
  //   TIM_SetCompare4(TIM1, 9500); //M4 -- 3417 rpm


 // 	TIM_SetCompare1(TIM1, 11000); //M1 --4812 rpm
  //   TIM_SetCompare2(TIM1, 11000); //M2 -- 4475 rpm
   //  TIM_SetCompare3(TIM1, 11000); //M3 -- 4747 rpm
   //    TIM_SetCompare4(TIM1, 11000); //M4 --   rpm




while(1){

	if(conttrolflag==1){
		GPIOD->BSRRL = 0x8000; // set PD1
	        skipIMU:
		            	ControlLoop();


			conttrolflag=0;


			 GPIOD->BSRRH = 0x8000; // reset PD1
		  //  while(received_msg!=1);


	}


if(serialflag==1){


//RxGyroR
//	M1Radio_in,M2Radio_in,M3Radio_in,M4Radio_in;
	serial_output("SetX\t%c%d.%d\t",Csign(setX),C1(setX),C2(setX));
	serial_output("SetY\t%c%d.%d\t",Csign(setY),C1(setY),C2(setY));
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
   serial_output("AC\t %c%d.%d\t cm,",Csign(actualclimbrate),C1(actualclimbrate),C2(actualclimbrate));
   serial_output(" EAC\t %c%d.%d\t cm,",Csign( ErrorClimbrate),C1( ErrorClimbrate),C2( ErrorClimbrate));


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

    serial_output("Radio1=%5d\t ",DutyCycle2_radio1);
    serial_output("\n");
    serialflag=0;
}

his_timercount=timercount;
}
return 0;
}

/* @brief  This function handles TIM4 global interrupt request.
* @param  None
* @retval None
*/
void TIM4_IRQHandler(void)
{

	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	 //togglebit^=1;
	//	  if(togglebit) GPIOD->BSRRL = 0xF000; // set PD1
		  	 // wait a short period of time
	//	  else GPIOD->BSRRH = 0xF000; // reset PD1

	//	TIM_ICInitTypeDef  TIM_ICInitStructure;


/* Clear TIM4 Capture compare interrupt pending bit */
TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);

/* Get the Input Capture value */
IC2Value = TIM_GetCapture2(TIM4);

if (IC2Value != 0)
{
  /* Duty cycle computation */
  //DutyCycle = (TIM_GetCapture1(TIM4) * 100) / IC2Value;
  DutyCycle=IC2Value;
  DutyCycle2=TIM_GetCapture1(TIM4);
  sensorheight=DutyCycle2/96.6;
  /* Frequency computation
     TIM4 counter clock = (RCC_Clocks.HCLK_Frequency)/2 */

  Frequency = (RCC_Clocks.HCLK_Frequency)/2 / IC2Value;
}
else
{
  DutyCycle = 0;
  Frequency = 0;
}
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void radio_in(){
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	 serial_output("Radio signal=\t%c%d.%d\t",Csign(DutyCycle2_radio),C1(DutyCycle2_radio),C2(DutyCycle2_radio));
        if(DutyCycle2_radio<20000)
        {
        	Rstate=ns_radio;
            switch(Rstate)
            {
				case 0:ns_radio= ns_radio+1;
				break;
				case 1:ns_radio= ns_radio+1;
								break;
				case 2:ns_radio= ns_radio+1;
								break;


				case 3:ns_radio= 0;
				Radio_status=0;

				break;
		    }
        }
	    else
	    {
			Radio_status=1;
			ns_radio=0;






         }


	    if((DutyCycle2_radio>20000) && (DutyCycle2_radio<40000))

	    {

         //  radioin=manualradio;

	   radioin=((DutyCycle2_radio-20000)/1.358)+8100;
	 //  radioin=10000.00;
	    /*
       	m2m4_rpm=(-0.0002996819 *(radioin)*(radioin)) +7.5004877*(radioin) -40913.2314;
                     		m1m3_rpm=(-0.0002740191*(radioin)*(radioin)) + 6.9107775322*(radioin) -37915.38407;
                     		average_rpm=(m1m3_rpm+m2m4_rpm)/2.0;
                     		m1Rcompensate=(average_rpm-m1m3_rpm)+average_rpm;
                     		m2Rcompensate=(average_rpm-m2m4_rpm)+average_rpm;
                     		m3Rcompensate=(average_rpm-m1m3_rpm)+average_rpm;
                     		m4Rcompensate=(average_rpm-m2m4_rpm)+average_rpm;



                     		M1Radio_in=((0.0001306929)*(m1Rcompensate*m1Rcompensate)) - (0.1127928036 *(m1Rcompensate)) + 8420.18897922;
                     		M3Radio_in=((0.0001306929)*(m3Rcompensate*m3Rcompensate)) - (0.1127928036 *(m3Rcompensate)) + 8420.18897922;

                     		M2Radio_in=(( 0.0001242078 )*(m2Rcompensate*m2Rcompensate)) - (0.1740771018*m2Rcompensate) + 8750.5685299;
                     		M4Radio_in=(( 0.0001242078 )*(m4Rcompensate*m4Rcompensate)) - (0.1740771018*m4Rcompensate) + 8750.5685299;



*/




	    }

	if(Radio_status==0){
		StartMotor=0;
		M1=0;
		M2=0;
		M3=0;
		M4=0;
		TIM_SetCompare1(TIM1, 8000);
						TIM_SetCompare2(TIM1, 8000);
						TIM_SetCompare3(TIM1, 8000);
						TIM_SetCompare4(TIM1, 8000);
						M1=8000;
						M2=8000;
						M3=8000;
						M4=8000;
		radioin=0;
		DutyCycle2_radio=0;

	}



	 if((timercount>5000) & (Radio_status==1)){
		 StartMotor=1;
	 }

	DutyCycle2_radio=0;

}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ControlLoop(){
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
          //--------Begin PID correction----------


/*



	 GyroXvalue=(MPU9150_readSensor(MPU9150_GYRO_XOUT_L,MPU9150_GYRO_XOUT_H));


     GyroYvalue=(MPU9150_readSensor(MPU9150_GYRO_YOUT_L,MPU9150_GYRO_YOUT_H));
     GyroZvalue=(MPU9150_readSensor(MPU9150_GYRO_ZOUT_L,MPU9150_GYRO_ZOUT_H));
     AccXvalue=((MPU9150_readSensor(MPU9150_ACCEL_XOUT_L,MPU9150_ACCEL_XOUT_H)));//to convert to 2nd complement -128 to 128 instead of 0-256
     AccYvalue=((MPU9150_readSensor(MPU9150_ACCEL_YOUT_L,MPU9150_ACCEL_YOUT_H)));//to convert to 2nd complement -128 to 128 instead of 0-256
     AccZvalue=((MPU9150_readSensor(MPU9150_ACCEL_ZOUT_L,MPU9150_ACCEL_ZOUT_H)));//to convert to 2

   //  GPIOD->BSRRL = 0xF000; // set PD1
     fuseGyroAcc(AccXvalue,AccYvalue,AccZvalue,GyroXvalue,GyroYvalue,GyroZvalue);
  //   GPIOD->BSRRH = 0xF000; // reset PD1


//--------- adding 3DM-GX1 as secondry IMU



     	//  GPIOD->BSRRL = 0xF000; // set PD1

*/



     //while(received_msg==0);
     /*
     for(sf=0;sf<24;sf++){
     	receivedmsg[sf]=0;
     }
     */



     /*
      *
      * The Roll and Yaw angles have a range of –32768 to +32767 representing –180 to +180 degrees.
      * The Pitch angle has a range of –16384 to +16383 representing –90 to +90 degrees.
      * To obtain angles in units of degrees, the integer outputs should be multiplied by the scaled factor (360/65536).
      *
      *
      *
      * Byte 2
     Roll MSB Byte 1
     Roll LSB Byte 2
     Pitch MSB Byte 3
     Pitch LSB Byte 4
     Yaw MSB Byte 5
     Yaw LSB Byte 6
      */


  //   GPIOD->BSRRH = 0xF000; // reset PD1



	float SDM_pitch,SDM_roll,SDM_raw;
if(CRCvalidation==0){
	SDM_pitch=DM_pitch;
	SDM_roll= DM_roll;
	SDM_raw=DM_raw;

	if((firstime==0)&(timercount>5200)){


		if((flightmode==0) & (StableMode==1) & (sensorheight>100)){
		setZ=SDM_raw;
        initialbearing=setZ;
        firstime=1;
		}




	}
}


		   ErrorX=setX-SDM_pitch;//Axr
		   ErrorY=setY-SDM_roll;
		   ErrorZ=setZ-SDM_raw;
/*

			if	((abs(ErrorX)>2)&(abs(ErrorX)<5)){					                     		       //}
								                     		 PgainX=10;
							                     		 Dgain=14.5;
			}
			if (abs(ErrorX)<2)

			{
				 PgainX=5;
			    Dgain=1;
	}
			if  (abs(ErrorX)>5) {
							  PgainX=11;
							  Dgain=18;

						}

*/


		   if((Radio_status==1)){


				   PID_Start=PID_Start+1;


							   if(PID_Start>30000){
								   PID_Start=10;

							   }

//----------------------compenstae unbalance CW and CCW -------------------------
								float check;
								/*
								check=(M1Radio_in+M3Radio_in+M2Radio_in+M4Radio_in)/4.0;
						                     		  if((check>MXlimit) ||  (check<MNlimit)){
						                     			 M1Radio_in=radioin;
						                     			 M2Radio_in=radioin;
						                     			 M3Radio_in=radioin;
						                     			 M4Radio_in=radioin;


						                     		       }
						                     		// if((radioin>12000)){

*/


						                     		  if(flightmode==0){

						                     								                     			M1Radio_in=radioin;
						                     								                     			M2Radio_in=radioin;
						                     								                     			M3Radio_in=radioin;
						                     								                     			M4Radio_in=radioin;


						                     		   }





//----------------------End xx compenstae unbalance CW and CCW -------------------------



								err_diffX=(ErrorX-PreviousErrX)/0.02;
								int_errX=int_errX + ErrorX;

//----------------------------------------------------------------

								p_termx=PgainX*ErrorX;  //2.4
								i_termx=IGain*int_errX;
								d_termx=(Dgain*err_diffX);
								pidx=p_termx+d_termx+i_termx;
								PreviousErrX=ErrorX;
//--------------------------------------------------------------
								err_diffY=ErrorY-PreviousErrY;
								int_errY=int_errY + ErrorY;
								p_termy=PGain*ErrorY;
								i_termy=IGain*int_errY;
								d_termy=Dgain*(err_diffY/0.02);
								pidy=p_termy+d_termy+i_termy;
								PreviousErrY=ErrorY;


//---------------------------------------------------------------


								err_diffZ=(ErrorZ-PreviousErrZ)/0.02;
								int_errZ=int_errZ + ErrorZ;

//----------------------------------------------------------------

								p_termz=PgainZ*ErrorZ;  //2.4
								i_termz=IgainZ*int_errZ;
								d_termz=(DgainZ*err_diffZ);
								pidz=p_termz+d_termz+i_termz;
								PreviousErrZ=ErrorZ;
//--------------------------------------------------------------



								 if((ErrorY<1) && (ErrorY>-1)) {int_errY=0;IntErrRateY=0; }
								 if((ErrorX<1) && (ErrorX>-1)) {int_errX=0; IntErrRateX=0;}
								 if((ErrorZ<1) && (ErrorZ>-1)) {int_errX=0; IntErrRateX=0;}
                                 if((abs(ErrorX) <3) & (abs(ErrorY) <3)){
                                	 StableMode=1;
                                 }
                                 else {
                                	 StableMode=0;
								}


									// if((flightmode==1)&(PreviousFlightMode!=flightmode)){   //Altitude Hold Mode just entered
									//	 setheight=sensorheight;
 									// }
									// PreviousFlightMode=flightmode;





//----------------- Temp disable to test Rate Gyro------------------------------------------------------------------------
						//RatePIDY
								    ErrRateY=pidy-DM_CompAngRateY;
								 	PtermRateY=	ErrRateY*RateYPG;
								 	DiffErrRateY=ErrRateY-PreviousErrRateY;
								 	DtermRateY=DiffErrRateY*RateYDG;
								 	IntErrRateY=IntErrRateY+ErrRateY;
								 	ItermRateY=RateYIG*IntErrRateY;
								 	PIDRateY=PtermRateY+DtermRateY+ItermRateY;
						   //End Rate PID Y

						  //	 RatePIDX
								 	ErrRateX=pidx-DM_CompAngRateX;
								 	PtermRateX=	ErrRateX*RateXPG;
								 	DiffErrRateX=ErrRateX-PreviousErrRateX;
								 	DtermRateX=DiffErrRateX*RateXDG;
								 	IntErrRateX=IntErrRateX+ErrRateX;
								 	ItermRateX=RateXIG*IntErrRateX;
								 	PIDRateX=PtermRateX+DtermRateX+ItermRateX;
						 //    End Rate PID XY


								 	  //	 RatePIDZ
								 	if(flightmode==0){
								 	ErrRateZ=pidz-DM_CompAngRateZ;
								 	RateZPG=1.2;
								 	}
								 	else{
								 		ErrRateZ=setZ-DM_CompAngRateZ;
								 		RateZPG=8;
								 	}
								 	PtermRateZ=	ErrRateZ*RateZPG;
								 	DiffErrRateZ=ErrRateZ-PreviousErrRateZ;
								 	DtermRateZ=DiffErrRateZ*RateZDG;
								 	IntErrRateZ=IntErrRateZ+ErrRateZ;
								 	ItermRateZ=RateZIG*IntErrRateZ;
								 	PIDRateZ=PtermRateZ+DtermRateZ+ItermRateZ;
								 							 //    End Rate PID XY





									 if(PIDoption==0){
									   M2= M2Radio_in-PIDRateZ;//;//+PIDRateX+PIDRateY
									   M1= M1Radio_in+PIDRateZ;//;//+PIDRateX-PIDRateY
                                      //---------------XASIS -----------------------------------
									   M3= M3Radio_in+PIDRateZ;//;//-PIDRateX+PIDRateY
									   M4= M4Radio_in-PIDRateZ;//;//-PIDRateX-PIDRateY
									 }
									 if(PIDoption==1){
																		   M2= M2Radio_in-pidz+pidx+pidy;
																		   M1= M1Radio_in+pidz+pidx-pidy;
									                                      //---------------XASIS -----------------------------------
																		   M3= M3Radio_in+pidz-pidx+pidy;
																		   M4= M4Radio_in-pidz-pidx-pidy;


																		   if(sensorheight>100){
																			   M2= M2-pidz;
																			   M1= M1+pidz;
																			   M3= M3+pidz;
																			   M4= M4-pidz;

																		   }


																		 }











//----------------- End Temp disable to test Rate Gyro------------------------------------------------------------------------
			   //--------End PID correction----------

	        }
               //------- if Radio is off ?------------

		    if(M1>(MXlimit)){ M1=MXlimit;}
		    if(M2>(MXlimit)){M2=MXlimit;}
			if(M3>(MXlimit)){M3=MXlimit;}
            if(M4>MXlimit){M4=MXlimit;}
            if(M1<MNlimit){M1=MNlimit;}
			if(M2<MNlimit){M2=MNlimit;}
			if(M3<MNlimit){M3=MNlimit;}
            if(M4<MNlimit){M4=MNlimit;}

/*
            if(flightmode==1){

            	 PreviousAlt_M1=M1;
            	 PreviousAlt_M2=M2;
            	 PreviousAlt_M3=M3;
            	 PreviousAlt_M4=M4;



            }

  */          //---Begin only  execute if StartMotor Flag =1;
            if((StartMotor==1)& (Radio_status==1))
            {

				TIM_SetCompare1(TIM1, M1); //M1
			 	TIM_SetCompare2(TIM1, M2); //M2
			 	TIM_SetCompare3(TIM1, M3); //M3
			 	TIM_SetCompare4(TIM1, M4); //M4



            //---End only  execute if StartMotor Flag =1;




}

            else

            {

            	TIM_SetCompare1(TIM1, 8000);
            		            	TIM_SetCompare2(TIM1, 8000);
            		            	TIM_SetCompare3(TIM1, 8000);
            		            	TIM_SetCompare4(TIM1, 8000);

            }



            if(sensorheight>700){

            	TIM_SetCompare1(TIM1, 12000);
            	            		            	TIM_SetCompare2(TIM1, 12000);
            	            		            	TIM_SetCompare3(TIM1, 12000);
            	            		            	TIM_SetCompare4(TIM1, 12000);
M1=8000;
M2=8000;
M3=8000;
M4=8000;
            	            		            	StartMotor=0;

            }
}
//--- End Control loop--------------------------------------------------------------------------
void TIM2_IRQHandler()
{
   if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        timercount=timercount+1;
        if(timercount>500000){

        	timercount=5000;
        }

        if(timercount%100==0){
        	serialflag=1;
        }

        if(timercount%400==0){

        	 ConvertedValue = adc_convert();//Read the ADC converted value
        	 Batter_V=(((ConvertedValue/4096.0)*2.96)-0.20)*10.0;


        	 if(flightmode==1){

if((sensorheight>40.00) ){

	PreviousAlt_M1=PreviousAlt_M2=PreviousAlt_M3=PreviousAlt_M4=12500;
	takeoff=1;

}

if((sensorheight<40.00) & (takeoff==0)){

	PreviousAlt_M1=PreviousAlt_M2=PreviousAlt_M3=PreviousAlt_M4=13000;


}
if(StableMode){

        							                     						ErrorH=setheight-sensorheight;
        							                     						err_diffH=(ErrorH-PreviousErrH)/0.4;
        							                     						int_errH=int_errH + ErrorH;
        							                     						p_termh=PGainH*ErrorH;  //2.4
        							                     						i_termh=IGainH*int_errH;
        							                     						d_termh=(DgainH*err_diffH);
        							                     						pidh=p_termh+d_termh+i_termh;
        							                     						PreviousErrH=ErrorH;

        							                     						//	 float setclimbrate=0.5,actualclimbrate=0;
        							                     						actualclimbrate=(sensorheight-PreviousHeight)/0.4;
        							                     						ErrorClimbrate=(ErrorH/8)-actualclimbrate;
        							                     						PidClimbRate=ErrorClimbrate* CRateGain;
        							                     						PreviousHeight=sensorheight;
        							                     						M1Radio_in=PreviousAlt_M1+PidClimbRate;
        							                     						M2Radio_in=PreviousAlt_M2+PidClimbRate;
        							                     						M3Radio_in=PreviousAlt_M3+PidClimbRate;
        							                     						M4Radio_in=PreviousAlt_M4+PidClimbRate;


}

        							                     																		 }
               }


        if((timercount%20==0) & (timercount>5000)){

        	  UART2_sendbyte(0x31);
              expect_received=1;


        }


        if(timercount%5==0){
        	conttrolflag=1;
        }

        if(timercount%20==0)
              {
        radio_in();



              	if(Radio_status==1){


              	//	m2m4_rpm=(0.0001*(radioin)*(radioin)) - 0.1741*(radioin) + 8610.6;
              	//	m1m3_rpm=(0.0001*(radioin)*(radioin)) -0.1128*(radioin) +8553.2;



              	}

              }

        if((timercount%10==0) & (timercount>5000)){

        }



        if(timercount==5000){
        	StartMotor=1;
                     }

        if(timercount%10000==0){
       //     	setY=-25;
               }


            //   if(timercount%30000==0){
              //               setX=0.0;
                //             }

               if(timercount%20000==0){
                         		//setY=0;
                             }


    }
}

void TIM3_IRQHandler(void)
{


	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
/* Clear TIM4 Capture compare interrupt pending bit */
TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);

/* Get the Input Capture value */
IC2Value_radio = TIM_GetCapture2(TIM3);

if (IC2Value_radio != 0)
{
  DutyCycle_radio=IC2Value_radio;
  DutyCycle2_radio=TIM_GetCapture1(TIM3);
  Frequency_radio = (RCC_Clocks.HCLK_Frequency)/2 / IC2Value_radio;
}
else
{
	DutyCycle_radio = 0;
	Frequency_radio = 0;
	DutyCycle2_radio=0;
}
//serial_output("PWMwidth=%5d\t",DutyCycle2_radio);



}


TIM8_CC_IRQHandler(void){

int 	IC2Value_radio1;

	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
/* Clear TIM4 Capture compare interrupt pending bit */
TIM_ClearITPendingBit(TIM8, TIM_IT_CC2);

/* Get the Input Capture value */
IC2Value_radioCh1= TIM_GetCapture2(TIM8);
//int IC2Value_radioCh1,DutyCycle_radio1,DutyCycle2_radio1,Frequency_radio1;
if (IC2Value_radioCh1 != 0)
{
	DutyCycle_radio1=IC2Value_radioCh1;
	DutyCycle2_radio1=TIM_GetCapture1(TIM8);
	Frequency_radio1 = (RCC_Clocks.HCLK_Frequency)/2 / IC2Value_radioCh1;
	if(DutyCycle2_radio1>24400){
		setY=(DutyCycle2_radio1-24400)/456;

	}
	else
		{
		setY=(-1)*(24400-DutyCycle2_radio1)/456;
		}




}
else
{
	DutyCycle_radio1 = 0;
	Frequency_radio1 = 0;
	DutyCycle2_radio1=0;
}
//serial_output("PWMwidth=%5d\t",DutyCycle2_radio);



}


TIM8_BRK_TIM12_IRQHandler(void){


	int 	IC2Value_radio2;

		RCC_ClocksTypeDef RCC_Clocks;
		RCC_GetClocksFreq(&RCC_Clocks);
	/* Clear TIM4 Capture compare interrupt pending bit */
	TIM_ClearITPendingBit(TIM12, TIM_IT_CC2);

	/* Get the Input Capture value */
	IC2Value_radioCh2= TIM_GetCapture2(TIM12);
	//int IC2Value_radioCh1,DutyCycle_radio1,DutyCycle2_radio1,Frequency_radio1;
	if (IC2Value_radioCh2 != 0)
	{
		DutyCycle_radio2=IC2Value_radioCh2;
		DutyCycle2_radio2=TIM_GetCapture1(TIM12);
		Frequency_radio2 = (RCC_Clocks.HCLK_Frequency)/2 / IC2Value_radioCh2;

		if(DutyCycle2_radio2>24400){
				setX=((-1)*(DutyCycle2_radio2-24400)/456)+xtrim;

			}
			else
				{
				setX=((24400-DutyCycle2_radio2)/456)+xtrim;
				}



	}
	else
	{
		DutyCycle_radio2 = 0;
		Frequency_radio2 = 0;
		DutyCycle2_radio2=0;
	}
}


TIM1_BRK_TIM9_IRQHandler(void){


	int 	IC2Value_radio2;

		RCC_ClocksTypeDef RCC_Clocks;
		RCC_GetClocksFreq(&RCC_Clocks);
	/* Clear TIM4 Capture compare interrupt pending bit */
	TIM_ClearITPendingBit(TIM9, TIM_IT_CC2);

	/* Get the Input Capture value */
	IC2Value_radioCh4= TIM_GetCapture2(TIM9);
	//int IC2Value_radioCh1,DutyCycle_radio1,DutyCycle2_radio1,Frequency_radio1;
	if (IC2Value_radioCh4 != 0)
	{
		DutyCycle_radio4=IC2Value_radioCh4;
		DutyCycle2_radio4=TIM_GetCapture1(TIM9);
		Frequency_radio4 = (RCC_Clocks.HCLK_Frequency)/2 / IC2Value_radioCh4;
if(rawmanualcontrol==1){

		if(DutyCycle2_radio4>24400){
		//	setZ=((DutyCycle2_radio4-24400)/76)+initialbearing;

			}
			else
				{
		//	setZ=((-1)*(24400-DutyCycle2_radio4)/76)+initialbearing;
				}
}

	}
	else
	{
		DutyCycle_radio4 = 0;
		Frequency_radio4 = 0;
		DutyCycle2_radio4=0;
	}
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void TIM5_IRQHandler(void)
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
{
//For Ch5 radio control input
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	 //togglebit^=1;
	//	  if(togglebit) GPIOD->BSRRL = 0xF000; // set PD1
		  	 // wait a short period of time
	//	  else GPIOD->BSRRH = 0xF000; // reset PD1

	//	TIM_ICInitTypeDef  TIM_ICInitStructure;


/* Clear TIM4 Capture compare interrupt pending bit */
TIM_ClearITPendingBit(TIM5, TIM_IT_CC2);

/* Get the Input Capture value */
IC2Value_radioCh5 = TIM_GetCapture2(TIM5);

if (IC2Value_radioCh5 != 0)
{
  /* Duty cycle computation */
  //DutyCycle = (TIM_GetCapture1(TIM4) * 100) / IC2Value;
  DutyCycle_radio5=IC2Value_radioCh5;
  DutyCycle2_radio5=TIM_GetCapture1(TIM5);




  /* Frequency computation
     TIM4 counter clock = (RCC_Clocks.HCLK_Frequency)/2 */

  Frequency_radio5 = (RCC_Clocks.HCLK_Frequency)/2 / IC2Value_radioCh5;

  if((DutyCycle2_radio5<3000) & (DutyCycle2_radio5>1700)){
	flightmode=0;
}

if((DutyCycle2_radio5<3200) & (DutyCycle2_radio5>3001)){
	flightmode=1;
}






}
else
{
	DutyCycle_radio5 = 0;
	Frequency_radio5 = 0;
	DutyCycle2_radio5=0;
}
}

/*
 *
 *  Debug logging ---
 *
 *
            if( (flightmode==1)){
        	   int k;
        	   	            	float sampletime=0;
        	   	            	  StartMotor=0;
        	   	            		            	TIM_SetCompare1(TIM1, 8100);
        	   	            		            	TIM_SetCompare2(TIM1, 8100);
        	   	            		            	TIM_SetCompare3(TIM1, 8100);
        	   	            		            	TIM_SetCompare4(TIM1, 8100);
        	   	            		            	TIM_Cmd(TIM2, DISABLE);
        	   	            		            	 for(k=0;k<500;k++){

        	   	            		            			                sampletime=k;
        	   	            		            			                serial_output("%d\t",k);
        	   	            		            			             Delay(5000);
        	   	            		            			                serial_output("Rin:\t%c%d.%d\t",Csign(Radioinx[k]),C1(Radioinx[k]),C2(Radioinx[k]));
        	   	            		            			             Delay(10000);
        	   	            		            			          serial_output("CurrentX:\t%c%d.%d\t",Csign(CurrentX[k]),C1(CurrentX[k]),C2(CurrentX[k]));
        	   	            		            			                 	   	            		            			             Delay(10000);


        	   	            		            			                serial_output("M1\t%c%d.%d\t",Csign(M1x[k]),C1(M1x[k]),C2(M1x[k]));
        	   	            		            			             Delay(10000);
        	   	            		            			                serial_output("M2\t%c%d.%d\t",Csign(M2x[k]),C1(M2x[k]),C2(M2x[k]));
        	   	            		            			             Delay(10000);
        	   	            		            			                serial_output("M3\t%c%d.%d\t",Csign(M3x[k]),C1(M3x[k]),C2(M3x[k]));
        	   	            		            			             Delay(10000);
        	   	            		            			                serial_output("M4\t%c%d.%d\t",Csign(M4x[k]),C1(M4x[k]),C2(M4x[k]));
        	   	            		            			             Delay(10000);
        	   	            		            				           serial_output("P\t%c%d.%d\t",Csign(Ptermx[k]),C1(Ptermx[k]),C2(Ptermx[k]));
        	   	            		            				        Delay(5000);
        	   	            		            				           serial_output("D\t%c%d.%d\t",Csign(Dtermx[k]),C1(Dtermx[k]),C2(Dtermx[k]));
        	   	            		            				        Delay(5000);
        	   	            		            				           serial_output("PID:\t\t%c%d.%d\t",Csign(PIDx[k]),C1(PIDx[k]),C2(PIDx[k]));
        	   	            		            				        Delay(5000);
        	   	            		            				     serial_output("SetX:\t\t%c%d.%d\t",Csign(setX),C1(setX),C2(setX));
        	   	            		            				          // serial_output("Set\t\t%c%d.%d\t",Csign(setX),C1(setX),C2(setX));
        	   	            		            				        Delay(5000);
        	   	            		            				     serial_output("PGain:\t\t%c%d.%d\t",Csign(PGain),C1(PGain),C2(PGain));
        	   	            		            				        //   serial_output("PG\t\t%c%d.%d\t",Csign(PGain),C1(PGain),C2(PGain));
        	   	            		            				        Delay(5000);
        	   	            		            				     //      serial_output("DG\t\t%c%d.%d\t",Csign(Dgain),C1(Dgain),C2(Dgain));
        	   	            		            				        serial_output("Dgain:\t\t%c%d.%d\t",Csign(Dgain),C1(Dgain),C2(Dgain));
        	   	            		            				        Delay(5000);
        	   	            		            				           serial_output("Err\t%c%d.%d\t",Csign( XErrbuffer[k]),C1(XErrbuffer[k]),C2(XErrbuffer[k]));

        	   	            		            				           serial_output("\n");

        	   	            			                }



           }

 *
 *
 *if(XErrbuf<Logbuf)
					{
						XErrbuffer[XErrbuf]=ErrorX;
						M1x[XErrbuf]=M1;
						M2x[XErrbuf]=M2;
						M3x[XErrbuf]=M3;
						M4x[XErrbuf]=M4;

						 Ptermx[XErrbuf]=p_termx;
						Dtermx[XErrbuf]=d_termx;
										 Itermx[XErrbuf]=i_termx;
										 PIDx[XErrbuf]=PIDRateX;
										 Radioinx[XErrbuf]=radioin;
										 CurrentX[XErrbuf]=Axr;
					XErrbuf=XErrbuf+1;
					}
 *
 *
 *
 * */


/*
 *
 */
