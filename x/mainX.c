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
int RX_indidate=0;

#include "uart.h"
#include "i2c.h"
#include "gpio.h"
#include "main.h"
#include "Global_variables.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "quaternion_supervisor.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"
/* Private typedef -----------------------------------------------------------*/
// Update Result here ---

//----ok now is to test out moving average and higher sampling---

// End update resilt here.---


float PitchAngle;
float RollAngle;
float RawAngle;

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
float PGain=10,PgainX=10,ErrorX=0,ErrorY=0,setX=0,setY=0,setheight,ErrorH=0,GH=0.0005;
float IGain=0,Dgain=14,err_diffX=0.0,err_diffY=0.0,int_errX=0.0,int_errY=0.0,PreviousErrX=0.0,PreviousErrY=0.0;
//--------------------------------------------- Rate PID ---------------------------------------------------------
float RateYPG=0.8,RateYDG=0,RateYIG=0,SetYRate=5;
float PreviousErrRateY,ErrRateY,DiffErrRateY,IntErrRateY,PtermRateY,DtermRateY,ItermRateY;
float PIDRateY;


float RateXPG=0.8,RateXDG=0,RateXIG=0.000,SetXRate=5;
float PreviousErrRateX,ErrRateX,DiffErrRateX,IntErrRateX,PtermRateX,DtermRateX,ItermRateX;
float PIDRateX,Axz,Ayz,RateAyzpast;


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
float MXlimit=13000;
#define MNlimit 8100
#define step 100

int IC2Value_radio, DutyCycle_radio,DutyCycle2_radio,Frequency_radio,curr_ch3,prev_ch3;
int IC2Value_radioCh5,DutyCycle_radio5,DutyCycle2_radio5,Frequency_radio5;
int IC2Value_radioCh9,DutyCycle_radio9,DutyCycle2_radio9,Frequency_radio9;
float sensorheight=0;


float PGainH=0.2,IGainH=0,DgainH=0;
float err_diffH,int_errH,p_termh,i_termh,d_termh,pidh,PreviousErrH;
int PreviousFlightMode=0,StableMode=0;
float 	RxAcc,RyAcc,RzAcc,RxGyro,RyGyro,RzGyro;
//Radio status state machine
int Radio_status=0;
int Rstate=0;
int ns_radio=0;
float radioin=0;
int tempradio_status=0;

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

float DM_roll,DM_pitch,DM_raw,DM_CompAngRateX,GyroGain,DM_CompAngRateY;
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
* This funcion initializes the USART3 peripheral
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
	// GPIOD->BSRRH = 0xF000; // reset PD1
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






//------------------------Add MPL ----------------------------------------------------------------

/**
 *   @defgroup  eMPL
 *   @brief     Embedded Motion Processing Library
 *
 *   @{
 *       @file      mllite_test.c
 *       @brief     Test app for eMPL using the Motion Driver DMP image.
 */

/* Includes ------------------------------------------------------------------*/


/* Data read from MPL. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_LINEAR_ACCEL (0x100)

volatile uint32_t hal_timestamp = 0;
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (100)// -- //20 change by zinbo

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
#define MPU9150
#define USE_DMP
#define EMPL


#define PEDO_READ_MS    (1000)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100)
struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};
struct hal_s hal = {0};

/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
 * because it's declared extern elsewhere.
 */
volatile unsigned char rx_new;

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
    signed char orientation[9];
};

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

#if defined MPU9150 || defined MPU9250
struct platform_data_s compass_pdata = {
    .orientation = { 0, 1, 0,
                     1, 0, 0,
                     0, 0, -1}
};
#define COMPASS_ENABLED 1
#elif defined AK8975_SECONDARY
struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0, 1, 0,
                     0, 0,-1}
};
#define COMPASS_ENABLED 1
#elif defined AK8963_SECONDARY
struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0,-1, 0,
                     0, 0, 1}
};
#define COMPASS_ENABLED 1
#endif


/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint32_t TimingDelay;
RCC_ClocksTypeDef RCC_Clocks;

/* Private function prototypes -----------------------------------------------*/
//void  RCC_Configuration(void);
void  Init_GPIOs (void);
void Delay(uint32_t nTime);
void platform_init(void);
int stm32l_get_clock_ms(unsigned long *count);
/* ---------------------------------------------------------------------------*/
/* Get data from MPL.
 * TODO: Add return values to the inv_get_sensor_type_xxx APIs to differentiate
 * between new and stale data.
 */
void read_from_mpl(void)
{
    long msg, data[9];
    int8_t accuracy;
    unsigned long timestamp;
    float float_data[3] = {0};

    if (inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&timestamp)) {
       /* Sends a quaternion packet to the PC. Since this is used by the Python
        * test app to visually represent a 3D quaternion, it's sent each time
        * the MPL has new data.
        */
        eMPL_send_quat(data);

        /* Specific data packets can be sent or suppressed using USB commands. */
        if (hal.report & PRINT_QUAT)
            eMPL_send_data(PACKET_DATA_QUAT, data);
    }

    if (hal.report & PRINT_ACCEL) {
        if (inv_get_sensor_type_accel(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_ACCEL, data);
    }
    if (hal.report & PRINT_GYRO) {
        if (inv_get_sensor_type_gyro(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_GYRO, data);
    }
#ifdef COMPASS_ENABLED
    if (hal.report & PRINT_COMPASS) {
        if (inv_get_sensor_type_compass(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_COMPASS, data);
    }
#endif
    if (hal.report & PRINT_EULER) {
        if (inv_get_sensor_type_euler(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_EULER, data);
    }
    if (hal.report & PRINT_ROT_MAT) {
        if (inv_get_sensor_type_rot_mat(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_ROT, data);
    }
    if (hal.report & PRINT_HEADING) {
        if (inv_get_sensor_type_heading(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_HEADING, data);
    }
    if (hal.report & PRINT_LINEAR_ACCEL) {
        if (inv_get_sensor_type_linear_acceleration(float_data, &accuracy, (inv_time_t*)&timestamp)) {
        	MPL_LOGI("Linear AccelX: %7.5f %7.5f %7.5f\r\n",
        			float_data[0], float_data[1], float_data[2]);
         }
    }
    if (hal.report & PRINT_PEDO) {
        unsigned long timestamp;
        stm32l_get_clock_ms(&timestamp);
        if (timestamp > hal.next_pedo_ms) {
            hal.next_pedo_ms = timestamp + PEDO_READ_MS;
            unsigned long step_count, walk_time;
            dmp_get_pedometer_step_count(&step_count);
            dmp_get_pedometer_walk_time(&walk_time);
            MPL_LOGI("Walked %ld steps over %ld milliseconds..\n", step_count,
            walk_time);
        }
    }

    /* Whenever the MPL detects a change in motion state, the application can
     * be notified. For this example, we use an LED to represent the current
     * motion state.
     */
    msg = inv_get_message_level_0(INV_MSG_MOTION_EVENT |
            INV_MSG_NO_MOTION_EVENT);
    if (msg) {
        if (msg & INV_MSG_MOTION_EVENT) {
            MPL_LOGI("Motion!\n");
        } else if (msg & INV_MSG_NO_MOTION_EVENT) {
            MPL_LOGI("No motion!\n");
        }
    }
}

#ifdef COMPASS_ENABLED
void send_status_compass() {
	long data[3] = { 0 };
	int8_t accuracy = { 0 };
	unsigned long timestamp;
	inv_get_compass_set(data, &accuracy, (inv_time_t*) &timestamp);
	MPL_LOGI("Compass: %7.4f %7.4f %7.4f ",
			data[0]/65536.f, data[1]/65536.f, data[2]/65536.f);
	MPL_LOGI("Accuracy= %d\r\n", accuracy);

}
#endif

/* Handle sensor on/off combinations. */
void setup_gyro(void)
{
    unsigned char mask = 0, lp_accel_was_on = 0;
    if (hal.sensors & ACCEL_ON)
        mask |= INV_XYZ_ACCEL;
    if (hal.sensors & GYRO_ON) {
        mask |= INV_XYZ_GYRO;
        lp_accel_was_on |= hal.lp_accel_mode;
    }
#ifdef COMPASS_ENABLED
    if (hal.sensors & COMPASS_ON) {
        mask |= INV_XYZ_COMPASS;
        lp_accel_was_on |= hal.lp_accel_mode;
    }
#endif
    /* If you need a power transition, this function should be called with a
     * mask of the sensors still enabled. The driver turns off any sensors
     * excluded from this mask.
     */
    mpu_set_sensors(mask);
    mpu_configure_fifo(mask);
    if (lp_accel_was_on) {
        unsigned short rate;
        hal.lp_accel_mode = 0;
        /* Switching out of LP accel, notify MPL of new accel sampling rate. */
        mpu_get_sample_rate(&rate);
        inv_set_accel_sample_rate(1000000L / rate);
    }
}

void tap_cb(unsigned char direction, unsigned char count)
{
    switch (direction) {
    case TAP_X_UP:
        MPL_LOGI("Tap X+ ");
        break;
    case TAP_X_DOWN:
        MPL_LOGI("Tap X- ");
        break;
    case TAP_Y_UP:
        MPL_LOGI("Tap Y+ ");
        break;
    case TAP_Y_DOWN:
        MPL_LOGI("Tap Y- ");
        break;
    case TAP_Z_UP:
        MPL_LOGI("Tap Z+ ");
        break;
    case TAP_Z_DOWN:
        MPL_LOGI("Tap Z- ");
        break;
    default:
        return;
    }
    MPL_LOGI("x%d\n", count);
    return;
}

void android_orient_cb(unsigned char orientation)
{
	switch (orientation) {
	case ANDROID_ORIENT_PORTRAIT:
        MPL_LOGI("Portrait\n");
        break;
	case ANDROID_ORIENT_LANDSCAPE:
        MPL_LOGI("Landscape\n");
        break;
	case ANDROID_ORIENT_REVERSE_PORTRAIT:
        MPL_LOGI("Reverse Portrait\n");
        break;
	case ANDROID_ORIENT_REVERSE_LANDSCAPE:
        MPL_LOGI("Reverse Landscape\n");
        break;
	default:
		return;
	}
}


inline void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

#if defined (MPU6500) || defined (MPU9250)
    result = mpu_run_6500_self_test(gyro, accel, 0);
#elif defined (MPU6050) || defined (MPU9150)
    result = mpu_run_self_test(gyro, accel);
#endif
    if (result == 0x7) {
	MPL_LOGI("Passed!\n");
        MPL_LOGI("accelXX: %7.4f %7.4f %7.4f\n",
                    accel[0]/65536.f,
                    accel[1]/65536.f,
                    accel[2]/65536.f);
        MPL_LOGI("gyro: %7.4f %7.4f %7.4f\n",
                    gyro[0]/65536.f,
                    gyro[1]/65536.f,
                    gyro[2]/65536.f);
        /* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

#ifdef USE_CAL_HW_REGISTERS
        /*
         * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
         * instead of pushing the cal data to the MPL software library
         */
        unsigned char i = 0;

        for(i = 0; i<3; i++) {
        	gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
        	accel[i] *= 4096.f; //convert to +-8G
        	accel[i] = accel[i] >> 16;
        	gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
        mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
        mpu_set_accel_bias_6050_reg(accel);
#endif
#else
        /* Push the calibrated data to the MPL library.
         *
         * MPL expects biases in hardware units << 16, but self test returns
		 * biases in g's << 16.
		 */
    	unsigned short accel_sens;
    	float gyro_sens;

		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		inv_set_accel_bias(accel, 3);
		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long) (gyro[0] * gyro_sens);
		gyro[1] = (long) (gyro[1] * gyro_sens);
		gyro[2] = (long) (gyro[2] * gyro_sens);
		inv_set_gyro_bias(gyro, 3);
#endif
    }
    else {
            if (!(result & 0x1))
                MPL_LOGE("Gyro failed.\n");
            if (!(result & 0x2))
                MPL_LOGE("Accel failed.\n");
            if (!(result & 0x4))
                MPL_LOGE("Compass failed.\n");
     }

}

void handle_input(void)
{

    char c = USART_ReceiveData(USART3);

    switch (c) {
    /* These commands turn off individual sensors. */
    case '8':
        hal.sensors ^= ACCEL_ON;
        setup_gyro();
        if (!(hal.sensors & ACCEL_ON))
            inv_accel_was_turned_off();
        break;
    case '9':
        hal.sensors ^= GYRO_ON;
        setup_gyro();
        if (!(hal.sensors & GYRO_ON))
            inv_gyro_was_turned_off();
        break;
#ifdef COMPASS_ENABLED
    case '0':
        hal.sensors ^= COMPASS_ON;
        setup_gyro();
        if (!(hal.sensors & COMPASS_ON))
            inv_compass_was_turned_off();
        break;
#endif
    /* The commands send individual sensor data or fused data to the PC. */
    case 'a':
        hal.report ^= PRINT_ACCEL;
        break;
    case 'g':
        hal.report ^= PRINT_GYRO;
        break;
#ifdef COMPASS_ENABLED
    case 'c':
        hal.report ^= PRINT_COMPASS;
        break;
#endif
    case 'e':
        hal.report ^= PRINT_EULER;
        break;
    case 'r':
        hal.report ^= PRINT_ROT_MAT;
        break;
    case 'q':
        hal.report ^= PRINT_QUAT;
        break;
    case 'h':
        hal.report ^= PRINT_HEADING;
        break;
    case 'i':
        hal.report ^= PRINT_LINEAR_ACCEL;
        break;
#ifdef COMPASS_ENABLED
	case 'w':
		send_status_compass();
		break;
#endif
    /* This command prints out the value of each gyro register for debugging.
     * If logging is disabled, this function has no effect.
     */
    case 'd':
        mpu_reg_dump();
        break;
    /* Test out low-power accel mode. */
    case 'p':
        if (hal.dmp_on)
            /* LP accel is not compatible with the DMP. */
            break;
        mpu_lp_accel_mode(20);
        /* When LP accel mode is enabled, the driver automatically configures
         * the hardware for latched interrupts. However, the MCU sometimes
         * misses the rising/falling edge, and the hal.new_gyro flag is never
         * set. To avoid getting locked in this state, we're overriding the
         * driver's configuration and sticking to unlatched interrupt mode.
         *
         * TODO: The MCU supports level-triggered interrupts.
         */
        mpu_set_int_latched(0);
        hal.sensors &= ~(GYRO_ON|COMPASS_ON);
        hal.sensors |= ACCEL_ON;
        hal.lp_accel_mode = 1;
        inv_gyro_was_turned_off();
        inv_compass_was_turned_off();
        break;
    /* This snippet of code shows how to load and store calibration data from
     * the MPL. The original code is intended for a MSP430, the flash segment
     * must be unlocked before reading/writing and locked when no longer in use.
     * When porting to a different microcontroller, flash memory might be
     * accessible at anytime, or may not be available at all.
     */
    /*case 'l':
        inv_get_mpl_state_size(&store_size);
        if (store_size > FLASH_SIZE) {
            MPL_LOGE("Calibration data exceeds available memory.\n");
            break;
        }
        FCTL3 = FWKEY;
        inv_load_mpl_states(FLASH_MEM_START, store_size);
        FCTL3 = FWKEY + LOCK;
        inv_accel_was_turned_off();
        inv_gyro_was_turned_off();
        inv_compass_was_turned_off();
        break;
    case 's':
        inv_get_mpl_state_size(&store_size);
        if (store_size > FLASH_SIZE) {
            MPL_LOGE("Calibration data exceeds available memory.\n");
            return;
        } else {
            unsigned char mpl_states[100], tries = 5, erase_result;
            inv_save_mpl_states(mpl_states, store_size);
            while (tries--) {
                // Multiple attempts to erase current data.
                Flash_SegmentErase((uint16_t*)FLASH_MEM_START);
                erase_result = Flash_EraseCheck((uint16_t*)FLASH_MEM_START,
                    store_size>>1);
                if (erase_result == FLASH_STATUS_OK)
                    break;
            }
            if (erase_result == FLASH_STATUS_ERROR) {
                MPL_LOGE("Could not erase user page for calibration "
                    "storage.\n");
                break;
            }
            FlashWrite_8(mpl_states, FLASH_MEM_START, store_size);
        }
        inv_accel_was_turned_off();
        inv_gyro_was_turned_off();
        inv_compass_was_turned_off();
        break;*/
    /* The hardware self test can be run without any interaction with the
     * MPL since it's completely localized in the gyro driver. Logging is
     * assumed to be enabled; otherwise, a couple LEDs could probably be used
     * here to display the test results.
     */
    case 't':
        run_self_test();
        /* Let MPL know that contiguity was broken. */
        inv_accel_was_turned_off();
        inv_gyro_was_turned_off();
        inv_compass_was_turned_off();
        break;
    /* Depending on your application, sensor data may be needed at a faster or
     * slower rate. These commands can speed up or slow down the rate at which
     * the sensor data is pushed to the MPL.
     *
     * In this example, the compass rate is never changed.
     */
    case '1':
        if (hal.dmp_on) {
            dmp_set_fifo_rate(10);
            inv_set_quat_sample_rate(100000L);
        } else
            mpu_set_sample_rate(10);
        inv_set_gyro_sample_rate(100000L);
        inv_set_accel_sample_rate(100000L);
        break;
    case '2':
        if (hal.dmp_on) {
            dmp_set_fifo_rate(20);
            inv_set_quat_sample_rate(50000L);
        } else
            mpu_set_sample_rate(20);
        inv_set_gyro_sample_rate(50000L);
        inv_set_accel_sample_rate(50000L);
        break;
    case '3':
        if (hal.dmp_on) {
            dmp_set_fifo_rate(40);
            inv_set_quat_sample_rate(25000L);
        } else
            mpu_set_sample_rate(40);
        inv_set_gyro_sample_rate(25000L);
        inv_set_accel_sample_rate(25000L);
        break;
    case '4':
        if (hal.dmp_on) {
            dmp_set_fifo_rate(50);
            inv_set_quat_sample_rate(20000L);
        } else
            mpu_set_sample_rate(50);
        inv_set_gyro_sample_rate(20000L);
        inv_set_accel_sample_rate(20000L);
        break;
    case '5':
        if (hal.dmp_on) {
            dmp_set_fifo_rate(100);
            inv_set_quat_sample_rate(10000L);
        } else
            mpu_set_sample_rate(100);
        inv_set_gyro_sample_rate(10000L);
        inv_set_accel_sample_rate(10000L);
        break;
	case ',':
        /* Set hardware to interrupt on gesture event only. This feature is
         * useful for keeping the MCU asleep until the DMP detects as a tap or
         * orientation event.
         */
        dmp_set_interrupt_mode(DMP_INT_GESTURE);
        break;
    case '.':
        /* Set hardware to interrupt periodically. */
        dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
        break;
    case '6':
        /* Toggle pedometer display. */
        hal.report ^= PRINT_PEDO;
        break;
    case '7':
        /* Reset pedometer. */
        dmp_set_pedometer_step_count(0);
        dmp_set_pedometer_walk_time(0);
        break;
    case 'f':
        if (hal.lp_accel_mode)
            /* LP accel is not compatible with the DMP. */
            return;
        /* Toggle DMP. */
        if (hal.dmp_on) {
            unsigned short dmp_rate;
            unsigned char mask = 0;
            hal.dmp_on = 0;
            mpu_set_dmp_state(0);
            /* Restore FIFO settings. */
            if (hal.sensors & ACCEL_ON)
                mask |= INV_XYZ_ACCEL;
            if (hal.sensors & GYRO_ON)
                mask |= INV_XYZ_GYRO;
            if (hal.sensors & COMPASS_ON)
                mask |= INV_XYZ_COMPASS;
            mpu_configure_fifo(mask);
            /* When the DMP is used, the hardware sampling rate is fixed at
             * 200Hz, and the DMP is configured to downsample the FIFO output
             * using the function dmp_set_fifo_rate. However, when the DMP is
             * turned off, the sampling rate remains at 200Hz. This could be
             * handled in inv_mpu.c, but it would need to know that
             * inv_mpu_dmp_motion_driver.c exists. To avoid this, we'll just
             * put the extra logic in the application layer.
             */
            dmp_get_fifo_rate(&dmp_rate);
            mpu_set_sample_rate(dmp_rate);
            inv_quaternion_sensor_was_turned_off();
            MPL_LOGI("DMP disabled.\n");
        } else {
            unsigned short sample_rate;
            hal.dmp_on = 1;
            /* Preserve current FIFO rate. */
            mpu_get_sample_rate(&sample_rate);
            dmp_set_fifo_rate(sample_rate);
            inv_set_quat_sample_rate(1000000L / sample_rate);
            mpu_set_dmp_state(1);
            MPL_LOGI("DMP enabled.\n");
        }
        break;
    case 'm':
        /* Test the motion interrupt hardware feature. */
		#ifndef MPU6050 // not enabled for 6050 product
		hal.motion_int_mode = 1;
		#endif
        break;

    case 'v':
        /* Toggle LP quaternion.
         * The DMP features can be enabled/disabled at runtime. Use this same
         * approach for other features.
         */
        hal.dmp_features ^= DMP_FEATURE_6X_LP_QUAT;
        dmp_enable_feature(hal.dmp_features);
        if (!(hal.dmp_features & DMP_FEATURE_6X_LP_QUAT)) {
            inv_quaternion_sensor_was_turned_off();
            MPL_LOGI("LP quaternion disabled.\n");
        } else
            MPL_LOGI("LP quaternion enabled.\n");
        break;
    default:
        break;
    }
    hal.rx.cmd = 0;
}

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */

void USART3_IRQHandler(void){



if (USART_GetITStatus(USART3, USART_IT_RXNE)) {
       /* A byte has been received via USART. See handle_input for a list of
        * valid commands.
        */
       USART_ClearITPendingBit(USART3, USART_IT_RXNE);
       RX_indidate=1;

   }
}


void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}
/*******************************************************************************/

/**
  * @brief main entry point.
  * @par Parameters None
  * @retval void None
  * @par Required preconditions: None
  */

int main(void)
{



  inv_error_t result;
    unsigned char accel_fsr,  new_temp = 0;
    unsigned short gyro_rate, gyro_fsr;
    unsigned long timestamp;
    struct int_param_s int_param;

#ifdef COMPASS_ENABLED
    unsigned char new_compass = 0;
    unsigned short compass_fsr;
#endif
  platform_init();
  MPL_LOGE("This is it.\n");
  result = mpu_init(&int_param);
  if (result) {
      MPL_LOGE("Could not initialize gyro.\n");
  }


    /* If you're not using an MPU9150 AND you're not using DMP features, this
     * function will place all slaves on the primary bus.
     * mpu_set_bypass(1);
     */

  result = inv_init_mpl();
  if (result) {
      MPL_LOGE("Could not initialize MPL.\n");
  }

    /* Compute 6-axis and 9-axis quaternions. */
    inv_enable_quaternion();
    inv_enable_9x_sensor_fusion();
    /* The MPL expects compass data at a constant rate (matching the rate
     * passed to inv_set_compass_sample_rate). If this is an issue for your
     * application, call this function, and the MPL will depend on the
     * timestamps passed to inv_build_compass instead.
     *
     * inv_9x_fusion_use_timestamps(1);
     */

    /* This function has been deprecated.
     * inv_enable_no_gyro_fusion();
     */

    /* Update gyro biases when not in motion.
     * WARNING: These algorithms are mutually exclusive.
     */
    inv_enable_fast_nomot();
    /* inv_enable_motion_no_motion(); */
    /* inv_set_no_motion_time(1000); */

    /* Update gyro biases when temperature changes. */
    inv_enable_gyro_tc();

    /* This algorithm updates the accel biases when in motion. A more accurate
     * bias measurement can be made when running the self-test (see case 't' in
     * handle_input), but this algorithm can be enabled if the self-test can't
     * be executed in your application.
     *
     * inv_enable_in_use_auto_calibration();
     */
#ifdef COMPASS_ENABLED
    /* Compass calibration algorithms. */
    inv_enable_vector_compass_cal();
    inv_enable_magnetic_disturbance();
#endif
    /* If you need to estimate your heading before the compass is calibrated,
     * enable this algorithm. It becomes useless after a good figure-eight is
     * detected, so we'll just leave it out to save memory.
     * inv_enable_heading_from_gyro();
     */

    /* Allows use of the MPL APIs in read_from_mpl. */
    inv_enable_eMPL_outputs();

  result = inv_start_mpl();
  if (result == INV_ERROR_NOT_AUTHORIZED) {
      while (1) {
          MPL_LOGE("Not authorized.\n");
      }
  }
  if (result) {
      MPL_LOGE("Could not start the MPL.\n");
  }

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
#ifdef COMPASS_ENABLED
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
#else
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
#endif
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
#ifdef COMPASS_ENABLED
    /* The compass sampling rate can be less than the gyro/accel sampling rate.
     * Use this function for proper power management.
     */
    mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
#endif
    /* Read back configuration in case it was set improperly. */
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
#ifdef COMPASS_ENABLED
    mpu_get_compass_fsr(&compass_fsr);
#endif
    /* Sync driver configuration with MPL. */
    /* Sample rate expected in microseconds. */
    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);
#ifdef COMPASS_ENABLED
    /* The compass rate is independent of the gyro and accel rates. As long as
     * inv_set_compass_sample_rate is called with the correct value, the 9-axis
     * fusion algorithm's compass correction gain will work properly.
     */
    inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
#endif
    /* Set chip-to-body orientation matrix.
     * Set hardware units to dps/g's/degrees scaling factor.
     */
    inv_set_gyro_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)gyro_fsr<<15);
    inv_set_accel_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)accel_fsr<<15);
#ifdef COMPASS_ENABLED
    inv_set_compass_orientation_and_scale(
            inv_orientation_matrix_to_scalar(compass_pdata.orientation),
            (long)compass_fsr<<15);
#endif
    /* Initialize HAL state variables. */
#ifdef COMPASS_ENABLED
    hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
#else
    hal.sensors = ACCEL_ON | GYRO_ON;
#endif
    hal.dmp_on = 0;
    hal.report = 0;
    hal.rx.cmd = 0;
    hal.next_pedo_ms = 0;
    hal.next_compass_ms = 0;
    hal.next_temp_ms = 0;

  /* Compass reads are handled by scheduler. */
  stm32l_get_clock_ms(&timestamp);

    /* To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     * 2. Push the gyro and accel orientation matrix to the DMP.
     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.
     * 4. Call dmp_enable_feature(mask) to enable different features.
     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
     * 6. Call any feature-specific control functions.
     *
     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
     * 200Hz. Integrating the gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */
    dmp_load_motion_driver_firmware();
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
    dmp_register_tap_cb(tap_cb);
    dmp_register_android_orient_cb(android_orient_cb);
    /*
     * Known Bug -
     * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
     * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
     * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
     * there will be a 25Hz interrupt from the MPU device.
     *
     * There is a known issue in which if you do not enable DMP_FEATURE_TAP
     * then the interrupts will be at 200Hz even if fifo rate
     * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
     *
     * DMP sensor fusion works only with gyro at +-2000dps and accel +-2G
     */
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
   // dmp_enable_6x_lp_quat(1);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;

  while(1){


	   GPIOD->BSRRL = 0xF000; // set PD1



    unsigned long sensor_timestamp;
    int new_data = 0;
   // MPL_LOGI("ZBN MPU 9150..\n");
if(RX_indidate==1){

	 handle_input();
	 RX_indidate=0;
}


    stm32l_get_clock_ms(&timestamp);

#ifdef COMPASS_ENABLED
        /* We're not using a data ready interrupt for the compass, so we'll
         * make our compass reads timer-based instead.
         */
        if ((timestamp > hal.next_compass_ms) && !hal.lp_accel_mode &&
            hal.new_gyro && (hal.sensors & COMPASS_ON)) {
            hal.next_compass_ms = timestamp + COMPASS_READ_MS;
            new_compass = 1;
        }
#endif
        /* Temperature data doesn't need to be read with every gyro sample.
         * Let's make them timer-based like the compass reads.
         */
        if (timestamp > hal.next_temp_ms) {
            hal.next_temp_ms = timestamp + TEMP_READ_MS;
            new_temp = 1;
        }

    if (hal.motion_int_mode) {
        /* Enable motion interrupt. */
        mpu_lp_motion_interrupt(500, 1, 5);
        /* Notify the MPL that contiguity was broken. */
        inv_accel_was_turned_off();
        inv_gyro_was_turned_off();
        inv_compass_was_turned_off();
        inv_quaternion_sensor_was_turned_off();
        /* Wait for the MPU interrupt. */
        while (!hal.new_gyro) {}
        /* Restore the previous sensor configuration. */
        mpu_lp_motion_interrupt(0, 0, 0);
        hal.motion_int_mode = 0;
    }

    if (!hal.sensors || !hal.new_gyro) {
        continue;
    }

        if (hal.new_gyro && hal.lp_accel_mode) {
            short accel_short[3];
            long accel[3];
            mpu_get_accel_reg(accel_short, &sensor_timestamp);
            accel[0] = (long)accel_short[0];
            accel[1] = (long)accel_short[1];
            accel[2] = (long)accel_short[2];
            inv_build_accel(accel, 0, sensor_timestamp);

        } else if (hal.new_gyro && hal.dmp_on) {
            short gyro[3], accel_short[3], sensors;
            unsigned char more;
            long accel[3], quat[4], temperature;
            /* This function gets new data from the FIFO when the DMP is in
             * use. The FIFO can contain any combination of gyro, accel,
             * quaternion, and gesture data. The sensors parameter tells the
             * caller which data fields were actually populated with new data.
             * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
             * the FIFO isn't being filled with accel data.
             * The driver parses the gesture data to determine if a gesture
             * event has occurred; on an event, the application will be notified
             * via a callback (assuming that a callback function was properly
             * registered). The more parameter is non-zero if there are
             * leftover packets in the FIFO.
             */
            dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);


            if (!more)
                hal.new_gyro = 0;
            if (sensors & INV_XYZ_GYRO) {
                /* Push the new data to the MPL. */
                inv_build_gyro(gyro, sensor_timestamp);
                new_data = 1;
                if (new_temp) {
                    new_temp = 0;
                    /* Temperature only used for gyro temp comp. */
                    mpu_get_temperature(&temperature, &sensor_timestamp);
                    inv_build_temp(temperature, sensor_timestamp);
                }
            }
            if (sensors & INV_XYZ_ACCEL) {
                accel[0] = (long)accel_short[0];
                accel[1] = (long)accel_short[1];
                accel[2] = (long)accel_short[2];
                inv_build_accel(accel, 0, sensor_timestamp);
                new_data = 1;
            }
            if (sensors & INV_WXYZ_QUAT) {
                inv_build_quat(quat, 0, sensor_timestamp);
                new_data = 1;
            }
        } else if (hal.new_gyro) {
            short gyro[3], accel_short[3];
            unsigned char sensors, more;
            long accel[3], temperature;
            /* This function gets new data from the FIFO. The FIFO can contain
             * gyro, accel, both, or neither. The sensors parameter tells the
             * caller which data fields were actually populated with new data.
             * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
             * being filled with accel data. The more parameter is non-zero if
             * there are leftover packets in the FIFO. The HAL can use this
             * information to increase the frequency at which this function is
             * called.
             */
            hal.new_gyro = 0;
            mpu_read_fifo(gyro, accel_short, &sensor_timestamp,
                &sensors, &more);
            if (more)
                hal.new_gyro = 1;
            if (sensors & INV_XYZ_GYRO) {
                /* Push the new data to the MPL. */
                inv_build_gyro(gyro, sensor_timestamp);
                new_data = 1;
                if (new_temp) {
                    new_temp = 0;
                    /* Temperature only used for gyro temp comp. */
                    mpu_get_temperature(&temperature, &sensor_timestamp);
                    inv_build_temp(temperature, sensor_timestamp);
                }
            }
            if (sensors & INV_XYZ_ACCEL) {
                accel[0] = (long)accel_short[0];
                accel[1] = (long)accel_short[1];
                accel[2] = (long)accel_short[2];
                inv_build_accel(accel, 0, sensor_timestamp);
                new_data = 1;
            }
        }
#ifdef COMPASS_ENABLED
        if (new_compass) {
            short compass_short[3];
            long compass[3];
            new_compass = 0;
            /* For any MPU device with an AKM on the auxiliary I2C bus, the raw
             * magnetometer registers are copied to special gyro registers.
             */
            if (!mpu_get_compass_reg(compass_short, &sensor_timestamp)) {
                compass[0] = (long)compass_short[0];
                compass[1] = (long)compass_short[1];
                compass[2] = (long)compass_short[2];
                /* NOTE: If using a third-party compass calibration library,
                 * pass in the compass data in uT * 2^16 and set the second
                 * parameter to INV_CALIBRATED | acc, where acc is the
                 * accuracy from 0 to 3.
                 */
                inv_build_compass(compass, 0, sensor_timestamp);
            }
            new_data = 1;
        }
#endif
        if (new_data) {
            inv_execute_on_data();
            /* This function reads bias-compensated sensor data and sensor
             * fusion outputs from the MPL. The outputs are formatted as seen
             * in eMPL_outputs.c. This function only needs to be called at the
             * rate requested by the host.
             */
         read_from_mpl();
       //  inv_execute_on_data();
         long msg, data[9];
           int8_t accuracy;
           unsigned long timestamp;
           float float_data[3] = {0};
           inv_get_sensor_type_euler(data, &accuracy,(inv_time_t*)&timestamp);
     //    inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&timestamp);
       //  MPL_LOGI("Assigned Eular Angle P=%.2f Roll=%.2f Raw=%.2f\n",PitchAngle,RollAngle,RawAngle);
    //     MPL_LOGI("accel: %7.4f %7.4f %7.4f\n",
        //		 PitchAngle,
        //		 RollAngle,
        //		 RawAngle);
int a=11;
//serial_output("M3\t%c%d.%d\t",Csign(M3x[k]),C1(M3x[k]),C2(M3x[k]));
        MPL_LOGI("Pitch: %c%d.%d\n",Csign(PitchAngle),C1(PitchAngle),C2(PitchAngle));
        MPL_LOGI("Roll: %c%d.%d\n",Csign(RollAngle),C1(RollAngle),C2(RollAngle));
        MPL_LOGI("RAW: %c%d.%d\n",Csign(RawAngle),C1(RawAngle),C2(RawAngle));

              GPIOD->BSRRH = 0xF000; // reset PD1
            Delay(100);


        }
    }
}

/*---------------------------------------------------------------------------*/

/**
 * Configure the hardware of the Discovery board to link with the MPU
 */
void platform_init(void)
{

	  /* Configure Clocks for Application need */
	  RCC_Configuration();

	  /* Configure SysTick IRQ and SysTick Timer to generate interrupts every 1ms */
	RCC_ClocksTypeDef RCC_Clocks;
		RCC_GetClocksFreq(&RCC_Clocks);
	//  RCC_GetClocksFreq(&RCC_Clocks);
	  SysTick_Config(RCC_Clocks.HCLK_Frequency / 2000);
	  //SysTick_CLKSourceConfig(RCC_Clocks.HCLK_Frequency / 2000);

  Init_GPIOs();  //Initialize the I2C, UART, Intterupts, and the Green and Blue LEDs
//  init_USART3(9600);
//  	init_USART2(38400);
      init_GPIO();

}

void SysTick_Handler(void)
{


    TimingDelay_Decrement();
    TimeStamp_Increment();


}
/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{

  /* Enable HSI Clock */
  RCC_HSICmd(ENABLE);

  /*!< Wait till HSI is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET)
  {}

  RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);

  //RCC_MSIRangeConfig(RCC_MSIRange_6);

  RCC_HSEConfig(RCC_HSE_OFF);
  if(RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET )
  {
    while(1);
  }

  /* Enable  comparator clock LCD and PWR mngt */
  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_LCD | RCC_APB1Periph_PWR, ENABLE);

  /* Enable ADC clock & SYSCFG */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_SYSCFG, ENABLE);

}


void  Init_GPIOs (void)
{
  //GPIO_InitTypeDef GPIO_InitStructure;



  //Configure I2C
  I2C_Config();

  //Configure Interrupts
  GPIO_Config();

  //Configure UART
  USART_Config();

/* Configure the GPIO_LED pins  LD3 & LD4*/
  /*GPIO_InitStructure.GPIO_Pin = LD_GREEN_GPIO_PIN | LD_BLUE_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(LD_GPIO_PORT, &GPIO_InitStructure);
  GPIO_LOW(LD_GPIO_PORT, LD_GREEN_GPIO_PIN);
  GPIO_LOW(LD_GPIO_PORT, LD_BLUE_GPIO_PIN);*/

}


/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
  * @retval None
  */
void Delay(uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);

}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{

  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }

}

void TimeStamp_Increment(void)
{
  hal_timestamp++;
}

int stm32l_get_clock_ms(unsigned long *count)
{
    if (!count)
        return 1;
    count[0] = hal_timestamp;
    return 0;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {


  }
}

#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/



//-----------------------End Add MPL-------------------------------------------------------------




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







//----------- End adding  IMU


	float SDM_pitch,SDM_roll;
if(CRCvalidation==0){
	SDM_pitch=DM_pitch;
	SDM_roll= DM_roll;

if(DM_pitch<-3){
								int kg=0;
								kg=kg+1;

					        }
}


		   ErrorX=setX-SDM_pitch;//Axr
		   ErrorY=setY-SDM_roll;
		 //  ErrorH=setheight-DutyCycle2;

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

		   if((Radio_status==1)){


				   PID_Start=PID_Start+1;


							   if(PID_Start>30000){
								   PID_Start=10;

							   }

//----------------------compenstae unbalance CW and CCW -------------------------
								float check;
								check=(M1Radio_in+M3Radio_in+M2Radio_in+M4Radio_in)/4.0;
						                     		  if((check>MXlimit) ||  (check<MNlimit)){
						                     			 M1Radio_in=radioin;
						                     			 M2Radio_in=radioin;
						                     			 M3Radio_in=radioin;
						                     			 M4Radio_in=radioin;


						                     		       }
						                     		// if((radioin>12000)){
						                     								                     			M1Radio_in=radioin;
						                     								                     			M2Radio_in=radioin;
						                     								                     			M3Radio_in=radioin;
						                     								                     			M4Radio_in=radioin;


						                     								                     		       //}



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


							//	if(((ErrorY<2) && (ErrorY>-2)) && ((ErrorX<2) && (ErrorX>-2))){
								if( ErrorX==0){
																	 StableMode=1;

																 }
																 else
																 {
																	 StableMode=0;
																 }


								 if((ErrorY<2) && (ErrorY>-2)) {int_errY=0;IntErrRateY=0; }
								 if((ErrorX<2) && (ErrorX>-2)) {int_errX=0; IntErrRateX=0;}

								 if(StableMode==1){

									 if((flightmode==1)&(PreviousFlightMode!=flightmode)){   //Altitude Hold Mode just entered
										 setheight=sensorheight;
 									 }
									 PreviousFlightMode=flightmode;

								 }




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


						//		M2=radioin+PIDRateY;//;//+(GH*ErrorH); -pidx
						//		M3=radioin+PIDRateY;//;//+(GH*ErrorH);+(GH*ErrorH); -pidx


						//		M1=radioin-PIDRateY;//;//+(GH*ErrorH);+(GH*ErrorH); +pidx
						//		M4=radioin-PIDRateY;//;//+(GH*ErrorH);+(GH*ErrorH); +pidx
								 if(flightmode==0){


									 if(PIDoption==0){
									   M2= M2Radio_in+PIDRateX+PIDRateY;//;//+PIDRateY
									   M1= M1Radio_in+PIDRateX-PIDRateY;//;//-PIDRateY
                                      //---------------XASIS -----------------------------------
									   M3= M3Radio_in-PIDRateX+PIDRateY;//;//+PIDRateY
									   M4= M4Radio_in-PIDRateX-PIDRateY;//;//-PIDRateY
									 }
									 if(PIDoption==1){
																		   M2= M2Radio_in+pidx+pidy;//;//+PIDRateY
																		   M1= M1Radio_in+pidx-pidy;//;//-PIDRateY
									                                      //---------------XASIS -----------------------------------
																		   M3= M3Radio_in-pidx+pidy;//;//+PIDRateY
																		   M4= M4Radio_in-pidx-pidy;//;//-PIDRateY
																		 }


								 }


                                    if(flightmode==1){

										ErrorH=setheight-sensorheight;
										err_diffH=(ErrorH-PreviousErrH)/0.07;
										int_errH=int_errH + ErrorH;
								        p_termh=PGainH*ErrorH;  //2.4
										i_termh=IGainH*int_errH;
										d_termh=(DgainH*err_diffH);
										pidh=p_termh+d_termh+i_termh;
										PreviousErrH=ErrorH;

										M1=M1+pidh;
										M2=M2+pidh;
										M3=M3+pidh;
										M4=M4+pidh;


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
            //---Begin only  execute if StartMotor Flag =1;
            if((StartMotor==1)& (Radio_status==1))
            {

				TIM_SetCompare1(TIM1, M1); //M1
			 	TIM_SetCompare2(TIM1, M2); //M2
			 	TIM_SetCompare3(TIM1, M3); //M3
			 	TIM_SetCompare4(TIM1, M4); //M4



            //---End only  execute if StartMotor Flag =1;
	            if(XErrbuf<Logbuf)
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

	            else
	            {


	            }


}

            else

            {

            	TIM_SetCompare1(TIM1, 8000);
            		            	TIM_SetCompare2(TIM1, 8000);
            		            	TIM_SetCompare3(TIM1, 8000);
            		            	TIM_SetCompare4(TIM1, 8000);

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

        if(timercount%300==0){
        	serialflag=1;
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
            	setY=-25;
               }


            //   if(timercount%30000==0){
              //               setX=0.0;
                //             }

               if(timercount%20000==0){
                         		setY=0;
                             }


    }
}

void TIM3_IRQHandler(void)
{

	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	 //togglebit^=1;
	//	  if(togglebit) GPIOD->BSRRL = 0xF000; // set PD1
		  	 // wait a short period of time
	//	  else GPIOD->BSRRH = 0xF000; // reset PD1

	//	TIM_ICInitTypeDef  TIM_ICInitStructure;


/* Clear TIM4 Capture compare interrupt pending bit */
TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);

/* Get the Input Capture value */
IC2Value_radio = TIM_GetCapture2(TIM3);

if (IC2Value_radio != 0)
{
  /* Duty cycle computation */
  //DutyCycle = (TIM_GetCapture1(TIM4) * 100) / IC2Value;
  DutyCycle_radio=IC2Value_radio;
  DutyCycle2_radio=TIM_GetCapture1(TIM3);
  /* Frequency computation
     TIM4 counter clock = (RCC_Clocks.HCLK_Frequency)/2 */

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

