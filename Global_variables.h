extern void fuseGyroAcc(int,int,int,int,int,int);

extern char Csign(float);
extern int C1(float);
extern int C2(float);
extern void init_USART3(uint32_t);
extern void init_USART2(uint32_t);
extern void  UART2_sendbyte(uint8_t);
extern void USART_puts(USART_TypeDef*, volatile char *);
extern void radio_in(void);
extern  void PWMinput_radioCH3(void);
//void USART_puts(USART_TypeDef* USARTx, volatile char *s);
#define SERIAL_BUFFER_SIZE 512
extern char serial_buffer[SERIAL_BUFFER_SIZE] ;
extern int sb_index ;
extern int end_index ;
extern int Buf_counte;
extern int togglebit;
extern int timercount;
extern int his_timercount;
extern int stoptimer;
extern int serialflag;
extern int movavgcounter;
extern int expect_received;
extern int received_msg;
extern int16_t receivedmsg[25];
//good performance
//P=0.09, D=0.008
// P=0.07, D=0.006
//P=0.004 , I=0.0001,
//Pgain=0.0008 is good.
// Pgain =0.001 is ok, 0.002 will oscillate .0.0001
extern float PGain,ErrorX,ErrorY,setX,setY,setheight,ErrorH,GH;
extern float IGain,Dgain,err_diffX,err_diffY,int_errX,int_errY,PreviousErrX,PreviousErrY; //Pgain=0.2 , Dgain=0.005 ,lIGain=0.005  best possible configuration
extern int16_t AccXA[20],AccYA[20],AccZA[20],GyroXA[20],GyroYA[20],GyroZA[20];
extern float DM_roll,DM_pitch,DM_raw;
extern int16_t  DM_cmd,DM_roll_cal,DM_pitch_cal,DM_raw_cal,DM_AccelX_cal,DM_AccelY_cal,DM_AccelZ_cal,DM_CompAngRateX_cal,DM_CompAngRateY_cal,DM_CompAngRateZ_cal,DM_TimerTicks_cal,checksum;
extern int16_t mutex,CRCvalidation,Gyro_sensitivity,DM_CompAngRateX;
//extern int16_t DM_cmd;

//Radio status state machine
extern int Radio_status;
extern int Rstate;
extern int ns_radio;
extern int tempradio_status;

//end Radio status state machine




//idea, X Err Signal, Y Err Signal, M1 ,M2,M3,M4,


extern int XErrbuf;
extern float M1,M2,M3,M4,radioin;
extern int trottle_manual,trottleintrrupt,conttrolflag,setmotorspeed,state1;
extern int IC2Value_radio, DutyCycle_radio,DutyCycle2_radio,Frequency_radio,curr_ch3,prev_ch3;
extern int IC2Value_radioCh5,DutyCycle_radio5,DutyCycle2_radio5,Frequency_radio5;
extern int IC2Value_radioCh9,DutyCycle_radio9,DutyCycle2_radio9,Frequency_radio9;
extern float temperrX,temperrY,RateAxz,RateAyz,AccAngleX,AccAngleY;
extern float p_termx,i_term,d_termx,p_termy,i_termy,d_termy;
extern float pidx,pidy;
#define I2C_TIMEOUT  (0x10)
#define PI 3.14159265358979
extern float MXlimit;
#define MNlimit 500
#define step 100
//Version Number : Major.Minor.Complete/Progress
// Revision : ZIO1.M1.2 # added PWM for motor


// this slave address belongs to the STM32F4-Discovery board's
// CS43L22 Audio DAC
// connect PD4 to VDD in order to get the DAC out of reset and test the I2C
// interface
#define SLAVE_ADDRESS 0x68 // the slave address (example)

extern __IO uint16_t IC2Value ;
extern __IO uint16_t DutyCycle;
extern __IO uint16_t StartMotor;
extern __IO uint16_t DutyCycle2;
extern __IO uint32_t Frequency ;



extern int16_t AccXvalue;
extern int16_t AccYvalue;
extern int16_t  AccZvalue;
extern int16_t GyroXvalue;
extern int16_t GyroYvalue;
extern int16_t GyroZvalue;

extern float RxEst,RyEst,RzEst,RxEstpast,RyEstpast,RzEstpast,Axr,Ayr,Azr;
extern float 	RxAccR,RyAccR,RzAccR,RxGyroR,RyGyroR,RzGyroR;
#define SENSITIVITY 4096
extern int T;


#define MAX_STRLEN 5 // this is the maximum string length of our string in characters
extern volatile char received_string[MAX_STRLEN+1]; // this will hold the recieved string
