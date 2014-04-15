/**
  ******************************************************************************
  * @file    TIM_ComplementarySignals/main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

#include "main.h"



/** @defgroup USBH_USR_MAIN_Private_Variables
* @{
*/
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE      USB_OTG_Core __ALIGN_END;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USBH_HOST                USB_Host __ALIGN_END;
/**
* @}
*/




/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup TIM_ComplementarySignals
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
//#define ADC_CDR_ADDRESS    ((uint32_t)0x40012308)
#define ADC_CDR_ADDRESS		((uint32_t)(ADC_BASE + 8))


#define ADC_NUM_OF_SAMPLE	68
#define ADC_NUM_OF_CHANNEL	6
#define ADC_BUFF_SIZE		ADC_NUM_OF_CHANNEL*ADC_NUM_OF_SAMPLE

#define ADC_MAX_COUNTS				4095

//2568
#define ADC_COUNTS_FOR_VBATT_12V	955
#define ADC_COUNTS_FOR_VBATT_1V		((float)ADC_COUNTS_FOR_VBATT_12V/12)


#define ADC_COUNTS_FOR_VSOLAR_12V	963
#define ADC_COUNTS_FOR_VSOLAR_1V	((float)ADC_COUNTS_FOR_VSOLAR_12V/12)


#define ADC_COUNTS_FOR_IBATT_0A		528//472	//ACS755xCB-100 zero current out= 0.6V
#define ADC_COUNTS_FOR_IBATT_100A	3888
#define ADC_COUNTS_FOR_IBATT_1A	\
				((float)(ADC_COUNTS_FOR_IBATT_100A-ADC_COUNTS_FOR_IBATT_0A)/100)



#define ADC_COUNTS_FOR_ISOLAR_0A	528   //465
#define ADC_COUNTS_FOR_ISOLAR_100A	3988
#define ADC_COUNTS_FOR_ISOLAR_1A	\
				(((float)ADC_COUNTS_FOR_ISOLAR_100A-ADC_COUNTS_FOR_ISOLAR_0A)/100)


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint16_t PWM_Period = 0;
uint16_t Channel1Pulse = 0, Channel2Pulse = 0, Channel3Pulse = 0;
uint16_t msTicks;
volatile uint16_t ADC12DualConvertedBuff[ADC_BUFF_SIZE];
uint16_t ADCRaw_avg[ADC_NUM_OF_CHANNEL];
uint16_t test_avg[ADC_NUM_OF_CHANNEL];

struct SolarMPPTGlobalVar MPPTVars = {
	.userBit1 = 1,
};
struct Test_Float_Math test_float_math = {
	.test_float_en = DISABLE,
	.a = 0.12345679f,
	.b = 81,
};
struct SolarMPPT SolarCharger = {
	.num_of_MPPT_points = 168,
	.do_global_MPPT =1,
	.do_data_logging=0,

};


FATFS Sol_fs;
FIL Sol_log_file;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

float get_current_feedback(void)
{
	float tmp_feedback=0;
	int i;

	_delay_ms(388);
	for(i=0; i<38; i++){
		cal_adc();
		tmp_feedback += MPPTVars.PBatt;
	}

	return (tmp_feedback/38);

}

//find global MPPT point & return the PWM value

uint16_t find_global_MPPT_point(struct SolarMPPT * vars){
	int i;
	uint16_t delta_PWM;
	uint16_t num_of_MPPT_points_todo;
	num_of_MPPT_points_todo = vars->num_of_MPPT_points/3;// + vars->num_of_MPPT_points/25;
	delta_PWM = 1000 / vars->num_of_MPPT_points;
	vars->current_max_MPPT_value = 0;
	/////printf("---begin global MPPT --- \n");
	for(i=5; i< num_of_MPPT_points_todo; i++){
		set_PWM_charger(i*delta_PWM);
		vars->feedback_value[i] = get_current_feedback();
		/////printf("%.2f at %d \n", vars->feedback_value[i], i*delta_PWM);
		if (vars->current_max_MPPT_value < vars->feedback_value[i]){
			vars->current_max_MPPT_value = vars->feedback_value[i];
			vars->current_max_MPPT_point = i;
		}
	}

	set_PWM_charger(vars->current_max_MPPT_point*delta_PWM);
	/////printf("max at %d: %d %.2f \n", vars->current_max_MPPT_point, vars->current_max_MPPT_point*delta_PWM, vars->current_max_MPPT_value);



	return 0;
}
uint16_t current_PWM, up_PWM, down_PWM, upup_PWM, downdown_PWM;

uint16_t find_local_MPPT_point(struct SolarMPPT * vars){
	const uint16_t delta_PWM = 8;


	current_PWM = vars->current_PWM_charger;
	up_PWM = current_PWM + delta_PWM;
	upup_PWM = current_PWM + 2*delta_PWM;
	down_PWM = current_PWM - delta_PWM;
	downdown_PWM =  current_PWM - 2*delta_PWM;
	if((down_PWM<18) || (downdown_PWM<18)){
		down_PWM = 18;
		downdown_PWM = 18;

	}

	//set_PWM_charger(current_PWM);
	vars->feedback_local_current = get_current_feedback();

	set_PWM_charger(up_PWM);
	vars->feedback_local_up = get_current_feedback();
	//set_PWM_charger(upup_PWM);
	//vars->feedback_local_upup = get_current_feedback();

	set_PWM_charger(down_PWM);
	vars->feedback_local_down = get_current_feedback();
	//set_PWM_charger(downdown_PWM);
	//vars->feedback_local_down = get_current_feedback();

	set_PWM_charger(current_PWM); //very important to set PWM back to current

	if(vars->feedback_local_current < vars->feedback_local_down)
	{
		/////printf("---down local MPPT --- \n");
		/*////printf("%.2f %.2f %.2f \n",
				vars->feedback_local_down,
				vars->feedback_local_current,
				vars->feedback_local_up);
				*/
		/////printf("%d %d %d \n", down_PWM, current_PWM, up_PWM);
		set_PWM_charger(current_PWM-1);
	}
	if( vars->feedback_local_current < vars->feedback_local_up)
	{
		/////printf("---up local MPPT --- \n");
		/*////printf("%.2f %.2f %.2f \n",
				vars->feedback_local_down,
				vars->feedback_local_current,
				vars->feedback_local_up);
		*/
		/////printf("%d %d %d \n", down_PWM, current_PWM, up_PWM);
		set_PWM_charger(current_PWM+1);
	}


	return 0;
}
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */

	// SETUP CPU
	Cpu_enter_default_config();

	// SETUP all Ports
	Ports_enter_default_config();

	Usb_enter_default_config();

	Timers_enter_default_config();

	Adc_enter_default_config();

	RTC_to_default_state();

  	_delay_ms(100);
	LCD_Init();


	SolarCharger.current_PWM_charger = 288;
  while (1)
  {
  	/* Host Task handler TODO: Put in a timer interupt
  	to make sure USBH_Process(..) is called frequently*/
    	USBH_Process(&USB_OTG_Core, &USB_Host);

  	//MPPTVars.userBit1 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
  	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
  	RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);
	if(SolarCharger.curr_min != RTC_TimeStructure.RTC_Minutes){
		SolarCharger.last_min = SolarCharger.curr_min;
		SolarCharger.curr_min = RTC_TimeStructure.RTC_Minutes;
		SolarCharger.min_change = 1;
	}


	DisplayLCD();



	if((RTC_TimeStructure.RTC_Hours>14) && (RTC_TimeStructure.RTC_Hours<=23)){
		DIGITAL_OUTPUT_HIGH(D_OUT1);
	} else {
		DIGITAL_OUTPUT_LOW(D_OUT1);
	}


	if(STM_EVAL_PBGetState(BUTTON_USER)){
		//GPIOA->ODR ^= GPIO_Pin_0;
		//STM_EVAL_LEDToggle(Led_TypeDef Led)
		//GES_Solar_Log_Data();
	}else {

	}



	//all 100ms tasks go here
	if(MPPTVars.ms100_flag){
		MPPTVars.ms100_flag = 0;
		cal_adc();
	}

	if((RTC_TimeStructure.RTC_Minutes%5==0) && (SolarCharger.min_change==1) ){
		SolarCharger.min_change = 0;

		SolarCharger.do_data_logging=1;
		if(!SolarCharger.batt_full)
			SolarCharger.do_global_MPPT = 1;
	}
	if(MPPTVars.ms1000_flag){
		MPPTVars.ms1000_flag = 0;
		MPPTVars.sec_cnt++;
		if(MPPTVars.sec_cnt == 300){
			MPPTVars.sec_cnt = 0;

		} else if( (MPPTVars.sec_cnt % 3) == 0){
			if(!SolarCharger.batt_full)
				SolarCharger.do_local_MPPT = 1;
				//while (ITM_Port32(1) == 0);
					//ITM_Port32(1) = (unsigned int)'8';
		}
	}
	//SET_PULSE_CHANNEL_A(Channel1Pulse);

	//SET_PULSE_CHANNEL_B(Channel2Pulse);

	if(SolarCharger.do_data_logging){
		GES_Solar_Log_Data();
		SolarCharger.do_data_logging=0;
	}
	if(SolarCharger.do_global_MPPT){
		find_global_MPPT_point(&SolarCharger);
		SolarCharger.do_global_MPPT = 0;
	}

	if(SolarCharger.do_local_MPPT){
		find_local_MPPT_point(&SolarCharger);
		SolarCharger.do_local_MPPT = 0;
	}
	if(SolarCharger.en_manual_MPPT){
		set_PWM_charger(SolarCharger.manual_PWM_charger);

		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)){
			SolarCharger.manual_PWM_charger+=1;
			_delay_ms(300);
		} else {
			//LCD_printf("0");
		}
		MPPTVars.sec_cnt = 0;
	} else {
		LCD_GotoXY(18,2);
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)){
			LCD_printf("1");
			//SolarCharger.do_global_MPPT= 1;
		} else {
			LCD_printf("0");
		}

	}

	if(MPPTVars.VBatt> CHARGER_STOP_VOLTAGE){
		SolarCharger.batt_full = 1;
		DisableSolarCharger();
	} else if(MPPTVars.VBatt < CHARGER_START_VOLTAGE){
		SolarCharger.batt_full = 0;
		EnableSolarCharger();
	};


  }
}

void DisplayLCD(void){
	LCD_GotoXY(0,1);
	LCD_printf("Solar=%.1fV ", MPPTVars.VSolar);
	LCD_GotoXY(11,1);
	LCD_printf(" P= %.0fW", MPPTVars.PBatt);
	LCD_GotoXY(0,0);
	//LCD_printf("Solar= %.2fV %.2fA ", MPPTVars.VSolar, MPPTVars.ISolar, MPPTVars.PSolar);

	LCD_printf("%0.2d %0.2d:%0.2d:%0.2d %.1fA ",RTC_DateStructure.RTC_Date,
				RTC_TimeStructure.RTC_Hours,
		   		RTC_TimeStructure.RTC_Minutes,
				RTC_TimeStructure.RTC_Seconds,
				MPPTVars.ISolar);

	LCD_GotoXY(0,2);
	LCD_printf("Batt= %.1fV %.1fA ",MPPTVars.VBatt, MPPTVars.IBatt, MPPTVars.PBatt);

	LCD_GotoXY(0,3);
	LCD_printf("%5d", msTicks);

	LCD_GotoXY(6,3);
	LCD_printf("%5d",TIM1->CCR1);

	LCD_GotoXY(5,3);


	LCD_GotoXY(12,3);
	if(SolarCharger.batt_full){

		LCD_printf("BattFull");
	} else {

		LCD_printf("Charging");
	}

}

// printf like function for the LCD screen
void get_string( uint8_t* ret, const uint8_t* format, ... ) {
    va_list args;
    //char        string[88];

    va_start( args, format );
    vsprintf( ret, format, args );
    va_end( args );
    //LCD_Print( string );
}

char log_str[38];

void GES_Solar_Log_Data(void)
{
	int ret;


	printf("> Writing Solar Log file ...\n");
	//check for Write protect
	if(USBH_MSC_Param.MSWriteProtect == DISK_WRITE_PROTECTED)
	{
		printf( "> Disk flash is write protected \n");
		USBH_USR_ApplicationState = USH_USR_FS_LOGGING;
		return;
	}

	/* Register work area for logical drives (malloc)*/
	f_mount(0, &Sol_fs);

	if(f_open(&Sol_log_file, "0:GES_Sol.TXT",FA_OPEN_ALWAYS | FA_READ | FA_WRITE) == FR_OK)
	{
		get_string(log_str, "%0.2d:%0.2d:%0.2d:%0.2d::%.1f:%.1f:%.1f:%.1f\n",
						RTC_DateStructure.RTC_Date,
						RTC_TimeStructure.RTC_Hours,
						RTC_TimeStructure.RTC_Minutes,
						RTC_TimeStructure.RTC_Seconds,
						MPPTVars.VBatt,
						MPPTVars.IBatt,
						MPPTVars.VSolar,
						MPPTVars.ISolar);

		f_lseek(&Sol_log_file, Sol_log_file.fsize);
		ret = f_printf(&Sol_log_file, log_str);


		if(ret == -1) /*EOF or Error*/{
			printf(">CANNOT log data\n");
		} else {
			printf("> data logged\n");
		}

			 /*close file and filesystem*/
		f_close(&Sol_log_file);
	} else {
		printf(">f_open != FR_OK\n");
	}

	f_mount(0, NULL);

}


void set_PWM_charger(uint16_t pwm){

	uint16_t pulse;
	SolarCharger.current_PWM_charger = pwm;
	pulse = GET_PWM_DUTY_CYCLE(pwm,1000,PWM_PERIOD);

	if (pulse>MAX_PWM_PULSE){
		pulse = MAX_PWM_PULSE;
	}
	if (pulse<MIN_PWM_PULSE){
		pulse = MIN_PWM_PULSE;
	}

	SET_PULSE_CHANNEL_A(pulse);

}


#define DELTA_ADC_CNT 	300
volatile int testPoint;

void cal_adc()
{
	unsigned int i,j;
	uint32_t ADCSum, num_of_out_adc_range_value;
	int32_t tmp_buff;

	//get the initial avg
	for (i=0; i<ADC_NUM_OF_CHANNEL; i++){
		ADCSum=0;
		for(j=i; j<ADC_BUFF_SIZE; j += ADC_NUM_OF_CHANNEL){

			ADCSum+= ADC12DualConvertedBuff[j];
		}
		ADCRaw_avg[i] = ADCSum/ADC_NUM_OF_SAMPLE;
	}
	/*
	//get the exact avg
	for (i=0; i<ADC_NUM_OF_CHANNEL; i++){
		ADCSum=0;
		num_of_out_adc_range_value = 0;

		for(j=i; j<ADC_BUFF_SIZE; j += ADC_NUM_OF_CHANNEL){
			tmp_buff = ADC12DualConvertedBuff[j];

			//check for out of range ADC value
			if((((int32_t)ADCRaw_avg[i] - tmp_buff)>DELTA_ADC_CNT)
				|| ((tmp_buff - (int32_t)ADCRaw_avg[i])>DELTA_ADC_CNT)) {
				num_of_out_adc_range_value +=1;

			} else {

				ADCSum+= tmp_buff;
			}
		}

		ADCRaw_avg[i] = ADCSum/(ADC_NUM_OF_SAMPLE-num_of_out_adc_range_value);
	}

	*/

	testPoint = 0;

	MPPTVars.VBatt = (float)ADCRaw_avg[1]/ADC_COUNTS_FOR_VBATT_1V;
	MPPTVars.ISolar = ((float)ADCRaw_avg[0]-ADC_COUNTS_FOR_ISOLAR_0A)/ADC_COUNTS_FOR_ISOLAR_1A;


	MPPTVars.VSolar = (float)ADCRaw_avg[3]/ADC_COUNTS_FOR_VSOLAR_1V;
	MPPTVars.IBatt = ((float)ADCRaw_avg[2]-ADC_COUNTS_FOR_IBATT_0A)/ADC_COUNTS_FOR_IBATT_1A;
/*
	if(MPPTVars.IBatt < 0)
		MPPTVars.IBatt = 0;
	if(MPPTVars.ISolar<0)
		MPPTVars.ISolar = 0;
*/

	MPPTVars.PBatt = MPPTVars.IBatt*MPPTVars.VBatt;
	MPPTVars.PSolar = MPPTVars.ISolar*MPPTVars.VSolar;
	testPoint = 1;
}


void Cpu_enter_default_config(void)
{
	if (SysTick_Config(SystemCoreClock / 1000)){
		printf("ERROR: SysTick_Config failed\n");
	} else {
		printf("SysTick_Config success!\n");
		NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) -1);
	}
}


//----------------------------------------------------------
// Sets up ports for all peripherals
// NOTE 'default' Refers to the default device setup for the application.

//GPIOA pin 0 : USER_BUTTON
//GPIOA Pin1 :
//GPIOA Pin4 : I2S3_WS
//GPIOA Pin 5 & 7 : SPI1
void Ports_enter_default_config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

/* BOARD LED & BUTTON*************************/
	//Enable the GPIO_LED Clock GPIOD
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	/* Configure the GPIO_LED pins & digital output port*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	//Enable Digital Output ports
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);

  	// Enable the User BUTTON Clock: GPIOA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	//Configure the GPIO User Button as input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);





	//Clock for EXTI if you wanna use External interupt
  	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

/**ADC************************************/
	//Enable Clock for GPIOA GPIOB
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
	//Configure GPIO Pins for using with ADC
	// PA1 PA3  PB0, PB1  for ADC Channel 1,3,8,9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
/**PWM************************************/
	//PWM 1,2,3  --> PA8, PE11, PA10
	//PWM 1N,2N,3N --> PB13, PB14, PB15
	//Break IN (BKIN) --> PB12
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
						RCC_AHB1Periph_GPIOB |
						RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOE, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//Connect pin AF
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8,GPIO_AF_TIM1);
	//GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
	//GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_TIM1);
	////GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM1);
	//GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM1);
	////GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);

/***LCD required ports************************/
	//LCD port init
	LCD_port_init();
	//D:0 2 4 6
	//E:1 3
	//B:9

/***USB required ports ***********************/
	//RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA , ENABLE);

	/* Configure DM DP Pins */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 |
							GPIO_Pin_12;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_OTG1_FS) ;
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_OTG1_FS) ;


	//VBUS_FS
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	/* this for ID line debug */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_OTG1_FS) ;

	//C0 to enable 5V output
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, GPIO_Pin_0);  //default disable VBUS
}


void Timers_enter_default_config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
	//uint16_t PWM_Frequency_Hz = PWM_FREQ_HZ;

	PWM_Period = PWM_PERIOD;
	Channel1Pulse = GET_PWM_DUTY_CYCLE(PWM_CHANNEL_A_INIT,PWM_PERIOD,PWM_Period);
	Channel2Pulse = GET_PWM_DUTY_CYCLE(PWM_CHANNEL_B_INIT,PWM_PERIOD,PWM_Period);
	Channel3Pulse = GET_PWM_DUTY_CYCLE(PWM_CHANNEL_C_INIT,PWM_PERIOD,PWM_Period);

	//Time Base config
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = PWM_Period;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	// Channel 1,2,3 OC config in PWM mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);


	TIM_OCInitStructure.TIM_Pulse = Channel2Pulse;
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);

	//Config Break, deadtime, automatic output
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
	TIM_BDTRInitStructure.TIM_DeadTime = PWM_DEADTIME_DEFAULT;
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

	//TIM1 begin to Count
	TIM_Cmd(TIM1, ENABLE);

	//Now Begin output PWM
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void EnableSolarCharger(void){
	//TIM1 begin to Count
	TIM_Cmd(TIM1, ENABLE);

	//Now Begin output PWM
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

}

void DisableSolarCharger(void){
	TIM_Cmd(TIM1, DISABLE);
	TIM_SetCounter(TIM1,0);
	TIM_CtrlPWMOutputs(TIM1,DISABLE);

}

ADC_InitTypeDef 	  ADC_InitStructure;
ADC_CommonInitTypeDef ADC_CommonInitStructure;
DMA_InitTypeDef DMA_InitStructure;

void Adc_enter_default_config(void)
{


	//Enable Clock to ADC1, 2, 3
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 |
						RCC_APB2Periph_ADC2 |
						RCC_APB2Periph_ADC3,ENABLE);
	//Enable Clock to DMA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	//Init DMA for ADC
	DMA_DeInit(DMA2_Stream0);
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC12DualConvertedBuff;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC_CDR_ADDRESS;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = ADC_BUFF_SIZE/2;  //this is the memory buffer size
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;

	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC8;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);



	ADC_DeInit();
	//ADC Common Configuration
	ADC_CommonInitStructure.ADC_Mode = ADC_DualMode_RegSimult;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;  //APB2 clk =84Mhz --> ADC clk=21Mhz
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_2;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	//ADC1,2 Regular channels configuration
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 3;

	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_56Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_56Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_56Cycles);

	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 1, ADC_SampleTime_56Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 2, ADC_SampleTime_56Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 3, ADC_SampleTime_56Cycles);

	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
	//ADC_SoftwareStartConv(ADC1);
	/* DMA2_Stream0 enable */
	DMA_Cmd(DMA2_Stream0, ENABLE);
	ADC_SoftwareStartConv(ADC1);

}



void Usb_enter_default_config(void){
	  /* Init Host Library */
	  USBH_Init(&USB_OTG_Core,
	            USB_OTG_FS_CORE_ID,
	            &USB_Host,
	            &USBH_MSC_cb,
	            &USR_cb);
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

  while (1)
  {}
}

#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
