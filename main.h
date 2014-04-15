#ifndef __MAIN_H
#define __MAIN_H
#include "stm32f4_discovery.h"

#include "float.h"
#include "math.h"
#include "hd44780.h"
#include "bitband.h"
#include "GES_Solar_RTC.h"




#include "usbh_core.h"
#include "usbh_usr.h"
#include "usbh_msc_core.h"


#define BATT_FULL_VOLTAGE	29.5
#define CHARGER_STOP_VOLTAGE	BATT_FULL_VOLTAGE
#define CHARGER_START_VOLTAGE	26.5

//#define USE_STDPERIPH_DRIVER
//#define STM32F4XX
#define PWM_DEADTIME_DEFAULT	(88)
#define PWM_FREQ_HZ		(8888)

#define PWM_PERIOD		((SystemCoreClock / PWM_FREQ_HZ) - 1)

#define MAX_PWM_PULSE	(PWM_PERIOD-588)
#define MIN_PWM_PULSE	(268)

#define PWM_CHANNEL_A_INIT	(288)
#define PWM_CHANNEL_B_INIT	(588)
#define PWM_CHANNEL_C_INIT	(688)

#define GET_PWM_DUTY_CYCLE(duty, max_duty, Period)	\
	((uint16_t) (((uint32_t)duty * (PWM_Period-1))/max_duty))

#define SET_PWM_CHANNEL_A(duty, max_duty, Period)	\
	TIM1->CCR1 = GET_PWM_DUTY_CYCLE(duty,max_duty,Period)

#define SET_PWM_CHANNEL_B(duty, max_duty, Period)	\
	TIM1->CCR2 = GET_PWM_DUTY_CYCLE(duty,max_duty,Period)

#define SET_PULSE_CHANNEL_A(pulse)	\
	TIM1->CCR1 = pulse;
#define SET_PULSE_CHANNEL_B(pulse)	\
	TIM1->CCR2 = pulse;

#define ITM_Port32(n) (*((volatile unsigned int *)(0xE0000000+4*n)))

#define D_OUT1_PORT		GPIOD_BASE
#define D_OUT1_PIN		14
#define D_OUT2_PORT		GPIOD_BASE
#define D_OUT2_PIN		12
#define D_OUT3_PORT		GPIOD_BASE
#define D_OUT3_PIN		10

#define DIGITAL_OUTPUT_HIGH(d_out)	\
		*((volatile unsigned char *) \
			(BITBAND_PERI(d_out##_PORT+BSRRL_REG_OFFSET,d_out##_PIN))) = 1

#define DIGITAL_OUTPUT_LOW(d_out)	\
		*((volatile unsigned char *) \
			(BITBAND_PERI(d_out##_PORT+BSRRH_REG_OFFSET,d_out##_PIN))) = 1


enum SolarMPPT_Type {
	MPPT_IBATT,
	MPPT_PBATT,
	MPPT_PSOLAR
};
//should use MPPT_IBATT

/*	TODO: use other algorithm which search MPPT on the fly, no need float buffer
	******get global MPPT point.*******
	SEARCH for global MPPT point and SET PWM_Charger to that point
	*****input: number of global MPPT point to sample: num_of_MPPT_points
	*****input: MAX_PWM_PULSE & MIN_PWM_PULSE
	*****input: function pointer to set PWM
	*****input: function pointer to get feedback value
	*****input: a float array to store feedback value
	*****return: uint16_t MPPT_PWM_Pulse

	delta_PWM = (MAX_PWM_PULSE -MIN_PWM_PULSE) / num_of_MPPT_points;


	for(i=0; i<num_of_MPPT_points; i++) {
		set_PWM_Charger(MIN_PWM_PULSE+i*delta_PWM);
		feedback_value[i] = get_feedback_value(us, num);

	}
	debug print out


	*****get local MPPT point ******


**/
#define NUM_FEEBACK_BUFFER
struct SolarMPPT	{
	enum SolarMPPT_Type MPPT_Type; //do MPPT on IBATT PBATT or PSOLAR
	uint16_t 			us_delay_between_samples; //delay time between samples
	uint16_t			num_of_samples;	//number of samples to get average
	uint16_t 			num_of_MPPT_points;
	uint16_t			current_max_MPPT_point;

	float 			feedback_value[88];
	float			feedback_local_current;
	float			feedback_local_up;
	float			feedback_local_upup;
	float			feedback_local_down;
	float			feedback_local_downdown;
	int16_t			(*set_PWM_Charger)(uint16_t PWM_Solar);
	//function to set PWM of charger
	float			(*get_feedback_value)(uint16_t us, uint16_t num);
	// function to get feedback value (need us delay between sample & num of samples

	uint16_t 			current_PWM_charger;
	uint16_t			manual_PWM_charger;
	float			current_max_MPPT_value;


	unsigned int 		do_global_MPPT:1;
	unsigned int		do_local_MPPT:1;
	unsigned int		do_data_logging:1;
	unsigned int 		en_manual_MPPT:1;
	unsigned int		min_change:1;
	uint8_t			last_min;
	uint8_t			curr_min;

	unsigned int		batt_full:1;

};

struct SolarMPPTGlobalVar {


	float IBatt;
	float VBatt;
	float ISolar;
	float VSolar;
	float PSolar;
	float PBatt;
	uint16_t msTickerTest;
	uint8_t user_input_test;
	uint8_t user_input_test2;
	uint16_t		sec_cnt;
	unsigned int ms10_flag:1;
	unsigned int ms100_flag:1;
	unsigned int ms1000_flag:1;

	unsigned int userBit1:1;
	unsigned int userBit2:1;
	unsigned int enable_auto_adc:1;
	unsigned int enable_VSolar_in:1;



/*
	float VBatt_scale_100milivol;
	float VSolar_scale_100milivol;
	float VBatt_scale_vol;
	float VSolar_scale_vol;
*/


};

struct Test_Float_Math {

	FunctionalState test_float_en;

	float a, b, x, y, z;

	uint16_t ms_mul[2], ms_div[2], ms_sqrt[2];


};

extern uint32_t SystemCoreClock;
extern struct SolarMPPTGlobalVar MPPTVars;

extern USB_OTG_CORE_HANDLE          USB_OTG_Core;
extern USBH_HOST                    USB_Host;


void Delay(__IO uint32_t);
void Cpu_enter_default_config(void);
void Ports_enter_default_config(void);
void Adc_enter_default_config(void);
void Timers_enter_default_config(void);
void cal_adc();
void do_solar_MPPT(struct SolarMPPT input);
void set_PWM_charger(uint16_t pwm);
void EnableSolarCharger(void);
void DisableSolarCharger(void);
void DisplayLCD(void);

void Usb_enter_default_config(void);
void GES_Solar_Log_Data(void);
#endif