#ifndef STM32F446X_TIMER_DRIVER_H_
#define STM32F446X_TIMER_DRIVER_H_


#include "stm32f407xx.h"

/* BIT POSSITION */
// TIM_CR1
#define TIM_CR1_CEN			0
#define TIM_CR1_UDIS		1
#define TIM_CR1_URS			2
#define TIM_CR1_OPM			3
#define TIM_CR1_DIR			4
#define TIM_CR1_CMS			5
#define TIM_CR1_ARPE		7
#define TIM_CR1_CKD			8
// TIM_CR2
#define TIM_CR2_CCPC		0
#define TIM_CR2_CCUS		2
#define TIM_CR2_CCDS		3
#define TIM_CR2_MMS			4
#define TIM_CR2_TI1S		7
#define TIM_CR2_OIS1		8
#define TIM_CR2_OIS1N		9
#define TIM_CR2_OIS2		10
#define TIM_CR2_OIS2N		11
#define TIM_CR2_OIS3		12
#define TIM_CR2_OIS3N		13
#define TIM_CR2_OIS4		14
// TIM_SMCR
#define TIM_SMCR_SMS		0
#define TIM_SMCR_TS			4
#define TIM_SMCR_MSM		7
#define TIM_SMCR_ETF		8
#define TIM_SMCR_ETPS		12
#define TIM_SMCR_ECE		14
#define TIM_SMCR_ETP		15
// TIM_DIER
#define TIM_DIER_UIE			0
#define TIM_DIER_CC1IE		1
#define TIM_DIER_CC2IE		2
#define TIM_DIER_CC3IE		3
#define TIM_DIER_CC4IE		4
#define TIM_DIER_COMIE		5
#define TIM_DIER_TIE			6
#define TIM_DIER_BIE			7
#define TIM_DIER_UDE			8
#define TIM_DIER_CC1DE		9
#define TIM_DIER_CC2DE		10
#define TIM_DIER_CC3DE		11
#define TIM_DIER_CC4DE		12
#define TIM_DIER_COMDE		13
#define TIM_DIER_TDE			14
// TIMx_SR
#define TIM_SR_UIF				0
#define TIM_SR_CC1IF			1
#define TIM_SR_CC2IF			2
#define TIM_SR_CC3IF			3
#define TIM_SR_CC4IF			4
#define TIM_SR_COMIF			5
#define TIM_SR_TIF				6
#define TIM_SR_BIF				7
#define TIM_SR_CC1OF			9
#define TIM_SR_CC2OF			10
#define TIM_SR_CC3OF			11
#define TIM_SR_CC4OF			12
// TIMx_EGR
#define TIM_EGR_UG				0
#define TIM_EGR_CC1G			1
#define TIM_EGR_CC2G			2
#define TIM_EGR_CC3G			3
#define TIM_EGR_CC4G			4
#define TIM_EGR_CCMG			5
#define TIM_EGR_TG				6
#define TIM_EGR_BG				7
// TIMx_CCMR1
#define TIM_CCMR1_CC1S		0
#define TIM_CCMR1_CC2S		8
// output compare
#define TIM_CCMR1_OC1FE		2
#define TIM_CCMR1_OC1PE		3
#define TIM_CCMR1_OC1M		4
#define TIM_CCMR1_OC1CE		7
#define TIM_CCMR1_OC2FE		10
#define TIM_CCMR1_OC2PE		11
#define TIM_CCMR1_OC2N		12
#define TIM_CCMR1_OC2CE		15
// input capture
#define TIM_CCMR1_IC1PSC	2
#define TIM_CCMR1_IC1F		4
#define TIM_CCMR1_IC2PSC	10
#define TIM_CCMR1_IC2F		12

// TIMx_CCMR2
#define TIM_CCMR2_CC3S		0
#define TIM_CCMR2_CC4S		8
// output compare
#define TIM_CCMR2_OC3FE		2
#define TIM_CCMR2_OC3PE		3
#define TIM_CCMR2_OC3M		4
#define TIM_CCMR2_OC3CE		7
#define TIM_CCMR2_OC4FE		10
#define TIM_CCMR2_OC4PE		11
#define TIM_CCMR2_OC4N		12
#define TIM_CCMR2_OC4CE		15
// input capture
#define TIM_CCMR2_IC3PSC	2
#define TIM_CCMR2_IC3F		4
#define TIM_CCMR2_IC4PSC	10
#define TIM_CCMR2_IC4F		12

// TIMx_CCER
#define TIM_CCER_CC1E			0
#define TIM_CCER_CC1P			1
#define TIM_CCER_CC1NE		2
#define TIM_CCER_CC1NP		3
#define TIM_CCER_CC2E			4
#define TIM_CCER_CC2P			5
#define TIM_CCER_CC2NE		6
#define TIM_CCER_CC2NP		7
#define TIM_CCER_CC3E			8
#define TIM_CCER_CC3P			9
#define TIM_CCER_CC3NE		10
#define TIM_CCER_CC3NP		11
#define TIM_CCER_CC4E			12
#define TIM_CCER_CC4P			13
#define TIM_CCER_CC4NP		15

// TIMx_BDTR
#define TIM_BDTR_DTG			0
#define TIM_BDTR_LOCK			8
#define TIM_BDTR_OSSI			10
#define TIM_BDTR_OSSR			11
#define TIM_BDTR_BKE			12
#define TIM_BDTR_BKP			13
#define TIM_BDTR_AOE			14
#define TIM_BDTR_MOE			15

// TIMx_DCR
#define TIM_DCR_DBA				0
#define TIM_DCR_DBL				8

typedef struct
{
	uint32_t Prescaler;				/*Min_Data = 0x0000 and Max_Data = 0xFFFF*/
	uint32_t CounterMode;     /* @ref TIM_Counter_Mode */
	uint32_t Period;					/* Min_Data = 0x0000 and Max_Data = 0xFFFF */
	uint32_t ClockDivision;		/* @ref TIM_ClockDivision */
//	uint32_t RepetitionCounter;
	uint32_t AutoReloadPreload; 	/* @ref TIM_AutoReloadPreload */
}TIM_BaseInit_t;

/*
 *@ref TIM_Counter_Mode
 */
#define TIM_COUNTERMODE_UP		0
#define TIM_COUNTERMODE_DOWN	(1 << TIM_CR1_DIR) /* only for timer2 - timer5 */
/*
 * @ref TIM_ClockDivision
 * #note : unavalible for timer base  
 */
#define TIM_CLOCKDIVISION_DIV1             ~(0x3<<TIM_CR1_CKD)                     /*!< Clock division: tDTS=tCK_INT   */
#define TIM_CLOCKDIVISION_DIV2             (0x1<<TIM_CR1_CKD)                      /*!< Clock division: tDTS=2*tCK_INT */
#define TIM_CLOCKDIVISION_DIV4             (0x2<<TIM_CR1_CKD)                      /*!< Clock division: tDTS=4*tCK_INT */ 

/* 
 *@ref TIM_AutoReloadPreload
 */
#define TIM_AUTORELOAD_PRELOAD_DISABLE                0x00000000U               /*!< TIMx_ARR register is not buffered */
#define TIM_AUTORELOAD_PRELOAD_ENABLE                 (1<<TIM_CR1_ARPE)              /*!< TIMx_ARR register is buffered */


typedef struct
{
	TIM_BaseInit_t Init;
	TIM_RegDef_t *pTIMx;
	
}	TIM_Handler_t;

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
static void TIM_PeriControl(TIM_RegDef_t *pTIMx, uint8_t EnorDis);
static void TIM_IT_Enable(TIM_RegDef_t *pTIMx, uint32_t TIM_IT_E);
void TIM_Base_Init(TIM_Handler_t *pTIMBase);

/*
 * IRQ Configuration and ISR handling
 */
void TIM_IRQInterruptConfig(uint8_t IRQNumber, uint32_t IRQPriority, uint8_t EnorDi);
void TIM_IRQHandling(TIM_RegDef_t * pTIMx);

void TIM_ApplicationEventCallback(TIM_RegDef_t *pTIMx,uint8_t AppEv);

#endif