#ifndef STM32F407XX_PWR_DRIVER_H
#define STM32F407XX_PWR_DRIVER_H
#include "stm32f407xx.h"



typedef struct
{
  uint32_t PVDLevel;   /*!< PVDLevel: Specifies the PVD detection level.
                            This parameter can be a value of @ref PWR_PVD_detection_level */

  uint32_t Mode;      /*!< Mode: Specifies the operating mode for the selected pins.
                           This parameter can be a value of @ref PWR_PVD_Mode */
}PWR_PVDTypeDef;

#define WFI()		__asm __vo ("wfi")
#define WFE()		__asm __vo ("wfe")
#define SEV()		__asm __vo ("sev")

#define WAKEUP_PIN		8

#define PWR_CLEAR_FLAG(FLAG)	PWR->CR |= (0x1<<(FLAG))
#define PWR_GET_FLAG(FLAG)		(PWR->CSR & (0x1<<(FLAG)))


#define PWR_CR_LPDS			0
#define PWR_CR_PDDS			1
#define PWR_CR_CWUF			2
#define PWR_CR_CSBF			3
#define PWR_CR_PVDE			4
#define PWR_CR_PLS			5
#define PWR_CR_DBP			8
#define PWR_CR_FPDS 		9
#define PWR_CR_VOS			14

#define PWR_CSR_WUF			0
#define PWR_CSR_SBF			1
#define PWR_CSR_PVDO		2
#define PWR_CSR_BRR			3
#define PWR_CSR_EWUP		8
#define PWR_CSR_BRE			9
#define PWR_CSR_VOSRDY	14

/* 
 * @ref PWR_Regulator_state_in_STOP_mode PWR Regulator state in SLEEP/STOP mode
 */
#define PWR_MAINREGULATOR_ON                        0x0
#define PWR_LOWPOWERREGULATOR_ON                    0x1

/* 
 * @ref PWR_SLEEP_mode_entry PWR SLEEP mode entry
 */
#define PWR_SLEEPENTRY_WFI              ((uint8_t)0x01)
#define PWR_SLEEPENTRY_WFE              ((uint8_t)0x02)
#define PWR_SLEEPENTRY_WFE_NO_EVT_CLEAR ((uint8_t)0x03)

/*
 * @ref PWR_STOP_mode_entry PWR STOP mode entry
 */
#define PWR_STOPENTRY_WFI               ((uint8_t)0x01)
#define PWR_STOPENTRY_WFE               ((uint8_t)0x02)
#define PWR_STOPENTRY_WFE_NO_EVT_CLEAR  ((uint8_t)0x03)

 
/*
 * @ref PWR_PVD_Mode PWR PVD Mode
 */
#define PWR_PVD_MODE_NORMAL                 0x00000000U   /*!< basic mode is used */

#define PWR_PVD_MODE_IT_RISING              0x00010001U   /*!< External Interrupt Mode with Rising edge trigger detection */
#define PWR_PVD_MODE_IT_FALLING             0x00010002U   /*!< External Interrupt Mode with Falling edge trigger detection */
#define PWR_PVD_MODE_IT_RISING_FALLING      0x00010003U   /*!< External Interrupt Mode with Rising/Falling edge trigger detection */

#define PWR_PVD_MODE_EVENT_RISING           0x00020001U   /*!< Event Mode with Rising edge trigger detection */
#define PWR_PVD_MODE_EVENT_FALLING          0x00020002U   /*!< Event Mode with Falling edge trigger detection */
#define PWR_PVD_MODE_EVENT_RISING_FALLING   0x00020003U   /*!< Event Mode with Rising/Falling edge trigger detection */

	
/* ===============================================================================
                 ##### Peripheral Control functions #####
 =============================================================================== */
	
void PWR_EnableBkUpAccess(void);
void PWR_DisableBkUpAccess(void);

void PWR_ConfigPVD(PWR_PVDTypeDef *sConfigPVD);

void PWR_EnablePVD(void);
void PWR_DisablePVD(void);

void PWR_EnableWakeUppin(void);
void PWR_DisableWakeUppin(void);

void PWR_EnterSLEEPMode(uint32_t Regulator, uint8_t SLEEPEntry);
void PWR_EnterSTOPMode(uint32_t Regulator, uint8_t STOPEntry);
void PWR_EnableBkUpRegulator(void);
void PWR_EnterSTANDBYMode(void);

void PWR_PVD_IRQHandler(void);
__weak void PWR_PVD_Callback(void);

void PWR_EnableSleepOnExit(void);
void PWR_DisableSleepOnExit(void);
void PWR_EnableSEVOnPend(void);
void PWR_DisableSEVOnPend(void);

#endif