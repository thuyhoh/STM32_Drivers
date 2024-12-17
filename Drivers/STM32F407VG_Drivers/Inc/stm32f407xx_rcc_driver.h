///*
// * stm32f407xx_rcc_driver.h
// *
// *  Created on: Mar 29, 2019
// *      Author: admin
// */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include "stm32f407xx.h"

#define RCC_CR_HSION    0
#define RCC_CR_HSIRDY   1
#define RCC_CR_HSITRIM  3
#define RCC_CR_HSICAL   8
#define RCC_CR_HSEON    16
#define RCC_CR_HSERDY		17
#define RCC_CR_HSEBYP		18
#define RCC_CR_CSSON		19
#define RCC_CR_PLLON		24
#define RCC_CR_PLLRDY		25
#define RCC_CR_PLLI2SON	26
#define RCC_CR_PLLI2SRDY 27

#define RCC_PLLCFGR_PLLM		0
#define RCC_PLLCFGR_PLLN		6
#define RCC_PLLCFGR_PLLP		16
#define RCC_PLLCFGR_PLLSRC	22
#define RCC_PLLCFGR_PLLQ		24

#define RCC_CFGR_SW				0
#define RCC_CFGR_SWS			2
#define RCC_CFGR_HPRE			4
#define RCC_CFGR_PPRE1		10
#define RCC_CFGR_PPRE2		13
#define RCC_CFGR_RTCPRE		16
#define RCC_CFGR_MCO1			21
#define RCC_CFGR_I2SSCR		23
#define RCC_CFGR_MCO1PRE 	24
#define RCC_CFGR_MCO2PRE	27
#define RCC_CFGR_MCO2			30

#define RCC_APB1RSTR_PWRRST			28





typedef struct
{
	uint8_t PLLState;			/*!< @ref RCC_PLL_Config PLL Config >!*/
	uint8_t PLLSource;		/*!< @ref RCC_PLL_Clock_Source PLL Clock Source >!*/
	uint8_t PLLM;					/*!< @ref MIN 0 MAX 63 >!*/
	uint16_t PLLN;				/*!< @ref MIN 0 MAX 432>!*/
	uint8_t PLLP;					/*!< @ref RCC_PLLP_Clock_Divider >!*/
	uint8_t PLLQ;					/*!< @ref MIN 2 MAX 15>!*/
}RCC_PLLInitTypeDef;

/*
 * @ref RCC_PLL_Config PLL Config
 */
#define RCC_PLL_NONE                      0
#define RCC_PLL_OFF                       1
#define RCC_PLL_ON                        2
/*
 * @ref RCC_PLL_Clock_Source PLL Clock Source
 */
#define RCC_PLLSOURCE_HSI                0
#define RCC_PLLSOURCE_HSE                1
/*
 * @ref RCC_PLLP_Clock_Divider PLLP Clock Divider
 */
#define RCC_PLLP_DIV2                  0x00000002U
#define RCC_PLLP_DIV4                  0x00000004U
#define RCC_PLLP_DIV6                  0x00000006U
#define RCC_PLLP_DIV8                  0x00000008U


typedef struct
{
	uint8_t OSCTYPE; /*!< @ref RCC_Oscillator_Type >!*/
	uint8_t HSEState; /*!< @ref RCC_HSE_Config >!*/
	uint8_t LSEState; /*!<>!*/
	uint8_t HSIState; /*!< @ref RCC_HSI_Config >!*/
	uint16_t HSICalibrationValue;  /*!<value : 0x00 - 0x1F default is 0x10>!*/  
	uint8_t LSIState; /*!<>!*/
	RCC_PLLInitTypeDef PLL;
}RCC_OscInit_t;

/*
 * @ref RCC_Oscillator_Type Oscillator Type
 */
#define RCC_OSCILLATORTYPE_NONE            0x00000000U
#define RCC_OSCILLATORTYPE_HSE             0x00000001U
#define RCC_OSCILLATORTYPE_HSI             0x00000002U
#define RCC_OSCILLATORTYPE_LSE             0x00000004U
#define RCC_OSCILLATORTYPE_LSI             0x00000008U
/*
 *	@ref RCC_HSE_Config
 */
#define RCC_HSE_OFF                      0
#define RCC_HSE_ON                       1
#define RCC_HSE_BYPASS                   2
/*
 *	@ref RCC_HSI_Config
 */
#define RCC_HSI_OFF                    0
#define RCC_HSI_ON                     1
#define RCC_HSI_BYPASS                 2
/*
 *	@ref RCC_LSE_Config
 */
#define RCC_LSE_OFF                    0
#define RCC_LSE_ON                     1
#define RCC_LSE_BYPASS                 2
/*
 *	@ref RCC_LSI_Config
 */
#define RCC_LSI_OFF                    0x00000000U
#define RCC_LSI_ON                     1
#define RCC_LSI_BYPASS                 2

typedef struct
{
  uint8_t ClockType;             /*!<  @ref RCC_System_Clock_Type      */
  uint8_t SYSCLKSource;          /*!<  @ref RCC_System_Clock_Source    */
  uint8_t AHBCLKDivider;         /*!<  @ref RCC_AHB_Clock_Source       */
  uint8_t APB1CLKDivider;        /*!<  @ref RCC_APB1_APB2_Clock_Source */
  uint8_t APB2CLKDivider;        /*!<  @ref RCC_APB1_APB2_Clock_Source */
} RCC_ClkInitTypeDef;

/*
 * @ref RCC_System_Clock_Source System Clock Source
 */
#define RCC_SYSCLKSOURCE_HSI             0
#define RCC_SYSCLKSOURCE_HSE             1
#define RCC_SYSCLKSOURCE_PLLCLK          2
/* 
 * @ref RCC_System_Clock_Type System Clock Type
 */
#define RCC_CLOCKTYPE_SYSCLK             1
#define RCC_CLOCKTYPE_HCLK               2
#define RCC_CLOCKTYPE_PCLK1              4
#define RCC_CLOCKTYPE_PCLK2              8
/*
 * @ref RCC_AHB_Clock_Source AHB Clock Source
 */
#define RCC_SYSCLK_DIV1                  0
#define RCC_SYSCLK_DIV2                  8
#define RCC_SYSCLK_DIV4                  9
#define RCC_SYSCLK_DIV8                  10
#define RCC_SYSCLK_DIV16                 11
#define RCC_SYSCLK_DIV64                 12
#define RCC_SYSCLK_DIV128                13
#define RCC_SYSCLK_DIV256                14
#define RCC_SYSCLK_DIV512                15

/*
 * @ref RCC_APB1_APB2_Clock_Source APB1/APB2 Clock Source
 */
#define RCC_HCLK_DIV1                    0
#define RCC_HCLK_DIV2                    4
#define RCC_HCLK_DIV4                    5
#define RCC_HCLK_DIV8                    6
#define RCC_HCLK_DIV16                   7


/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define RCC_GPIOA_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 0))
#define RCC_GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define RCC_GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define RCC_GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define RCC_GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define RCC_GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define RCC_GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define RCC_GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define RCC_GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))


/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define RCC_I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define RCC_I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define RCC_I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))
        

/*
 * Clock Enable Macros for SPIx peripheralsbu
 */
#define RCC_SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define RCC_SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define RCC_SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define RCC_SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))


/*
 * Clock Enable Macros for USARTx peripherals
 */
#define RCC_USART1_PCCK_EN() (RCC->APB2ENR |= (1 << 4))
#define RCC_USART2_PCCK_EN() (RCC->APB1ENR |= (1 << 17))
#define RCC_USART3_PCCK_EN() (RCC->APB1ENR |= (1 << 18))
#define RCC_UART4_PCCK_EN()  (RCC->APB1ENR |= (1 << 19))
#define RCC_UART5_PCCK_EN()  (RCC->APB1ENR |= (1 << 20))
#define RCC_USART6_PCCK_EN() (RCC->APB1ENR |= (1 << 5))
        
/*      
 * Clock Enable Macros for SYSCFG peripheral
 */
#define RCC_SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

/*      
 * Clock Enable Macros for PWR peripheral
 */
#define RCC_PWR_PCLK_EN()			(RCC->APB1ENR |= 1<<(RCC_APB1ENR_PWRRST))

/*
 * Clock Enable Macros for WWDG peripherals
 */
#define RCC_WWDG_PCLK_EN()		(RCC->APB1ENR |= 1 << RCC_APP1ENR_WWDGEN)


/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define RCC_GPIOA_PCLK_DI()

/*
 * Clock Disable Macros for SPIx peripherals
 */

/*
 * Clock Disable Macros for USARTx peripherals
 */


/*
 * Clock Disable Macros for SYSCFG peripheral
 */

/*
 * Clock Disable Macros for PWR peripheral
 */

#define RCC_PWR_PCLK_DI()			RCC->APB1ENR &= ~(1<<(RCC_APB1ENR_PWRRST))

/*
 *  Macros to reset GPIOx peripherals
 */
#define RCC_GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define RCC_GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define RCC_GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define RCC_GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define RCC_GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define RCC_GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define RCC_GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define RCC_GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define RCC_GPIOI_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)


#define RCC_WWDG_REG_RESET()		do{RCC->APB1RSTR |= 1 << RCC_APP1RSTR_WWDGEN ; RCC->APB1RSTR &= ~(1 << RCC_APP1RSTR_WWDGEN);}while(0)

#define RCC_PWR_REG_RESET() 	do{RCC->APB1RSTR |= 1<<(RCC_APB1RSTR_PWRRST); RCC->APB1RSTR &= ~(1<<(RCC_APB1RSTR_PWRRST));}while(0)



void RCC_OscConfig(RCC_OscInit_t *posc);
void RCC_ClockConfig(RCC_ClkInitTypeDef *Clk);

//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);

//
uint32_t  RCC_GetPLLOutputClock(void);

// 
uint32_t RCC_GetAPB1Timer(void);

// 
uint32_t RCC_GetAPB2Timer(void);

#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
