				 /*
 * stm3f407xx.h
 *
 *  Created on: Jan 29, 2019
 *      Author: admin
 */

#ifndef INC_STM3F407XX_H_
#define INC_STM3F407XX_H_

#include<stddef.h>
#include<stdint.h>
#include "CortexM4.h"



/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4

/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR						0x08000000U   		/*!<explain this macro briefly here  */
#define SRAM1_BASEADDR						0x20000000U  		/*!<explain this macro briefly here  */
#define SRAM2_BASEADDR						0x2001C000U 		/*!<explain this macro briefly here  */
#define ROM_BASEADDR						  0x1FFF0000U
#define SRAM 								      SRAM1_BASEADDR


/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR 						0x40000000U
#define APB1PERIPH_BASEADDR						PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR						0x40010000U
#define AHB1PERIPH_BASEADDR						0x40020000U
#define AHB2PERIPH_BASEADDR						0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * TODO : Complete for all other peripherals
 */

#define GPIOA_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 					 				 (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 					 				 (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 					 				 (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 					 				 (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 					 				 (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 					 				 (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR 					 				 (AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR                     (AHB1PERIPH_BASEADDR + 0x3800)
#define FLASH_REG_BASEADDR               (AHB1PERIPH_BASEADDR + 0x3C00)
#define BKPSRAM										       (AHB1PERIPH_BASEADDR + 0x4000)
/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO : Complete for all other peripherals
 */

#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x5000)

#define TIM2_BASEADDR							(APB1PERIPH_BASEADDR + 0x0000)
#define TIM3_BASEADDR							(APB1PERIPH_BASEADDR + 0x0400)
#define TIM4_BASEADDR							(APB1PERIPH_BASEADDR + 0x0800)
#define TIM5_BASEADDR							(APB1PERIPH_BASEADDR + 0x0C00)
#define TIM6_BASEADDR							(APB1PERIPH_BASEADDR + 0x1000)
#define TIM7_BASEADDR							(APB1PERIPH_BASEADDR + 0x1400)
#define TIM12_BASEADDR						(APB1PERIPH_BASEADDR + 0x1800)
#define TIM13_BASEADDR						(APB1PERIPH_BASEADDR + 0x1C00)
#define TIM14_BASEADDR						(APB1PERIPH_BASEADDR + 0x2000)

#define IWDG_BASEADDR							(APB1PERIPH_BASEADDR + 0x3000)
#define WWDG_BASEADDR							(APB1PERIPH_BASEADDR + 0x2C00)

#define PWR_BASEADDR 						  (APB1PERIPH_BASEADDR + 0x7000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO : Complete for all other peripherals
 */

#define EXTI_BASEADDR						  (APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR						  (APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR        		(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)

#define TIM11_BASEADDR					  (APB2PERIPH_BASEADDR + 0x4800)
#define TIM10_BASEADDR					  (APB2PERIPH_BASEADDR + 0x4400)
#define TIM9_BASEADDR						  (APB2PERIPH_BASEADDR + 0x4000)
#define TIM8_BASEADDR						  (APB2PERIPH_BASEADDR + 0x0400)
#define TIM1_BASEADDR						  (APB2PERIPH_BASEADDR + 0x0000)


/**********************************peripheral register definition structures **********************************/

/*
 * Note : Registers of a peripheral are specific to MCU
 * e.g : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 */

typedef struct
{
	__vo uint32_t MODER;                        /*!< GPIO port mode register,                    	Address offset: 0x00      */
	__vo uint32_t OTYPER;                       /*!< TODO,     										Address offset: 0x04      */
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];					 /*!< AFR[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register    		Address offset: 0x20-0x24 */
}GPIO_RegDef_t;



/*
 * peripheral register definition structure for RCC
 */
typedef struct
{
  __vo uint32_t CR;            /*!< TODO,     										Address offset: 0x00 */
  __vo uint32_t PLLCFGR;       /*!< TODO,     										Address offset: 0x04 */
  __vo uint32_t CFGR;          /*!< TODO,     										Address offset: 0x08 */
  __vo uint32_t CIR;           /*!< TODO,     										Address offset: 0x0C */
  __vo uint32_t AHB1RSTR;      /*!< TODO,     										Address offset: 0x10 */
  __vo uint32_t AHB2RSTR;      /*!< TODO,     										Address offset: 0x14 */
  __vo uint32_t AHB3RSTR;      /*!< TODO,     										Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                       */
  __vo uint32_t APB1RSTR;      /*!< TODO,     										Address offset: 0x20 */
  __vo uint32_t APB2RSTR;      /*!< TODO,     										Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
  __vo uint32_t AHB1ENR;       /*!< TODO,     										Address offset: 0x30 */
  __vo uint32_t AHB2ENR;       /*!< TODO,     										Address offset: 0x34 */
  __vo uint32_t AHB3ENR;       /*!< TODO,     										Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
  __vo uint32_t APB1ENR;       /*!< TODO,     										Address offset: 0x40 */
  __vo uint32_t APB2ENR;       /*!< TODO,     										Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
  __vo uint32_t AHB1LPENR;     /*!< TODO,     										Address offset: 0x50 */
  __vo uint32_t AHB2LPENR;     /*!< TODO,     										Address offset: 0x54 */
  __vo uint32_t AHB3LPENR;     /*!< TODO,     										Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
  __vo uint32_t APB1LPENR;     /*!< TODO,     										Address offset: 0x60 */
  __vo uint32_t APB2LPENR;     /*!< RTODO,     										Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
  __vo uint32_t BDCR;          /*!< TODO,     										Address offset: 0x70 */
  __vo uint32_t CSR;           /*!< TODO,     										Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
  __vo uint32_t SSCGR;         /*!< TODO,     										Address offset: 0x80 */
  __vo uint32_t PLLI2SCFGR;    /*!< TODO,     										Address offset: 0x84 */
  __vo uint32_t PLLSAICFGR;    /*!< TODO,     										Address offset: 0x88 */
  __vo uint32_t DCKCFGR;       /*!< TODO,     										Address offset: 0x8C */
  __vo uint32_t CKGATENR;      /*!< TODO,     										Address offset: 0x90 */
  __vo uint32_t DCKCFGR2;      /*!< TODO,     										Address offset: 0x94 */

} RCC_RegDef_t;



/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;    /*!< Give a short description,          	  	    Address offset: 0x00 */
	__vo uint32_t EMR;    /*!< TODO,                						Address offset: 0x04 */
	__vo uint32_t RTSR;   /*!< TODO,  									     Address offset: 0x08 */
	__vo uint32_t FTSR;   /*!< TODO, 										Address offset: 0x0C */
	__vo uint32_t SWIER;  /*!< TODO,  									   Address offset: 0x10 */
	__vo uint32_t PR;     /*!< TODO,                   					   Address offset: 0x14 */

}EXTI_RegDef_t;


/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t CRCPR;      /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t RXCRCR;     /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t TXCRCR;     /*!< TODO,     										Address offset: 0x18 */
	__vo uint32_t I2SCFGR;    /*!< TODO,     										Address offset: 0x1C */
	__vo uint32_t I2SPR;      /*!< TODO,     										Address offset: 0x20 */
} SPI_RegDef_t;


/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;       /*!< Give a short description,                    Address offset: 0x00      */
	__vo uint32_t PMC;          /*!< TODO,     									  Address offset: 0x04      */
	__vo uint32_t EXTICR[4];    /*!< TODO , 									  Address offset: 0x08-0x14 */
	uint32_t      RESERVED1[2];  /*!< TODO          							  Reserved, 0x18-0x1C    	*/
	__vo uint32_t CMPCR;        /*!< TODO         								  Address offset: 0x20      */
	uint32_t      RESERVED2[2];  /*!<                                             Reserved, 0x24-0x28 	    */
	__vo uint32_t CFGR;         /*!< TODO                                         Address offset: 0x2C   	*/
} SYSCFG_RegDef_t;


/*
 * peripheral register definition structure for I2C
 */
typedef struct
{
  __vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
  __vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
  __vo uint32_t OAR1;       /*!< TODO,     										Address offset: 0x08 */
  __vo uint32_t OAR2;       /*!< TODO,     										Address offset: 0x0C */
  __vo uint32_t DR;         /*!< TODO,     										Address offset: 0x10 */
  __vo uint32_t SR1;        /*!< TODO,     										Address offset: 0x14 */
  __vo uint32_t SR2;        /*!< TODO,     										Address offset: 0x18 */
  __vo uint32_t CCR;        /*!< TODO,     										Address offset: 0x1C */
  __vo uint32_t TRISE;      /*!< TODO,     										Address offset: 0x20 */
  __vo uint32_t FLTR;       /*!< TODO,     										Address offset: 0x24 */
}I2C_RegDef_t;

/*
 * peripheral register definition structure for USART
 */
typedef struct
{
	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t BRR;        /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t CR3;        /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t GTPR;       /*!< TODO,     										Address offset: 0x18 */
}USART_RegDef_t;


/*
 * peripheral register definition structure for USART
 */
typedef struct
{
	__vo uint32_t CR1;			/*!<Address offset: 0x00 */
	__vo uint32_t CR2;      /*!<Address offset: 0x04 */
	__vo uint32_t SMCR;     /*!<Address offset: 0x08 */
	__vo uint32_t DIER;     /*!<Address offset: 0x0C */
	__vo uint32_t SR;       /*!<Address offset: 0x10 */
	__vo uint32_t EGR;      /*!<Address offset: 0x14 */
	__vo uint32_t CCMPR1;   /*!<Address offset: 0x18 */
	__vo uint32_t CCMPR2;		/*!<Address offset: 0x20 */
	__vo uint32_t CCENR;    /*!<Address offset: 0x24 */
	__vo uint32_t CNT;      /*!<Address offset: 0x28 */
	__vo uint32_t PSC;      /*!<Address offset: 0x2C */
	__vo uint32_t ARR;      /*!<Address offset: 0x30 */
	__vo uint32_t RCR;      /*!<Address offset: 0x34 */
	__vo uint32_t CCR[4];   /*!<Address offset: .... */
	__vo uint32_t BDTR;			/*!<Address offset: 0x44 */
	__vo uint32_t DCR;      /*!<Address offset: 0x48 */
	__vo uint32_t DMAR;			/*!<Address offset: 0x4C */
	                        
}TIM_RegDef_t;

typedef struct
{
	uint32_t CR;
	uint32_t CFR;
	uint32_t SR;
}WWDG_RegDef_t;

typedef struct
{
	uint32_t KR;
	uint32_t PR;
	uint32_t RLR;
	uint32_t SR;
}IWDG_RegDef_t;

typedef struct
{
	uint32_t CR;
	uint32_t CSR;
}PWR_RegDef_t;

typedef struct
{
	uint32_t ADC_SR;
	uint32_t ADC_CR1;
	uint32_t ADC_CR2;
	uint32_t ADC_SMPR1;
	uint32_t ADC_SMPR2;
	uint32_t ADC_JOFR1;
	uint32_t ADC_JOFR2;
	uint32_t ADC_JOFR3;
	uint32_t ADC_JOFR4;
	uint32_t ADC_HTR;
	uint32_t ADC_LTR;
	uint32_t ADC_SQR1;
	uint32_t ADC_SQR2;
	uint32_t ADC_SQR3;
	uint32_t ADC_JSQR;
	uint32_t ADC_JDR1;
	uint32_t ADC_JDR2;
	uint32_t ADC_JDR3;
	uint32_t ADC_JDR4;
	uint32_t ADC_DR;
	uint32_t ADC_CSR;
	uint32_t ADC_CCR;
	uint32_t ADC_CDR;
}ADC_RegDef_t;

//typedef struct
//{
//	CAN_MCR
//	CAN_MSR
//	CAN_TSR
//	CAN_RF0R
//	CAN_RF1R
//	CAN_IER
//	CAN_ESR
//	CAN_BTR
//	Reserved1[28];
//	CAN_TI0R
//	CAN_TDT0R
//	CAN_TDL0R
//	CAN_TDH0R
//	CAN_TI1R
//	CAN_TDT1R
//	CAN_TDL1R
//	CAN_TDH1R
//	CAN_TI2R
//	CAN_TDT2R
//	CAN_TDL2R
//	CAN_TDH2R
//	CAN_RI0R
//	CAN_RDT0R
//	CAN_RDL0R
//	CAN_RDH0R
//	CAN_RI1R
//	CAN_RDT1R
//	CAN_RDL1R
//	CAN_RDH1R
//	Reserved2[12]
//	CAN_FMR
//	CAN_FM1R
//	Reserved3;
//	CAN_FS1R
//	Reserved3
//	CAN_FFA1R
//	Reserved
//	CAN_FA1R
//	Reserved[8]
//	CAN_F0R1
//	CAN_F0R2
//	CAN_F1R1
//	CAN_F1R2
//	...
//	CAN_F27R1
//	CAN_F27R2
//}bxCAN_RegDef_t;

typedef struct
{
	__vo uint32_t ACR;
	__vo uint32_t KEYR;
	__vo uint32_t OPTKEYR;
	__vo uint32_t SR;
	__vo uint32_t CR;
	__vo uint32_t OPTCR;
}FLASH_RegDef_t;

/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA  				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI  				((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


#define SPI1  				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2  				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3  				((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1  				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2  				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3  				((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1  			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  			((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  			((USART_RegDef_t*)USART3_BASEADDR)
#define UART4  				((USART_RegDef_t*)UART4_BASEADDR)
#define UART5  				((USART_RegDef_t*)UART5_BASEADDR)
#define USART6  			((USART_RegDef_t*)USART6_BASEADDR)

#define TIM1					((TIM_RegDef_t *)TIM1_BASEADDR )
#define TIM2					((TIM_RegDef_t *)TIM2_BASEADDR )
#define TIM3          ((TIM_RegDef_t *)TIM3_BASEADDR )
#define TIM4          ((TIM_RegDef_t *)TIM4_BASEADDR )
#define TIM5          ((TIM_RegDef_t *)TIM5_BASEADDR )
#define TIM6          ((TIM_RegDef_t *)TIM6_BASEADDR )
#define TIM7          ((TIM_RegDef_t *)TIM7_BASEADDR )
#define TIM8          ((TIM_RegDef_t *)TIM8_BASEADDR )
#define TIM9          ((TIM_RegDef_t *)TIM9_BASEADDR )
#define TIM10         ((TIM_RegDef_t *)TIM10_BASEADDR)
#define TIM11					((TIM_RegDef_t *)TIM11_BASEADDR)
#define TIM12         ((TIM_RegDef_t *)TIM12_BASEADDR) 
#define TIM13         ((TIM_RegDef_t *)TIM13_BASEADDR)
#define TIM14         ((TIM_RegDef_t *)TIM14_BASEADDR)
       
#define IWDG					((IWDG_RegDef_t *) IWDG_BASEADDR)
#define WWDG					((WWDG_RegDef_t *) WWDG_BASEADDR)			 
#define PWR						((PWR_RegDef_t *) PWR_BASEADDR)

#define FLASH					((FLASH_RegDef_t*) FLASH_REG_BASEADDR)


/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV     31
#define IRQ_NO_I2C1_ER     32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71


/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15


//some generic macros

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET         RESET
#define FLAG_SET 			SET



#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_uart_driver.h"
#include "stm32f407xx_rcc_driver.h"
#include "stm32f407xx_exti_driver.h"

#endif /* INC_STM3F407XX_H_ */