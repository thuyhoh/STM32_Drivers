///*
// * stm32f407xx_rcc_driver.c
// *
// *  Created on: Mar 29, 2019
// *      Author: admin
// */


#include "stm32f407xx_rcc_driver.h"


uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = { 2, 4 , 8, 16};



void RCC_OscConfig(RCC_OscInit_t *Osc)
{
/*----------------- HSE Configure ---------------------*/
	if( ((RCC->CFGR & (0x1<<RCC_CFGR_SWS)) == (0x1 << RCC_CFGR_SWS)) || 
		( ((RCC->CFGR & (0x2<<RCC_CFGR_SWS)) == (0x2 << RCC_CFGR_SWS)) && ((RCC->PLLCFGR & (1<<RCC_PLLCFGR_PLLSRC)) == (1<<RCC_PLLCFGR_PLLSRC))) )
	{
		// hse is used for system clock
	}
	else // config hse
	{
		if(Osc->HSEState != RCC_HSE_OFF)
		{
			if(Osc->HSEState == RCC_HSE_BYPASS)
				{
					RCC->CR |= 1 << RCC_CR_HSEBYP;
				}
			RCC->CR |= 1 << RCC_CR_HSEON;
			while(!(RCC->CR & (1<<RCC_CR_HSERDY))); 
		}else
		{
			// disable hse
			RCC->CR &= ~(1<<RCC_CR_HSEON);
			while((RCC->CR & (1<<RCC_CR_HSERDY)));
		}
	}
	
/*----------------- HSI Configure ---------------------*/
	if( ((RCC->CFGR & (0x0<<RCC_CFGR_SWS)) == (0 << RCC_CFGR_SWS))   || 
		( ((RCC->CFGR & (0x0<<RCC_CFGR_SWS)) == (0x2 << RCC_CFGR_SWS)) && (((RCC->PLLCFGR & (1<<22)) == RCC_PLLCFGR_PLLSRC))))
	{
		// hsi is used for system clock
	}else // config hsi
	{
		if(Osc->HSIState != RCC_HSI_OFF)
		{			
			RCC->CR |= 1<<RCC_CR_HSION;
			while(!(RCC->CR &(1<<RCC_CR_HSIRDY)));
		}else
		{
			// disable hsi
			RCC->CR |= 1<<RCC_CR_HSION;
			while(!(RCC->CR &(1<<RCC_CR_HSIRDY)));
		}
	}
/*----------------- LSE Configure ---------------------*/
/*----------------- LSI Configure ---------------------*/
/*----------------- PLL Configure ---------------------*/	
	uint32_t tmpreg = 0;
	if(Osc->PLL.PLLState != RCC_PLL_NONE)
	{
		if(RCC->CFGR & (0x2 << RCC_CFGR_SWS))
		{
			// pll is used for system clock
			if(Osc->PLL.PLLState == RCC_PLL_ON)
			{
				// disable PLL
				RCC->CR &= ~(1<<RCC_CR_PLLON);
				while(RCC->CR &(1<<RCC_CR_PLLRDY));// waiting PLL is locked
				// config PLLCFGR
				//PLL source
				tmpreg = Osc->PLL.PLLSource << RCC_PLLCFGR_PLLSRC;  
				// pll m
				tmpreg |= Osc->PLL.PLLM << RCC_PLLCFGR_PLLM;
				// pll n
				if(Osc->PLL.PLLN <= 432&& Osc->PLL.PLLN >= 50)
				{
					tmpreg |= Osc->PLL.PLLN << RCC_PLLCFGR_PLLN;
				}else
				{
					/*error*/
				}
				// pll p
				tmpreg |= Osc->PLL.PLLP << RCC_PLLCFGR_PLLP;
				// pll q
				tmpreg |= Osc->PLL.PLLQ << RCC_PLLCFGR_PLLQ;
				RCC->PLLCFGR = tmpreg;
				// enable pll
				RCC->CR |= 1<<RCC_CR_PLLON;
				// waiting pll is unlocked
				while(!(RCC->CR & (1<<RCC_CR_PLLRDY)));
			}
			else
			{
				// disable  pll
				RCC->CR &= ~(1<<RCC_CR_PLLON);
				while(RCC_CR_CSSON &(1<<RCC_CR_PLLRDY));// waiting PLL is locked
			}
		}else
		{
			// 
		}
	}	
}


void RCC_ClockConfig(RCC_ClkInitTypeDef *Clk)
{
/*-------------------------- HCLK Configuration --------------------------*/
	if (((Clk->ClockType) & RCC_CLOCKTYPE_HCLK) == RCC_CLOCKTYPE_HCLK)
	{
		if(((Clk->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
		{
			RCC->CFGR |= RCC_HCLK_DIV16 << RCC_CFGR_PPRE1;
		}
		if(((Clk->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
		{
			RCC->CFGR |= RCC_HCLK_DIV16 << RCC_CFGR_PPRE2;
		}
		RCC->CFGR = Clk->AHBCLKDivider << RCC_CFGR_HPRE;
	}
/*------------------------- SYSCLK Configuration ---------------------------*/
	if (((Clk->ClockType) & RCC_CLOCKTYPE_SYSCLK) == RCC_CLOCKTYPE_SYSCLK)
	{
		/* HSE is selected as System Clock Source */
		if(Clk->SYSCLKSource == RCC_SYSCLKSOURCE_HSE)
		{
			if(!(RCC->CR & 1<<RCC_CR_HSERDY))
			{
				/* error */
			}
		}
		/* PLL is selected as System Clock Source */
		if(Clk->SYSCLKSource  == RCC_SYSCLKSOURCE_PLLCLK)
		{
			if(!(RCC->CR &(1<<RCC_CR_PLLRDY)))
			{
				/* error */
			}
		}
		/* HSI is selected as System Clock Source */
		if(Clk->SYSCLKSource == RCC_SYSCLKSOURCE_HSI)
		{
			if(!(RCC->CR & (1<< RCC_CR_HSIRDY)))
			{
				/* error */ 
			}
		}
	}
	RCC->CFGR &= ~(0x3 << RCC_CFGR_SW);
	RCC->CFGR |= (Clk->SYSCLKSource << RCC_CFGR_SW);
	while((RCC->CFGR & (Clk->SYSCLKSource << RCC_CFGR_SWS)) == (Clk->SYSCLKSource << RCC_CFGR_SWS));
/*-------------------------- PCLK1 Configuration ---------------------------*/
	if (((Clk->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
	{
		RCC->CFGR &= (Clk->APB1CLKDivider << RCC_CFGR_PPRE1);
	}
/*-------------------------- PCLK2 Configuration ---------------------------*/
	if (((Clk->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
	{
		RCC->CFGR &= (Clk->APB2CLKDivider << RCC_CFGR_PPRE2);
	}
}

uint32_t RCC_GetSysClockFreq(void)
{
	
}



uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1,SystemClk;

	uint8_t clksrc,temp,ahbp,apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0 )
	{
		SystemClk = 16000000;
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}else if (clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	//for ahb
	temp = ((RCC->CFGR >> 4 ) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}



	//apb1
	temp = ((RCC->CFGR >> 10 ) & 0x7);

	if(temp < 4)
	{
		apb1p = 1;
	}else
	{
		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 =  (SystemClk / ahbp) /apb1p;

	return pclk1;
}



/*********************************************************************
 * @fn      		  - RCC_GetPCLK2Value
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t SystemClock=0,tmp,pclk2;
	uint8_t clk_src = ( RCC->CFGR >> 2) & 0X3;

	uint8_t ahbp,apb2p;

	if(clk_src == 0)
	{
		SystemClock = 16000000;
	}else
	{
		SystemClock = 8000000;
	}
	tmp = (RCC->CFGR >> 4 ) & 0xF;

	if(tmp < 0x08)
	{
		ahbp = 1;
	}else
	{
       ahbp = AHB_PreScaler[tmp-8];
	}

	tmp = (RCC->CFGR >> 13 ) & 0x7;
	if(tmp < 0x04)
	{
		apb2p = 1;
	}else
	{
		apb2p = APB1_PreScaler[tmp-4];
	}

	pclk2 = (SystemClock / ahbp )/ apb2p;

	return pclk2;
}

uint32_t  RCC_GetPLLOutputClock()
{

	return 0;
}

// 
uint32_t RCC_GetAPB1Timer(void)
{
	uint32_t APB1TimerClock = RCC_GetPCLK1Value();
	if(((RCC->CFGR >> RCC_CFGR_PPRE1) & 0x7) >= 0x4)
	{
		APB1TimerClock = APB1TimerClock * 2; 
	}
	return APB1TimerClock;
}

// 
uint32_t RCC_GetAPB2Timer(void)
{
	uint32_t APB2TimerClock = RCC_GetPCLK2Value();
	if(((RCC->CFGR >> RCC_CFGR_PPRE2) & 0x7) >= 0x4)
	{
		APB2TimerClock = APB2TimerClock * 2;
	}
	return APB2TimerClock;
}
