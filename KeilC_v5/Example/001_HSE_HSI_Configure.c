#if 0

#include <stdint.h>

#define RCC_BASE_ADDR 		0x40023800UL

#define RCC_CFGR_REG_OFFSET		0x08

#define RCC_CR_REG_OFFSET       0x00

#define RCC_CFGR_REG_ADDR		(RCC_BASE_ADDR + RCC_CFGR_REG_OFFSET)

#define RCC_CR_REG_ADDR			(RCC_BASE_ADDR + RCC_CR_REG_OFFSET)

#define GPIOA_BASE_ADDR			0x40020000UL

void HSI_Mesurement(void);
void HSE_Mesurement(void);
void GPIOA_PA8_AF_configure(void);

//int main(void)
//{


//	HSI_Configure();

//	GPIOA_PA8_AF_configure();

//	/* Loop forever */
//	while(1){}
//}

void HSI_Mesurement(void)
{
	uint32_t *pRCCCFGRreg = (uint32_t *)RCC_CFGR_REG_ADDR;

	// 1. configure the RCC_CFGR MCO1 bit fields to select HSI clock source.
	*pRCCCFGRreg &= ~(0x3<<21);

	// configure MCO1 prescaler
	*pRCCCFGRreg |= (1<<24);
}

void HSE_Mesurement(void)
{
	uint32_t *pRCCCRreg = (uint32_t *)RCC_CR_REG_ADDR;
	uint32_t *pRCCCFGRreg = (uint32_t *)RCC_CFGR_REG_ADDR;
	// 1. enable the HSE clock using HSEON bit
	*pRCCCRreg |= (1 << 16);
	// 2. wait until HSE clock from the external crystal ready
	while(!(*pRCCCRreg & (1<<17))){}
	// 3. switch the system clock to HSE
	*pRCCCFGRreg |= (1 << 0);
	// 4.
	// 1. configure the RCC_CFGR MCO1 bit fields to select HSI clock source.
	*pRCCCFGRreg &= ~(0x3<<21);
	*pRCCCFGRreg |= (0x2<<21);
	// configure MCO1 prescaler
	*pRCCCFGRreg |= (1<<24);
}

void GPIOA_PA8_AF_configure(void)
{
	// 2. configure FA8 to AF0 mode to behave as MCO1 signal
	uint32_t *pRCCAHB1ENR = (uint32_t *)(RCC_BASE_ADDR + 0x30);
	*pRCCAHB1ENR |= (1<<0); // enable GPIOA peri

	// configure the mode of GPIOA pin 8 as AF function mode
	uint32_t *pGPIOModereg = (uint32_t *)(GPIOA_BASE_ADDR);
	*pGPIOModereg &= (0x3 << 16);
	*pGPIOModereg |= (0x3 << 16);

	// configure the AF function register to set mode 0 for PA8
	uint32_t *pGPIOAAFreg = (uint32_t *)(GPIOA_BASE_ADDR + 0x24);
	*pGPIOAAFreg &= ~(0xf << 0);
}

#endif