/*
 *  file name  : CortexM4.h
 *  Created on : Aug 23, 2024
 *  Author     : THUY
 */

#ifndef  __CORTEXM4_H_
#define  __CORTEXM4_H_

#include "stdint.h"
#define __vo volatile
#define __weak __attribute__((weak))
#define __naked __attribute__((naked))


/******************************************************************************************************************************
																								BIT BANDING
******************************************************************************************************************************/

#define SRAM_BASEADDR																				0x20000000UL /*!< SRAM BASE ADDRESS >*/
#define SRAM_ALIAS_BASEADDR																	0x22000000U

#define PERI_BASEADDR																				0x40000000UL /*!< PHERIPHERAL BASE ADDRESS >*/
#define PERI_ALIAS_BASEADDR																	0x42000000UL

#define ALIAS_SRAM_ADDRESS(SRAM_addr, bitNo)							 	(SRAM_ALIAS_BASEADDR + (((SRAM_addr) - SRAM_BASEADDR)*32) + ((bitNo) * 4))
#define ALIAS_PERI_ADDRESS(PERI_addr, bitNo)				 			 	(PERI_ALIAS_BASEADDR + (((PERI_addr) - PERI_BASEADDR)*32) + ((bitNo) * 4))

#define ALIAS_SRAM_POINTER(SRAM_addr, bitNo)								(uint8_t *)ALIAS_SRAM_ADDRESS(SRAM_addr, bitNo)
#define ALIAS_PERI_POINTER(PERI_addr, bitNo)								(uint8_t *)ALIAS_PERI_ADDRESS(PERI_addr, bitNo)

/* Macro of Bit Status */
#define SET_BIT																							0x01
#define RESET_BIT																						0x00

#define BITBAND_SRAM_SET_BIT(SRAM_addr, bitNo)							*(ALIAS_SRAM_POINTER(SRAM_addr, bitNo)) = SET_BIT
#define BITBAND_SRAM_RESET_BIT(SRAM_addr, bitNo)						*(ALIAS_SRAM_POINTER(SRAM_addr, bitNo)) = RESET_BIT

#define BITBAND_PERI_SET_BIT(PERI_addr, bitNo)							*(ALIAS_PERI_POINTER(PERI_addr, bitNo)) = SET_BIT
#define BITBAND_PERI_RESET_BIT(PERI_addr, bitNo) 						*(ALIAS_PERI_POINTER(PERI_addr, bitNo)) = RESET_BIT

/******************************************************************************************************************************
																								System control block (SCB)
******************************************************************************************************************************/
#define SCB_BASEADDR								 0xE000ED00

typedef struct
{
	__vo uint32_t CPUID;
	__vo uint32_t ICSR;
	__vo uint32_t VTOR;
	__vo uint32_t AIRCR;
	__vo uint32_t SCR;
	__vo uint32_t CCR;
	__vo uint32_t SHPR1;
	__vo uint32_t SHPR2;
	__vo uint32_t SHPR3;
	__vo uint32_t SHCSR;
	__vo uint32_t CFSR;
	__vo uint32_t HFSR;
	__vo uint32_t MMAR;
	__vo uint32_t BFAR;
	__vo uint32_t AFSR;
}SCB_RegDef_t;

#define SCB					((SCB_RegDef_t *)SCB_BASEADDR)

#define SCB_SCR_SLEEPONEXIT 			1
#define SCB_SCR_SLEEPDEEP					2
#define SCB_SCR_SEVONPEND					4

#define SLEEPONEXIT()							SCB->SCR |= 0x1<<SCB_SCR_SLEEPONEXIT
#define SLEEPDEEP() 							SCB->SCR |= 0x1<<SCB_SCR_SLEEPDEEP
#define Reset_SLEEPDEEP()					SCB->SCR &= ~(0x1<<SCB_SCR_SLEEPDEEP)
/******************************************************************************************************************************
																								Systick Timer (STK)
******************************************************************************************************************************/

#define SYSTICK_BASEADDR																			0xE000E010u

typedef struct 
{
	uint32_t CTRL;
	uint32_t LOAD;
	uint32_t VAL;
	uint32_t CALIB;
}SysTick_RegDef_t;

/******************************************************************************************************************************
																										NVIC
******************************************************************************************************************************/

/* base address of NVIC */
#define NVIC_ICTR_BASE																	((__vo uint32_t *)0xE000E004U)
 
#define NVIC_ISER_BASE																	((__vo uint32_t *)0xE000E100U)

#define NVIC_ICER_BASE																	((__vo uint32_t *)0xE000E180U)

#define NVIC_ISPR_BASE																	((__vo uint32_t *)0xE000E200U)

#define NVIC_ICPR_BASE																	((__vo uint32_t *)0xE000E280U)

#define NVIC_IABR_BASE																	((__vo uint32_t *)0xE000E300U)

#define NVIC_IPR_BASE																		((__vo uint32_t *)0xE000E400U)	

/* NVIC MACRO */
#define IRQ_ENABLE					1
#define IRQ_DISABLE					0

#define NO_PR_BITS_IMPLEMENTED   4			/*number of prority bits of irq */ /* the total bits microtroller used to descri the priority of IRQ  */


#define INTERRUPT_DISABLE()  do{__ASM volatile ("MOV R0,#0x1"); __ASM volatile("MSR PRIMASK,R0"); } while(0)

#define INTERRUPT_ENABLE()   do{__ASM volatile ("MOV R0,#0x0"); __ASM volatile("MSR PRIMASK,R0"); } while(0)

/* IRQ Configuration and ISR handling */

void NVIC_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);

void NVIC_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

uint8_t NVIC_IRQGetPriority(uint8_t IRQNumber);


/* Vector Table */
#define IRQ0          		0
#define IRQ1          		1
#define IRQ2          		2
#define IRQ3          		3
#define IRQ4          		4
#define IRQ5          		5
#define IRQ6          		6
#define IRQ7          		7
#define IRQ8          		8
#define IRQ9          		9
#define IRQ10          		10
#define IRQ11          		11
#define IRQ12          		12
#define IRQ13          		13
#define IRQ14          		14
#define IRQ15          		15
#define IRQ16          		16
#define IRQ17          		17
#define IRQ18          		18
#define IRQ19          		19
#define IRQ20          		20
#define IRQ21          		21
#define IRQ22          		22
#define IRQ23          		23
#define IRQ24          		24
#define IRQ25          		25
#define IRQ26          		26
#define IRQ27          		27
#define IRQ28          		28
#define IRQ29          		29
#define IRQ30          		30
#define IRQ31          		31
#define IRQ32          		32
#define IRQ33          		33
#define IRQ34          		34
#define IRQ35          		35
#define IRQ36          		36
#define IRQ37          		37
#define IRQ38          		38
#define IRQ39          		39
#define IRQ40          		40	
#define IRQ41          		41	
#define IRQ42          		42	
#define IRQ43          		43	
#define IRQ44          		44	
#define IRQ45          		45	
#define IRQ46          		46	
#define IRQ47          		47	
#define IRQ48          		48	
#define IRQ49          		49	
#define IRQ50          		50
#define IRQ51          		51
#define IRQ52          		52
#define IRQ53          		53
#define IRQ54          		54
#define IRQ55          		55
#define IRQ56          		56
#define IRQ57          		57
#define IRQ58          		58
#define IRQ59          		59
#define IRQ60          		60	
#define IRQ61          		61	
#define IRQ62          		62	
#define IRQ63          		63	
#define IRQ64          		64	
#define IRQ65          		65	
#define IRQ66          		66	
#define IRQ67          		67	
#define IRQ68          		68	
#define IRQ69          		69	
#define IRQ70          		70
#define IRQ71          		71
#define IRQ72          		72
#define IRQ73          		73
#define IRQ74          		74
#define IRQ75          		75
#define IRQ76          		76
#define IRQ77          		77
#define IRQ78          		78
#define IRQ79          		79
#define IRQ80          		80
#define IRQ81          		81
#define IRQ82          		82
#define IRQ83          		83
#define IRQ84          		84
#define IRQ85          		85
#define IRQ86          		86
#define IRQ87          		87
#define IRQ88          		88
#define IRQ89          		89
#define IRQ90          		90
#define IRQ91          		91
#define IRQ92          		92
#define IRQ93          		93
#define IRQ94          		94
#define IRQ95          		95
#define IRQ96          		96
#define IRQ97          		97
#define IRQ98          		98
#define IRQ99          		99
#define IRQ100          	100
#define IRQ101          	101
#define IRQ102          	102
#define IRQ103          	103
#define IRQ104          	104
#define IRQ105          	105
#define IRQ106          	106
#define IRQ107          	107
#define IRQ108          	108
#define IRQ109          	109
#define IRQ110          	110
#define IRQ111          	111
#define IRQ112          	112
#define IRQ113          	113
#define IRQ114          	114
#define IRQ115          	115
#define IRQ116          	116
#define IRQ117          	117
#define IRQ118          	118
#define IRQ119          	119
#define IRQ120          	120
#define IRQ121          	121
#define IRQ122          	122
#define IRQ123          	123
#define IRQ124          	124
#define IRQ125          	125
#define IRQ126          	126
#define IRQ127          	127
#define IRQ128          	128
#define IRQ129          	129
#define IRQ130          	130
#define IRQ131          	131
#define IRQ132          	132
#define IRQ133          	133
#define IRQ134          	134
#define IRQ135          	135
#define IRQ136          	136
#define IRQ137          	137
#define IRQ138          	138
#define IRQ139          	139
#define IRQ140          	140
#define IRQ141          	141
#define IRQ142          	142
#define IRQ143          	143
#define IRQ144          	144
#define IRQ145          	145
#define IRQ146          	146
#define IRQ147          	147
#define IRQ148          	148
#define IRQ149          	149
#define IRQ150          	150
#define IRQ151          	151
#define IRQ152          	152
#define IRQ153          	153
#define IRQ154          	154
#define IRQ155          	155
#define IRQ156          	156
#define IRQ157          	157
#define IRQ158          	158
#define IRQ159          	159
#define IRQ160          	160
#define IRQ161          	161
#define IRQ162          	162
#define IRQ163          	163
#define IRQ164          	164
#define IRQ165          	165
#define IRQ166          	166
#define IRQ167          	167
#define IRQ168          	168
#define IRQ169          	169
#define IRQ170          	170
#define IRQ171          	171
#define IRQ172          	172
#define IRQ173          	173
#define IRQ174          	174
#define IRQ175          	175
#define IRQ176          	176
#define IRQ177          	177
#define IRQ178          	178
#define IRQ179          	179
#define IRQ180          	180
#define IRQ181          	181
#define IRQ182          	182
#define IRQ183          	183
#define IRQ184          	184
#define IRQ185          	185
#define IRQ186          	186
#define IRQ187          	187
#define IRQ188          	188
#define IRQ189          	189
#define IRQ190          	190
#define IRQ191          	191
#define IRQ192          	192
#define IRQ193          	193
#define IRQ194          	194
#define IRQ195          	195
#define IRQ196          	196
#define IRQ197          	197
#define IRQ198          	198
#define IRQ199          	199
#define IRQ200          	200
#define IRQ201          	201
#define IRQ202          	202
#define IRQ203          	203
#define IRQ204          	204
#define IRQ205          	205
#define IRQ206          	206
#define IRQ207          	207
#define IRQ208          	208
#define IRQ209          	209
#define IRQ210          	210
#define IRQ211          	211
#define IRQ212          	212
#define IRQ213          	213
#define IRQ214          	214
#define IRQ215          	215
#define IRQ216          	216
#define IRQ217          	217
#define IRQ218          	218
#define IRQ219          	219
#define IRQ220          	220
#define IRQ221          	221
#define IRQ222          	222
#define IRQ223          	223
#define IRQ224          	224
#define IRQ225          	225
#define IRQ226          	226
#define IRQ227          	227
#define IRQ228          	228
#define IRQ229          	229
#define IRQ230          	230
#define IRQ231          	231
#define IRQ232          	232
#define IRQ233          	233
#define IRQ234          	234
#define IRQ235          	235
#define IRQ236          	236
#define IRQ237          	237
#define IRQ238          	238
#define IRQ239          	239


#endif /* __CORTEXM4_H_ */