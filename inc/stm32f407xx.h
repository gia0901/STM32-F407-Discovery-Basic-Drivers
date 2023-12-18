#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define __vo 	volatile
#define __weak	__attribute__((weak))

/******************* Bus base address *********************/

#define APB1_BASEADDR		0x40000000L
#define APB2_BASEADDR		(APB1_BASEADDR + 0x10000)
#define AHB1_BASEADDR		(APB1_BASEADDR + 0x20000)


/******************* ARM Cortex M4 peripheral base addresses *************/

#define NVIC_BASEADDR		0xE000E100L

/******************* MCU Peripheral base address *********************/

/* RCC clock */
#define RCC_BASEADDR		(AHB1_BASEADDR + 0x3800)
/* GPIO */
#define GPIOA_BASEADDR		AHB1_BASEADDR
#define GPIOB_BASEADDR		(AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR		(AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR		(AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR		(AHB1_BASEADDR + 0x1000)
#define GPIOF_BASEADDR		(AHB1_BASEADDR + 0x1400)
#define GPIOG_BASEADDR		(AHB1_BASEADDR + 0x1800)

/* EXTI: External interrupt/event controller */
#define EXTI_BASEADDR		(APB2_BASEADDR + 0x3C00)

/* SYSCFG: System configuration	controller */
#define SYSCFG_BASEADDR		(APB2_BASEADDR + 0x3800)

/* SPI */
#define SPI1_BASEADDR		(APB2_BASEADDR + 0x3000)
#define SPI2_BASEADDR		(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR		(APB1_BASEADDR + 0x3C00)

/* I2C */
#define I2C1_BASEADDR		(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR		(APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR		(APB1_BASEADDR + 0x5C00)

/* UART */
#define USART1_BASEADDR		(APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR		(APB2_BASEADDR + 0x1400)
#define USART2_BASEADDR		(APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR		(APB1_BASEADDR + 0x4800)
#define UART4_BASEADDR		(APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR		(APB1_BASEADDR + 0x5000)

/******************* ARM Cortex M4 peripheral definition structures *********************/
/*
 * NVIC peripheral
 */
typedef struct
{
	__vo uint32_t NVIC_ISER[8];		/* Interrupt Set-enable registers */
	__vo uint32_t NVIC_ICER[8];		/* Interrupt Clear-enable registers */
	__vo uint32_t NVIC_ISPR[8];		/* Interrupt Set-pending registers */
	__vo uint32_t NVIC_ICPR[8];		/* Interrupt Clear-pending registers */
	__vo uint32_t NVIC_IABR[8];		/* Interrupt Active Bit registers */
	__vo uint32_t NVIC_IPR[60];		/* Interrupt Priority registers */
	__vo uint32_t STIR;				/* Software Trigger Interrupt registers */
} NVIC_RegDef_t;


/******************* MCU peripheral definition structures ******************/
/*
 * RCC (Reset & Control Clock) registers
 */
typedef struct
{
	__vo uint32_t RCC_CR;
	__vo uint32_t RCC_PLLCFGR;
	__vo uint32_t RCC_CFGR;
	__vo uint32_t RCC_CIR;
	__vo uint32_t RCC_AHB1RSTR;
	__vo uint32_t RCC_AHB2RSTR;
	__vo uint32_t RCC_AHB3RSTR;
	uint32_t RESERVED1;
	__vo uint32_t RCC_APB1RSTR;
	__vo uint32_t RCC_APB2RSTR;
	uint32_t RESERVED2[2];
	__vo uint32_t RCC_AHB1ENR;
	__vo uint32_t RCC_AHB2ENR;
	__vo uint32_t RCC_AHB3ENR;
	uint32_t RESERVED3;
	__vo uint32_t RCC_APB1ENR;
	__vo uint32_t RCC_APB2ENR;
} RCC_RegDef_t;

/*
 * GPIO registers
 */
typedef struct
{
	__vo uint32_t GPIO_MODER;		/* Mode register */
	__vo uint32_t GPIO_OTYPER;		/* Mode register */
	__vo uint32_t GPIO_OSPEEDR;		/* Mode register */
	__vo uint32_t GPIO_PUPDR;		/* Mode register */
	__vo uint32_t GPIO_IDR;			/* Mode register */
	__vo uint32_t GPIO_ODR;			/* Mode register */
	__vo uint32_t GPIO_BSRR;		/* Mode register */
	__vo uint32_t GPIO_LCKR;		/* Mode register */
	__vo uint32_t GPIO_AFRL;		/* Mode register */
	__vo uint32_t GPIO_AFRH;		/* Mode register */
} GPIO_RegDef_t;

/*
 * EXTI registers
 */
typedef struct
{
	__vo uint32_t EXTI_IMR;			/* Interrupt mask register */
	__vo uint32_t EXTI_EMR;			/* Event mask register */
	__vo uint32_t EXTI_RTSR;		/* Rising Trigger selection register */
	__vo uint32_t EXTI_FTSR;		/* Falling Trigger selection register */
	__vo uint32_t EXTI_SWIER;		/* Software interrupt event register */
	__vo uint32_t EXTI_PR;			/* Pending register */
} EXTI_RegDef_t;

/*
 * SYSCFG registers
 */
typedef struct
{
	__vo uint32_t SYSCFG_MEMRMP;	/* (not used)Memory remap register */
	__vo uint32_t SYSCFG_PMC;		/* (not used)Peripheral mode register */
	__vo uint32_t SYSCFG_EXTICR[4];	/* External interrupt config register 1-4 */
	__vo uint32_t SYSCFG_CMPCR;		/* (not used)Compensation cell control register */
} SYSCFG_RegDef_t;

/*
 * SPI registers
 */
typedef struct
{
	__vo uint32_t SPI_CR1;			/* SPI Control Register 1 */
	__vo uint32_t SPI_CR2;			/* SPI Control Register 2 */
	__vo uint32_t SPI_SR;			/* SPI Status Register  */
	__vo uint32_t SPI_DR;			/* SPI Data Register  */
	__vo uint32_t SPI_CRCPR;
	__vo uint32_t SPI_RXCRCR;		/* SPI RX CRC Register  */
	__vo uint32_t SPI_TXCRCR;		/* SPI TX CRC Register  */
	__vo uint32_t SPI_I2SCFGR;
	__vo uint32_t SPI_I2SPR;
} SPI_RegDef_t;

/*
 * I2C registers
 */
typedef struct
{
	__vo uint32_t I2C_CR1;			/* I2C Control Register 1 */
	__vo uint32_t I2C_CR2;			/* I2C Control Register 2 */
	__vo uint32_t I2C_OAR1;
	__vo uint32_t I2C_OAR2;
	__vo uint32_t I2C_DR;			/* I2C Data Register  */
	__vo uint32_t I2C_SR1;			/* I2C Status Register 1 */
	__vo uint32_t I2C_SR2;			/* I2C Status Register 2 */
	__vo uint32_t I2C_CCR;			/* I2C Clock control Register */
	__vo uint32_t I2C_TRISE;		/* I2C T-rise Register */
	__vo uint32_t I2C_FLTR;
} I2C_RegDef_t;

/*
 * UART registers
 */
typedef struct
{
	__vo uint32_t USART_SR;			/*USART Status Register */
	__vo uint32_t USART_DR;			/*USART Data Register */
	__vo uint32_t USART_BRR;		/*USART Baudrate Register */
	__vo uint32_t USART_CR1;		/*USART Control Register 1 */
	__vo uint32_t USART_CR2;		/*USART Control Register 2 */
	__vo uint32_t USART_CR3;		/*USART Control Register 3 */
	__vo uint32_t USART_GTPR;		/*USART Guard time and prescaler Register */
} USART_RegDef_t;

/******************* Peripheral definition macros ******************/
/* NVIC */
#define	NVIC		( (NVIC_RegDef_t *)NVIC_BASEADDR)
/* RCC */
#define RCC			( (RCC_RegDef_t*)RCC_BASEADDR)
/* EXTI */
#define	EXTI		( (EXTI_RegDef_t*)EXTI_BASEADDR)
/* SYSCFG */
#define SYSCFG		( (SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
/* GPIO */
#define GPIOA		( (GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		( (GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		( (GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD		( (GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE		( (GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF		( (GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG		( (GPIO_RegDef_t*)GPIOG_BASEADDR)
/* SPI */
#define SPI1		( (SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		( (SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3		( (SPI_RegDef_t*)SPI3_BASEADDR)
/* I2C */
#define I2C1		( (I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2		( (I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3		( (I2C_RegDef_t*)I2C3_BASEADDR)
/* UART/USART */
#define USART1		( (USART_RegDef_t*)USART1_BASEADDR)
#define USART2		( (USART_RegDef_t*)USART2_BASEADDR)
#define USART3		( (USART_RegDef_t*)USART3_BASEADDR)
#define UART4		( (USART_RegDef_t*)UART4_BASEADDR)
#define UART5		( (USART_RegDef_t*)UART5_BASEADDR)
#define USART6		( (USART_RegDef_t*)USART6_BASEADDR)


/******************* Peripheral clock enable/disable macros *********************/

/* GPIO clock enable - disable */
#define RCC_GPIOA_CLK_EN()	( RCC->RCC_AHB1ENR |= (1 << 0))
#define RCC_GPIOB_CLK_EN()	( RCC->RCC_AHB1ENR |= (1 << 1))
#define RCC_GPIOC_CLK_EN()	( RCC->RCC_AHB1ENR |= (1 << 2))
#define RCC_GPIOD_CLK_EN()	( RCC->RCC_AHB1ENR |= (1 << 3))
#define RCC_GPIOE_CLK_EN()	( RCC->RCC_AHB1ENR |= (1 << 4))
#define RCC_GPIOF_CLK_EN()	( RCC->RCC_AHB1ENR |= (1 << 5))
#define RCC_GPIOG_CLK_EN()	( RCC->RCC_AHB1ENR |= (1 << 6))
#define RCC_GPIOA_CLK_DI()	( RCC->RCC_AHB1ENR &= ~(1 << 0))
#define RCC_GPIOB_CLK_DI()	( RCC->RCC_AHB1ENR &= ~(1 << 1))
#define RCC_GPIOC_CLK_DI()	( RCC->RCC_AHB1ENR &= ~(1 << 2))
#define RCC_GPIOD_CLK_DI()	( RCC->RCC_AHB1ENR &= ~(1 << 3))
#define RCC_GPIOE_CLK_DI()	( RCC->RCC_AHB1ENR &= ~(1 << 4))
#define RCC_GPIOF_CLK_DI()	( RCC->RCC_AHB1ENR &= ~(1 << 5))
#define RCC_GPIOG_CLK_DI()	( RCC->RCC_AHB1ENR &= ~(1 << 6))

/* SYSCFG clock enable/disable */
#define RCC_SYSCFG_CLK_EN()	( RCC->RCC_APB2ENR |= (1 << 14))
#define RCC_SYSCFG_CLK_DI()	( RCC->RCC_APB2ENR &= ~(1 << 14))

/* SPI clock enable/disable */
#define RCC_SPI1_CLK_EN() 	( RCC->RCC_APB2ENR |= (1 << 12))
#define RCC_SPI2_CLK_EN() 	( RCC->RCC_APB1ENR |= (1 << 14))
#define RCC_SPI3_CLK_EN() 	( RCC->RCC_APB1ENR |= (1 << 15))
#define RCC_SPI1_CLK_DI() 	( RCC->RCC_APB2ENR &= ~(1 << 12))
#define RCC_SPI2_CLK_DI() 	( RCC->RCC_APB1ENR &= ~(1 << 14))
#define RCC_SPI3_CLK_DI() 	( RCC->RCC_APB1ENR &= ~(1 << 15))

/* I2C clock enable/disable */
#define RCC_I2C1_CLK_EN() 	( RCC->RCC_APB1ENR |= (1 << 21))
#define RCC_I2C2_CLK_EN() 	( RCC->RCC_APB1ENR |= (1 << 22))
#define RCC_I2C3_CLK_EN() 	( RCC->RCC_APB1ENR |= (1 << 23))
#define RCC_I2C1_CLK_DI() 	( RCC->RCC_APB1ENR &= ~(1 << 21))
#define RCC_I2C2_CLK_DI() 	( RCC->RCC_APB1ENR &= ~(1 << 22))
#define RCC_I2C3_CLK_DI() 	( RCC->RCC_APB1ENR &= ~(1 << 23))

/* UART/USART clock enable/disable */
#define RCC_USART1_CLK_EN()	( RCC->RCC_APB2ENR |= (1 << 4))
#define RCC_USART6_CLK_EN()	( RCC->RCC_APB2ENR |= (1 << 5))
#define RCC_USART2_CLK_EN()	( RCC->RCC_APB1ENR |= (1 << 17))
#define RCC_USART3_CLK_EN()	( RCC->RCC_APB1ENR |= (1 << 18))
#define RCC_UART4_CLK_EN()	( RCC->RCC_APB1ENR |= (1 << 19))
#define RCC_UART5_CLK_EN()	( RCC->RCC_APB1ENR |= (1 << 20))

#define RCC_USART1_CLK_DI()	( RCC->RCC_APB2ENR &= ~(1 << 4))
#define RCC_USART6_CLK_DI()	( RCC->RCC_APB2ENR &= ~(1 << 5))
#define RCC_USART2_CLK_DI()	( RCC->RCC_APB1ENR &= ~(1 << 17))
#define RCC_USART3_CLK_DI()	( RCC->RCC_APB1ENR &= ~(1 << 18))
#define RCC_UART4_CLK_DI()	( RCC->RCC_APB1ENR &= ~(1 << 19))
#define RCC_UART5_CLK_DI()	( RCC->RCC_APB1ENR &= ~(1 << 20))

/******************* Peripheral reset macros *********************/

/* GPIO reset */
#define RCC_GPIOA_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1 << 0)); (RCC->RCC_AHB1RSTR &= ~(1 << 0)); }while(0)
#define RCC_GPIOB_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1 << 1)); (RCC->RCC_AHB1RSTR &= ~(1 << 1)); }while(0)
#define RCC_GPIOC_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1 << 2)); (RCC->RCC_AHB1RSTR &= ~(1 << 2)); }while(0)
#define RCC_GPIOD_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1 << 3)); (RCC->RCC_AHB1RSTR &= ~(1 << 3)); }while(0)
#define RCC_GPIOE_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1 << 4)); (RCC->RCC_AHB1RSTR &= ~(1 << 4)); }while(0)
#define RCC_GPIOF_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1 << 5)); (RCC->RCC_AHB1RSTR &= ~(1 << 5)); }while(0)
#define RCC_GPIOG_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1 << 6)); (RCC->RCC_AHB1RSTR &= ~(1 << 6)); }while(0)
#define RCC_GPIOH_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1 << 7)); (RCC->RCC_AHB1RSTR &= ~(1 << 7)); }while(0)
#define RCC_GPIOI_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1 << 8)); (RCC->RCC_AHB1RSTR &= ~(1 << 8)); }while(0)

/* SPI reset */
#define RCC_SPI1_RESET()	do{ (RCC->RCC_APB2RSTR |= (1 << 12)); (RCC->RCC_APB2RSTR &= ~(1 << 12)); }while(0)
#define RCC_SPI2_RESET()	do{ (RCC->RCC_APB1RSTR |= (1 << 14)); (RCC->RCC_APB1RSTR &= ~(1 << 14)); }while(0)
#define RCC_SPI3_RESET()	do{ (RCC->RCC_APB1RSTR |= (1 << 15)); (RCC->RCC_APB1RSTR &= ~(1 << 15)); }while(0)

/* I2C reset */
#define RCC_I2C1_RESET()	do{ (RCC->RCC_APB1RSTR |= (1 << 21)); (RCC->RCC_APB1RSTR &= ~(1 << 21)); }while(0)
#define RCC_I2C2_RESET()	do{ (RCC->RCC_APB1RSTR |= (1 << 22)); (RCC->RCC_APB1RSTR &= ~(1 << 22)); }while(0)
#define RCC_I2C3_RESET()	do{ (RCC->RCC_APB1RSTR |= (1 << 23)); (RCC->RCC_APB1RSTR &= ~(1 << 23)); }while(0)

/* UART/USART reset */
#define RCC_USART1_RESET()	do{ (RCC->RCC_APB2RSTR |= (1 << 4)); (RCC->RCC_APB2RSTR &= ~(1 << 4)); }while(0)
#define RCC_USART6_RESET()	do{ (RCC->RCC_APB2RSTR |= (1 << 5)); (RCC->RCC_APB2RSTR &= ~(1 << 5)); }while(0)
#define RCC_USART2_RESET()	do{ (RCC->RCC_APB1RSTR |= (1 << 17)); (RCC->RCC_APB1RSTR &= ~(1 << 17)); }while(0)
#define RCC_USART3_RESET()	do{ (RCC->RCC_APB1RSTR |= (1 << 18)); (RCC->RCC_APB1RSTR &= ~(1 << 18)); }while(0)
#define RCC_UART4_RESET()	do{ (RCC->RCC_APB1RSTR |= (1 << 19)); (RCC->RCC_APB1RSTR &= ~(1 << 19)); }while(0)
#define RCC_UART5_RESET()	do{ (RCC->RCC_APB1RSTR |= (1 << 20)); (RCC->RCC_APB1RSTR &= ~(1 << 20)); }while(0)



/************************ IRQ Number *********************/
#define IRQ_NUM_EXTI0		6		/* EXTI line 0 interrupt */
#define IRQ_NUM_EXTI1		7		/* EXTI line 1 interrupt */
#define IRQ_NUM_EXTI2		8		/* EXTI line 2 interrupt */
#define IRQ_NUM_EXTI3		9		/* EXTI line 3 interrupt */
#define IRQ_NUM_EXTI4		10		/* EXTI line 4 interrupt */
#define IRQ_NUM_EXTI9_5		23		/* EXTI line 5-9 interrupts */
#define IRQ_NUM_EXTI15_10	40		/* EXTI line 10-15 interrupts */

#define IRQ_NUM_SPI1		35
#define IRQ_NUM_SPI2		36
#define IRQ_NUM_SPI3		51

#define IRQ_NUM_I2C1_EV		31
#define IRQ_NUM_I2C1_ER		32
#define IRQ_NUM_I2C2_EV		33
#define IRQ_NUM_I2C2_ER		34
#define IRQ_NUM_I2C3_EV		72
#define IRQ_NUM_I2C3_ER		73

#define IRQ_NUM_USART1		37
#define IRQ_NUM_USART2		38
#define IRQ_NUM_USART3		39
#define IRQ_NUM_UART4		52
#define IRQ_NUM_UART5		53
#define IRQ_NUM_USART6		71

#define IRQ_PRIORIRY_0		0
#define IRQ_PRIORITY_1		1
#define IRQ_PRIORITY_5		5
#define IRQ_PRIORITY_10		10


/********************** Alternate Function Mapping macros *********************/

#define AF4					4
#define AF5					5
#define AF6					6
#define AF7					7
#define AF8					8

#define AF4_I2C1			AF4
#define AF4_I2C2			AF4
#define AF4_I2C3			AF4
#define AF5_SPI1			AF5
#define AF5_SPI2			AF5
#define AF6_SPI3			AF6
#define AF7_USART1			AF7
#define AF7_USART2			AF7
#define AF7_USART3			AF7
#define AF8_UART4			AF8
#define AF8_UART5			AF8
#define AF8_USART6			AF8


/********************** RCC Peripheral Register Bit position macros *********************/
/* RCC_CR register */
#define RCC_CR_HSION			0
#define RCC_CR_HSIRDY			1
#define RCC_CR_HSEON			16
#define RCC_CR_HSERDY			17
#define RCC_CR_PLLON			24
#define RCC_CR_PLLRDY			25

/* RCC_PLLCFGR register */
#define RCC_PLLCFGR_PLLM		0
#define RCC_PLLCFGR_PLLN		6
#define RCC_PLLCFGR_PLLP		16
#define RCC_PLLCFGR_PLLSRC		22
#define RCC_PLLCFGR_PLLQ		24


/* RCC_CFGR register */
#define RCC_CFGR_SW				0
#define RCC_CFGR_SWS			2
#define RCC_CFGR_HRPE			4	// AHB prescaler
#define RCC_CFGR_PPRE1			10	// APB1 prescaler
#define RCC_CFGR_PPRE2			13	// APB2 prescaler
#define RCC_CFGR_MCO1			21
#define RCC_CFGR_MCO1PRE		24
#define RCC_CFGR_MCO2PRE		27
#define RCC_CFGR_MCO2			30


/********************** SPI Peripheral Register Bit position macros *********************/
/* SPI_CR1 register */
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2	// Master selection
#define SPI_CR1_BR				3	// Baudrate [3:5] control
#define SPI_CR1_SPE				6	// SPI enable -> start the communication
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10	// Control receive only in Full-duplex mode
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14	// Control transmit or receive only in Half-duplex mode
#define SPI_CR1_BIDIMODE		15	// Full-duplex / Half-duplex

/* SPI_CR2 register */
#define SPI_CR2_RXDMAEN			0	// RX buffer DMA enable
#define SPI_CR2_TXDMAEN			1	// TX buffer DMA enable
#define SPI_CR2_SSOE			2	// SS output enable
#define SPI_CR2_FRF				4	// Frame format
#define SPI_CR2_ERRIE			5	// Error interrupt enable
#define SPI_CR2_RXNEIE			6	// RX buffer not empty interrupt enable
#define SPI_CR2_TXEIE			7	// TX buffer empty interrupt enable

/* SPI_SR register */
#define SPI_SR_RXNE				0	// Receive buffer not empty
#define SPI_SR_TXE				1	// Transmit buffer empty
#define SPI_SR_CHSIDE			2	// Channel side
#define SPI_SR_UDR				3	// Underrun flag
#define SPI_SR_CRCERR			4	// CRC error flag
#define SPI_SR_MODF				5	// Mode fault
#define SPI_SR_OVR				6	// Overrun flag
#define SPI_SR_BSY				7	// Busy flag
#define SPI_SR_FRE				8	// Frame format error


/********************** I2C Peripheral Register Bit position macros *********************/
/* I2C_CR1 register */
#define I2C_CR1_PE				0	// Peripheral enable
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8	// Start generation
#define I2C_CR1_STOP			9	// Stop generation
#define I2C_CR1_ACK				10	// ACK enable: set & cleared by software, cleared by hardware when PE = 0

/* I2C_CR2 register */
#define I2C_CR2_FREQ			0	// 6 bits of Frequency, must bigger than 1
#define I2C_CR2_ITERREN			8	// Error interrupt enable
#define I2C_CR2_ITEVTEN			9	// Event interrupt enable
#define I2C_CR2_ITBUFEN			10	// TXE = 1 or RXNE = 1 will generate Event interrupt

/* I2C_SR1 register (Read-only) */
#define I2C_SR1_SB				0	// start bit: set when a start condition is generated, cleared by software by reading SR1 & writing DR
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2	// Byte transfer finished
#define I2C_SR1_ADD10			3	// 10 bit header sent
#define I2C_SR1_STOPF			4	// Stop detection (slave mode)
#define I2C_SR1_RxNE			6	// Data register is not empty
#define I2C_SR1_TxE				7	// Data register is empty
#define I2C_SR1_BERR			8	// Bus error
#define I2C_SR1_ARLO			9	// Arbitration lost (master mode)
#define I2C_SR1_AF				10	// ACK failure
#define I2C_SR1_OVR				11	// Overrun
#define I2C_SR1_TIMEOUT			14	// timeout


/* I2C_SR2 register (Read-only) */
#define I2C_SR2_MSL				0	// master/slave
#define I2C_SR2_BUSY			1	// bus busy
#define I2C_SR2_TRA				2	// transmitter/receiver


/* I2C_CCR register (must be configured when I2C is disabled (PE = 0) */
#define I2C_CCR_DUTY			14	// Fast mode duty cycle
#define I2C_CCR_FS				15	// Master mode selection: standard/fast


/********************** UART/USART Peripheral Register Bit position macros *********************/
/* USART_SR */
#define USART_SR_PE				0	// Parity error
#define USART_SR_FE				1	// Framing error
#define USART_SR_NF				2	// Noise detected flag
#define USART_SR_ORE			3	// Overrun error
#define USART_SR_IDLE			4	// IDLE line detected
#define USART_SR_RXNE			5	// RXNE
#define USART_SR_TC				6	// Transmission complete
#define USART_SR_TXE			7	// TXE
#define USART_SR_LBD			8
#define USART_SR_CTS			9	// CTS flag

/* USART_DR */

/* USART_BRR */
#define USART_BRR_DIV_Fraction	0
#define USART_BRR_DIV_Mantissa	4

/* USART_CR1 */
#define USART_CR1_SBK			0	// Send break
#define USART_CR1_RWU			1	// Receiver wakeup
#define USART_CR1_RE			2	// Receiver mode
#define USART_CR1_TE			3	// Transmitter mode
#define USART_CR1_IDLEIE		4	// IDLE interrupt enable
#define USART_CR1_RXNEIE		5	// RXNE interrupt enable
#define USART_CR1_TCIE			6	// Transmission complete interrupt enable
#define USART_CR1_TXEIE			7	// TXE interrupt enable
#define USART_CR1_PEIE			8	// PE (parity error) interrupt enable
#define USART_CR1_PS			9	// Parity selection
#define USART_CR1_PCE			10	// Parity control enable
#define USART_CR1_WAKE			11
#define USART_CR1_M				12	// Word length
#define USART_CR1_UE			13	// USART enable
#define USART_CR1_OVER8			15	// Oversampling mode: 16 or 8

/* USART_CR2 */
#define USART_CR2_ADD			0	// Address of the USART node
#define USART_CR2_LBDL			5	// (not use)LIN break detection length
#define USART_CR2_LBDIE			6	// (not use)LIN break detection interrupt enable
#define USART_CR2_LBCL			8	// Last bit clock pulse
#define USART_CR2_CPHA			9	// Clock phase
#define USART_CR2_CPOL			10	// Clock polarity
#define USART_CR2_CLKEN			11	// Clock enable (CK pin)
#define USART_CR2_STOP			12	// number of STOP bit
#define USART_CR2_LINEN			14	// (not use)Lin mode enable

/* USART_CR3 */
#define USART_CR3_EIE			0	// Error interrupt enable
#define USART_CR3_IREN			1	// (not use)IrDA mode enable
#define USART_CR3_IRLP			2	// (not use)IrDA low-power
#define USART_CR3_HDSEL			3	// (not use)Half-duplex selection
#define USART_CR3_NACK			4	// (not use)Smartcard NACK enable
#define USART_CR3_SCEN			5	// (not use)Smartcard mode enable
#define USART_CR3_DMAR			6	// (not use)DMA enable receiver
#define USART_CR3_DMAT			7	// (not use)DMA enable transmitter
#define USART_CR3_RTSE			8	// RTS enable
#define USART_CR3_CTSE			9	// CTS enable
#define USART_CR3_CTSIE			10	// CTS interrupt enable
#define USART_CR3_ONEBIT		11	// One sample bit method enable


/******************* Others ******************************/

/* Return the port number based on the GPIOx */
#define GPIO_BASEADDR_TO_PORT_NUMBER(x) (x == GPIOA ? 0:\
										(x == GPIOB)? 1:\
										(x == GPIOC)? 2:\
										(x == GPIOD)? 3:\
										(x == GPIOE)? 4:\
										(x == GPIOF)? 5:\
										(x == GPIOG)? 6: 0)

#define	ENABLE			1
#define DISABLE			0
#define	HIGH			ENABLE
#define LOW				DISABLE
#define FLAG_SET		ENABLE
#define FLAG_NOT_SET 	DISABLE

// include this make user application need not to include every single driver

#include "stm32f407xx_gpio.h"	// GPIO driver header file
#include "stm32f407xx_spi.h"	// SPI driver header file
#include "stm32f407xx_i2c.h"	// I2C driver header file
#include "stm32f407xx_uart.h"	// UART driver header file
#include "stm32f407xx_rcc.h"	// RCC driver header file

#endif /* INC_STM32F407XX_H_ */
