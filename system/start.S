/* STM32F103 start up code */


.syntax unified
.cpu cortex-m3
#.fpu softvfp
.thumb


.section .stack
	.space 0x100
__initial_sp:


.section .isr_vector
.align 2

/*
 * vector table mapped to dddress 0 at reset
*/

__vectors:
        .word   __initial_sp               /* Top of Stack */
        .word   reset_handler              /* Reset Handler */
        .word   NMI_Handler                /* NMI Handler */
        .word   HardFault_Handler          /* Hard Fault Handler */
        .word   MemManage_Handler          /* MPU Fault Handler */
        .word   BusFault_Handler           /* Bus Fault Handler */
        .word   UsageFault_Handler         /* Usage Fault Handler */
        .word   0                          /* Reserved */
        .word   0                          /* Reserved */
        .word   0                          /* Reserved */
        .word   0                          /* Reserved */
        .word   vPortSVCHandler            /* SVCall Handler */
        .word   DebugMon_Handler           /* Debug Monitor Handler */
        .word   0                          /* Reserved */
        .word   xPortPendSVHandler         /* PendSV Handler */
        .word   xPortSysTickHandler        /* SysTick Handler */

/*
 * External Interrupts
*/

        .word   WWDG_IRQHandler            /* Window Watchdog */
        .word   PVD_IRQHandler             /* PVD through EXTI Line detect */
        .word   TAMPER_IRQHandler          /* Tamper */
        .word   RTC_IRQHandler             /* RTC */
        .word   FLASH_IRQHandler           /* Flash */
        .word   RCC_IRQHandler             /* RCC */
        .word   EXTI0_IRQHandler           /* EXTI Line 0 */
        .word   EXTI1_IRQHandler           /* EXTI Line 1 */
        .word   EXTI2_IRQHandler           /* EXTI Line 2 */
        .word   EXTI3_IRQHandler           /* EXTI Line 3 */
        .word   EXTI4_IRQHandler           /* EXTI Line 4 */
        .word   DMA1_Channel1_IRQHandler   /* DMA1 Channel 1 */
        .word   DMA1_Channel2_IRQHandler   /* DMA1 Channel 2 */
        .word   DMA1_Channel3_IRQHandler   /* DMA1 Channel 3 */
        .word   DMA1_Channel4_IRQHandler   /* DMA1 Channel 4 */
        .word   DMA1_Channel5_IRQHandler   /* DMA1 Channel 5 */
        .word   DMA1_Channel6_IRQHandler   /* DMA1 Channel 6 */
        .word   DMA1_Channel7_IRQHandler   /* DMA1 Channel 7 */
        .word   ADC1_2_IRQHandler          /* ADC1_2 */
        .word   USB_HP_CAN1_TX_IRQHandler  /* USB High Priority or CAN1 TX */
        .word   USB_LP_CAN1_RX0_IRQHandler /* USB Low  Priority or CAN1 RX0 */
        .word   CAN1_RX1_IRQHandler        /* CAN1 RX1 */
        .word   CAN1_SCE_IRQHandler        /* CAN1 SCE */
        .word   EXTI9_5_IRQHandler         /* EXTI Line 9..5 */
        .word   TIM1_BRK_IRQHandler        /* TIM1 Break */
        .word   TIM1_UP_IRQHandler         /* TIM1 Update */
        .word   TIM1_TRG_COM_IRQHandler    /* TIM1 Trigger and Commutation */
        .word   TIM1_CC_IRQHandler         /* TIM1 Capture Compare */
        .word   TIM2_IRQHandler            /* TIM2 */
        .word   TIM3_IRQHandler            /* TIM3 */
        .word   TIM4_IRQHandler            /* TIM4 */
        .word   i2c_event_interrupt_handler     /* I2C1 Event */
        .word   i2c_error_interrupt_handler     /* I2C1 Error */
        .word   I2C2_EV_IRQHandler         /* I2C2 Event */
        .word   I2C2_ER_IRQHandler         /* I2C2 Error */
        .word   SPI1_IRQHandler            /* SPI1 */
        .word   SPI2_IRQHandler            /* SPI2 */
        .word   USART1_IRQHandler          /* USART1 */
        .word   USART2_IRQHandler          /* USART2 */
        .word   USART3_IRQHandler          /* USART3 */
        .word   EXTI15_10_IRQHandler       /* EXTI Line 15..10 */
        .word   RTCAlarm_IRQHandler        /* RTC Alarm through EXTI Line */
        .word   USBWakeUp_IRQHandler       /* USB Wakeup from suspend */

.text
.align
/*
 * the actual start code
*/
.globl reset_handler
.type reset_handler, %function

reset_handler:
		bl  SystemInit

		ldr		r0, =_end_of_rodata
		ldr		r1, =_begin_of_data
		ldr		r2, =_end_of_data
copy_l:
		cmp		r1, r2
		ldrblo	r3, [r0], #1
		strblo	r3, [r1], #1
		blo		copy_l

		ldr		r0, =_begin_of_bss
		ldr		r1, =_end_of_bss
		mov		r2, #0x00000000
clbss_l:
		cmp		r0, r1
		strblo	r2, [r0], #1
		blo		clbss_l

		ldr		pc, =main

/*
 * dummy exception handlers (infinite loops which can be modified)
*/

.type NMI_Handler, %function
.weak   NMI_Handler
NMI_Handler:
        b   .

.type HardFault_Handler, %function
.weak   HardFault_Handler
HardFault_Handler:
        b   .

.type MemManage_Handler, %function
.weak   MemManage_Handler
MemManage_Handler:
        b   .

.type BusFault_Handler, %function
.weak   BusFault_Handler
BusFault_Handler:
        b   .

.type UsageFault_Handler, %function
.weak   UsageFault_Handler
UsageFault_Handler:
        b   .

.type SVC_Handler, %function
.weak   SVC_Handler
SVC_Handler:
        b   .

.type DebugMon_Handler, %function
.weak   DebugMon_Handler
DebugMon_Handler:
        b   .

.type PendSV_Handler, %function
.weak   PendSV_Handler
PendSV_Handler:
        b   .

.type SysTick_Handler, %function
.weak   SysTick_Handler
SysTick_Handler:
        b   .

.type WWDG_IRQHandler, %function
.weak   WWDG_IRQHandler

.type PVD_IRQHandler, %function
.weak   PVD_IRQHandler

.type TAMPER_IRQHandler, %function
.weak   TAMPER_IRQHandler

.type RTC_IRQHandler, %function
.weak   RTC_IRQHandler

.type FLASH_IRQHandler, %function
.weak   FLASH_IRQHandler

.type RCC_IRQHandler, %function
.weak   RCC_IRQHandler

.type EXTI0_IRQHandler, %function
.weak   EXTI0_IRQHandler

.type EXTI1_IRQHandler, %function
.weak   EXTI1_IRQHandler

.type EXTI2_IRQHandler, %function
.weak   EXTI2_IRQHandler

.type EXTI3_IRQHandler, %function
.weak   EXTI3_IRQHandler

.type EXTI4_IRQHandler, %function
.weak   EXTI4_IRQHandler

.type DMA1_Channel1_IRQHandler, %function
.weak   DMA1_Channel1_IRQHandler

.type DMA1_Channel2_IRQHandler, %function
.weak   DMA1_Channel2_IRQHandler

.type DMA1_Channel3_IRQHandler, %function
.weak   DMA1_Channel3_IRQHandler

.type DMA1_Channel4_IRQHandler, %function
.weak   DMA1_Channel4_IRQHandler

.type DMA1_Channel5_IRQHandler, %function
.weak   DMA1_Channel5_IRQHandler

.type DMA1_Channel6_IRQHandler, %function
.weak   DMA1_Channel6_IRQHandler

.type DMA1_Channel7_IRQHandler, %function
.weak   DMA1_Channel7_IRQHandler

.type ADC1_2_IRQHandler, %function
.weak   ADC1_2_IRQHandler

.type USB_HP_CAN1_TX_IRQHandler, %function
.weak   USB_HP_CAN1_TX_IRQHandler

.type USB_LP_CAN1_RX0_IRQHandler, %function
.weak   USB_LP_CAN1_RX0_IRQHandler

.type CAN1_RX1_IRQHandler, %function
.weak   CAN1_RX1_IRQHandler

.type CAN1_SCE_IRQHandler, %function
.weak   CAN1_SCE_IRQHandler

.type EXTI9_5_IRQHandler, %function
.weak   EXTI9_5_IRQHandler

.type TIM1_BRK_IRQHandler, %function
.weak   TIM1_BRK_IRQHandler

.type TIM1_UP_IRQHandler, %function
.weak   TIM1_UP_IRQHandler

.type TIM1_TRG_COM_IRQHandler, %function
.weak   TIM1_TRG_COM_IRQHandler

.type TIM1_CC_IRQHandler, %function
.weak   TIM1_CC_IRQHandler

.type TIM2_IRQHandler, %function
.weak   TIM2_IRQHandler

.type TIM3_IRQHandler, %function
.weak   TIM3_IRQHandler

.type TIM4_IRQHandler, %function
.weak   TIM4_IRQHandler

.type I2C1_EV_IRQHandler, %function
.weak   I2C1_EV_IRQHandler

.type I2C1_ER_IRQHandler, %function
.weak   I2C1_ER_IRQHandler

.type I2C2_EV_IRQHandler, %function
.weak   I2C2_EV_IRQHandler

.type I2C2_ER_IRQHandler, %function
.weak   I2C2_ER_IRQHandler

.type SPI1_IRQHandler, %function
.weak   SPI1_IRQHandler

.type SPI2_IRQHandler, %function
.weak   SPI2_IRQHandler

.type USART1_IRQHandler, %function
.weak   USART1_IRQHandler

.type USART2_IRQHandler, %function
.weak   USART2_IRQHandler

.type USART3_IRQHandler, %function
.weak   USART3_IRQHandler

.type EXTI15_10_IRQHandler, %function
.weak   EXTI15_10_IRQHandler

.type RTCAlarm_IRQHandler, %function
.weak   RTCAlarm_IRQHandler

.type USBWakeUp_IRQHandler, %function
.weak   USBWakeUp_IRQHandler

WWDG_IRQHandler:
PVD_IRQHandler:
TAMPER_IRQHandler:
RTC_IRQHandler:
FLASH_IRQHandler:
RCC_IRQHandler:
EXTI0_IRQHandler:
EXTI1_IRQHandler:
EXTI2_IRQHandler:
EXTI3_IRQHandler:
EXTI4_IRQHandler:
DMA1_Channel1_IRQHandler:
DMA1_Channel2_IRQHandler:
DMA1_Channel3_IRQHandler:
DMA1_Channel4_IRQHandler:
DMA1_Channel5_IRQHandler:
DMA1_Channel6_IRQHandler:
DMA1_Channel7_IRQHandler:
ADC1_2_IRQHandler:
USB_HP_CAN1_TX_IRQHandler:
USB_LP_CAN1_RX0_IRQHandler:
CAN1_RX1_IRQHandler:
CAN1_SCE_IRQHandler:
EXTI9_5_IRQHandler:
TIM1_BRK_IRQHandler:
TIM1_UP_IRQHandler:
TIM1_TRG_COM_IRQHandler:
TIM1_CC_IRQHandler:
TIM2_IRQHandler:
TIM3_IRQHandler:
TIM4_IRQHandler:
I2C1_EV_IRQHandler:
I2C1_ER_IRQHandler:
I2C2_EV_IRQHandler:
I2C2_ER_IRQHandler:
SPI1_IRQHandler:
SPI2_IRQHandler:
USART1_IRQHandler:
USART2_IRQHandler:
USART3_IRQHandler:
EXTI15_10_IRQHandler:
RTCAlarm_IRQHandler:
USBWakeUp_IRQHandler:

        b   .

