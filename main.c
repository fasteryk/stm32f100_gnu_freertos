#include <stm32f10x.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>


#define ULONG_MAX						0xffffffff

#define PWR_PB_PIN					GPIO_Pin_0
#define RED_LED_PIN					GPIO_Pin_1
#define GREEN_LED_PIN				GPIO_Pin_2
#define PWR_EN_PIN					GPIO_Pin_3
#define PM_STATUS_PIN				GPIO_Pin_4

#define I2C_GPIO						GPIOB
#define I2C_SCL_PIN					GPIO_Pin_6
#define I2C_SDA_PIN					GPIO_Pin_7

#define I2C_PORT						I2C1
#define I2C_CLOCK						100000
#define I2C_DEVICE_ADDR			0x70

#define NV_BIT_PB_DOWN			0x01
#define NV_BIT_PB_UP				0x02
#define NV_BIT_I2C_CMD			0x04

#define PB_DELAY_VALUE			10

#define I2C_CMD_PWR_OFF			1
#define I2C_CMD_ENABLE_WD		2
#define I2C_CMD_REFRESH_WD	3

#define POWER_STATUS_REG		BKP_DR1


enum {
	POWER_ON = 1,
	POWER_OFF
};

enum {
	RED_LED = 0,
	GREEN_LED
};

enum {
	LED_ON = 1,
	LED_OFF,
	LED_FLASH
};

struct led_status {
	uint32_t led_pin;
	uint32_t led_mode;
};


uint8_t i2c_cmd, pm_status = 0, pb_timer_en = 0;
TaskHandle_t pm_task_handle;

struct led_status leds[2] = {
		{RED_LED_PIN, LED_OFF},
		{GREEN_LED_PIN, LED_OFF}
};


void init_gpio()
{
	GPIO_InitTypeDef gpio_def;

	GPIOA->BSRR = RED_LED_PIN | GREEN_LED_PIN;
	GPIOA->BRR = PWR_EN_PIN;

	gpio_def.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_def.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_def.GPIO_Pin = RED_LED_PIN;
	GPIO_Init(GPIOA, &gpio_def);
	gpio_def.GPIO_Pin = GREEN_LED_PIN;
	GPIO_Init(GPIOA, &gpio_def);
	gpio_def.GPIO_Pin = PWR_EN_PIN;
	GPIO_Init(GPIOA, &gpio_def);
	gpio_def.GPIO_Pin = PM_STATUS_PIN;
	GPIO_Init(GPIOA, &gpio_def);

	gpio_def.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio_def.GPIO_Pin = PWR_PB_PIN;
	GPIO_Init(GPIOA, &gpio_def);

	gpio_def.GPIO_Pin = I2C_SCL_PIN;
	gpio_def.GPIO_Mode = GPIO_Mode_AF_OD;
	gpio_def.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(I2C_GPIO, &gpio_def);
	gpio_def.GPIO_Pin = I2C_SDA_PIN;
	GPIO_Init(I2C_GPIO, &gpio_def);
}

void init_periph_clock()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 |
												 RCC_APB1Periph_BKP |
												 RCC_APB1Periph_PWR,
													ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO |
													RCC_APB2Periph_GPIOA |
													RCC_APB2Periph_GPIOB,
													ENABLE);
}

void init_nvic()
{
	NVIC_InitTypeDef nvic_def;

	//Must be set to NVIC_PriorityGroup_4
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	nvic_def.NVIC_IRQChannel = I2C1_EV_IRQn;
	nvic_def.NVIC_IRQChannelPreemptionPriority =
										configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
	nvic_def.NVIC_IRQChannelSubPriority = 0;
	nvic_def.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_def);

	nvic_def.NVIC_IRQChannel = I2C1_ER_IRQn;
	nvic_def.NVIC_IRQChannelPreemptionPriority =
										configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
	nvic_def.NVIC_IRQChannelSubPriority = 0;
	nvic_def.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_def);
}

void init_i2c()
{
	I2C_InitTypeDef i2c_def;

	i2c_def.I2C_Mode = I2C_Mode_I2C;
	i2c_def.I2C_DutyCycle = I2C_DutyCycle_2;
	i2c_def.I2C_OwnAddress1 = I2C_DEVICE_ADDR<<1;
	i2c_def.I2C_Ack = I2C_Ack_Enable;
	i2c_def.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	i2c_def.I2C_ClockSpeed = I2C_CLOCK;
	I2C_Init(I2C_PORT, &i2c_def);

	I2C_ITConfig(I2C_PORT, I2C_IT_BUF|I2C_IT_EVT|I2C_IT_ERR , ENABLE);

	I2C_Cmd(I2C_PORT, ENABLE);
}

void setup_hardware(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

	init_periph_clock();
	init_gpio();
	init_nvic();
	init_i2c();

	PWR_BackupAccessCmd(ENABLE);
}

void i2c_event_interrupt_handler(void)
{
	volatile uint32_t sr1, sr2;
	BaseType_t woken;

	sr1 = I2C_PORT->SR1;
	sr2 = I2C_PORT->SR2;

	if (sr1 & I2C_FLAG_ADDR) {

	}

	if (sr1 & I2C_FLAG_TXE)
		I2C_PORT->DR = pm_status;

	if (sr1 & I2C_FLAG_RXNE)
		i2c_cmd = I2C_PORT->DR;

	if (sr1 & I2C_FLAG_STOPF) {
		I2C_PORT->CR1 |= 1;
		xTaskNotifyFromISR(pm_task_handle, NV_BIT_I2C_CMD, eSetBits, &woken);
	}

	portYIELD_FROM_ISR(woken);
}

void i2c_error_interrupt_handler(void)
{
	if (I2C_GetITStatus(I2C_PORT, I2C_IT_BERR) == SET)
		I2C_ClearITPendingBit(I2C_PORT, I2C_IT_BERR);

	if (I2C_GetITStatus(I2C_PORT, I2C_IT_ARLO) == SET)
		I2C_ClearITPendingBit(I2C_PORT, I2C_IT_ARLO);

	if (I2C_GetITStatus(I2C_PORT, I2C_IT_AF) == SET)
		I2C_ClearITPendingBit(I2C_PORT, I2C_IT_AF);
}

void set_led(uint8_t led, uint8_t mode)
{
	leds[led].led_mode = mode;
}

inline void power_on()
{
	GPIOA->BSRR = PWR_EN_PIN;
	set_led(RED_LED, LED_OFF);
	set_led(GREEN_LED, LED_ON);

	BKP_WriteBackupRegister(POWER_STATUS_REG, POWER_ON);
}

inline void power_off()
{
	GPIOA->BRR = PWR_EN_PIN;
	set_led(RED_LED, LED_ON);
	set_led(GREEN_LED, LED_OFF);

	BKP_WriteBackupRegister(POWER_STATUS_REG, POWER_OFF);
}

void switch_debounce_task(void *p)
{
	uint8_t ps = 0, cs = 0;
	uint32_t nv;

	while (1) {
		cs = GPIOA->IDR & PWR_PB_PIN ? 1 : 0;

		if (cs != ps) {
			vTaskDelay(10);
			if (cs != ps) {
				nv = cs ? NV_BIT_PB_UP : NV_BIT_PB_DOWN;
				xTaskNotify(pm_task_handle, nv, eSetBits);
			}
		}

		ps = cs;
		vTaskDelay(5);
	}
}

void cycle_task(void *p)
{
	uint8_t pb_delay = 0, led_cnt = 0, led_idx = 0;
	uint16_t ps;

	while (1) {
		if (pb_timer_en) {
			if (++pb_delay == PB_DELAY_VALUE) {
				pb_delay = 0;

				ps = BKP_ReadBackupRegister(POWER_STATUS_REG);
				switch (ps) {
				case POWER_OFF:
					power_on();
					break;

				case POWER_ON:
					power_off();
					break;

				default: ;
				}
			}
		} else
			pb_delay = 0;

		if (++led_cnt == 5) {
			led_cnt = 0;

			for (led_idx = 0;
						led_idx < sizeof(leds)/sizeof(struct led_status);
						led_idx++) {
				switch (leds[led_idx].led_mode) {
				case LED_ON:
					GPIOA->BRR = leds[led_idx].led_pin;
					break;
				case LED_OFF:
					GPIOA->BSRR = leds[led_idx].led_pin;
					break;
				case LED_FLASH:
					GPIOA->ODR ^= leds[led_idx].led_pin;
					break;
				default: ;
				}
			}
		}

		vTaskDelay(100);
	}
}

void pm_task(void *p)
{
	uint32_t nv;
	uint16_t ps;

	if (RCC_GetFlagStatus(RCC_FLAG_PORRST) == SET)
		power_off();
	else {
		ps = BKP_ReadBackupRegister(POWER_STATUS_REG);
		if (ps == POWER_OFF)
			power_off();
		else {
			set_led(GREEN_LED, LED_OFF);
			set_led(RED_LED, LED_FLASH);
			vTaskDelay(5000);
			power_on();
		}
	}

	RCC_ClearFlag();

	while (1) {
		xTaskNotifyWait(0, ULONG_MAX, &nv, portMAX_DELAY);

		if (nv & NV_BIT_PB_DOWN)
			pb_timer_en = 1;

		if (nv & NV_BIT_PB_UP)
			pb_timer_en = 0;

		if (nv & NV_BIT_I2C_CMD) {
			switch (i2c_cmd) {
			case I2C_CMD_PWR_OFF:
				power_off();
				break;

			default: ;
			}
		}
	}
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	for ( ; ; ) {
		__WFI();
	}
}

int main(void)
{
	BaseType_t ret;

	setup_hardware();

	ret = xTaskCreate(pm_task, "pm", 64, (void *)NULL,
								tskIDLE_PRIORITY+1, &pm_task_handle);
	configASSERT(ret == pdPASS);

	xTaskCreate(cycle_task, "cycle", 64, (void *)NULL,
									tskIDLE_PRIORITY+2, NULL);

	xTaskCreate(switch_debounce_task, "sdeb", 64, (void *)NULL,
								tskIDLE_PRIORITY+2, NULL);

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle
	   task.  The idle task is created within vTaskStartScheduler(). */
	for ( ; ; ) {
		__WFI();
	}

	return 0;
}
