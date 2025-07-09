#include <K1921VG015.h>
#include <stdint.h>
#include <stdio.h>
#include <system_k1921vg015.h>
#include "retarget.h"
#include <string.h>

#define PWM_PERIOD 1000
#define CNT_SAMPLE 5
#define UART1_BAUD  115200

#define GPIOA_ALL_Msk  0xFFFF
#define GPIOB_ALL_Msk  0xFFFF
#define LEDS_MSK  0xFF00

#define PB0_MSK   (1 << 0)   // фонарик 2

#define PA5_MSK   (1 << 5)   // фонарик
#define PA7_MSK   (1 << 7)   // кнопка
#define LED0_MSK  (1 << 8)
#define LED1_MSK  (1 << 9)
#define LED2_MSK  (1 << 10)
#define LED3_MSK  (1 << 11)
#define LED4_MSK  (1 << 12)  // PA12
#define LED5_MSK  (1 << 13)
#define LED6_MSK  (1 << 14)
#define LED7_MSK  (1 << 15)  // PA15

static uint32_t button_click_counter = 1;
static uint32_t meows_barks_counter = 0;
static uint32_t flag_meows = 1;

char buff[120];

// Режимы мигания диодов
typedef enum LED_MODES{
	MODE_RUNNING_LIGHT = 1,
	MODE_BLINKING,
	MODE_ALL_ON,
	MODE_ALL_OFF,
	MODES_COUNT
};

void TMR32_IRQHandler();
void SPI0_IRQHandler();

void Send_buff(char* a)
{
	uint8_t i = 0;
	while ((i < 120) && (a[i] != '\0'))
	{
		retarget_put_char(a[i]);
		i++;
	}
}


void BSP_led_init() {
    RCU->CGCFGAHB_bit.GPIOAEN = 1;
    RCU->RSTDISAHB_bit.GPIOAEN = 1;

    // Инициализация GPIOB для PWM выхода
    RCU->CGCFGAHB_bit.GPIOBEN = 1;
    RCU->RSTDISAHB_bit.GPIOBEN = 1;
    GPIOB->OUTENSET = PB0_MSK;
    GPIOB->DATAOUTCLR = PB0_MSK;

    GPIOA->OUTENSET = LEDS_MSK;
    GPIOA->DATAOUTCLR = LEDS_MSK;

    GPIOA->OUTENCLR = PA7_MSK; // Кнопка как вход
    GPIOA->PULLMODE &= ~(0b11 << (7 * 2));
    GPIOA->PULLMODE |= (0b01 << (7 * 2)); // Подтяжка к уровню логической единицы

    GPIOA->OUTENSET = PA5_MSK; // Включение вывода PA5 как выход
}


void adcsar_init()
{
	// настройка питания ADCSAR
  PMUSYS->ADCPWRCFG_bit.LDOEN = 1;
  PMUSYS->ADCPWRCFG_bit.LVLDIS = 0;

  // ADCSAR->ACTL_bit.ISEL = 0;

  //Сбрасываем блок ADCSAR
  RCU->ADCSARCLKCFG_bit.RSTDIS = 0;
  RCU->RSTDISAPB_bit.ADCSAREN = 0;

  //Инициализация тактирвоания блока ADCSAR
  RCU->ADCSARCLKCFG_bit.CLKSEL = 1;
  RCU->ADCSARCLKCFG_bit.DIVEN = 0;
  RCU->ADCSARCLKCFG_bit.CLKEN = 1;
  RCU->ADCSARCLKCFG_bit.RSTDIS = 1;

  //инициализация тактирования и сброса логики ADCSAR
  RCU->CGCFGAPB_bit.ADCSAREN = 1;
  RCU->RSTDISAPB_bit.ADCSAREN = 1;

  //Настройка модуля ADCSAR
  //12бит и калибровка при включении
  ADCSAR->ACTL_bit.SELRES = ADCSAR_ACTL_SELRES_12bit;
  //ADCSAR->ACTL = 0;
  // ADCSAR->ACTL |= (3 << 4);  // источник опорного тока
  ADCSAR->ACTL_bit.CALEN = 1;
  ADCSAR->ACTL_bit.ADCEN = 1;
  //ADCSAR->ACTL_bit.ISEL = 1;

  //Настройка секвенсора 0: CH0 - CH7
  ADCSAR->EMUX_bit.EM0 = ADCSAR_EMUX_EM0_SwReq;
  //ADCSAR->EMUX = 0x0F;
  ADCSAR->SEQ[0].SCCTL_bit.ICNT = 0;
  ADCSAR->SEQ[0].SCCTL_bit.RCNT = 0;
  ADCSAR->SEQ[0].SRTMR = 0x0;
  ADCSAR->SEQ[0].SRQCTL_bit.QAVGVAL = ADCSAR_SEQ_SRQCTL_QAVGVAL_Disable;
  ADCSAR->SEQ[0].SRQCTL_bit.QAVGEN = 0;
  ADCSAR->SEQ[0].SRQCTL_bit.RQMAX = 7;
  ADCSAR->SEQ[0].SCCTL_bit.RAVGEN = 0;
  ADCSAR->SEQ[0].SRQSEL_bit.RQ0 = 0;
  ADCSAR->SEQ[0].SRQSEL_bit.RQ1 = 1;
  ADCSAR->SEQ[0].SRQSEL_bit.RQ2 = 2;
  ADCSAR->SEQ[0].SRQSEL_bit.RQ3 = 3;
  ADCSAR->SEQ[0].SRQSEL_bit.RQ4 = 4;
  ADCSAR->SEQ[0].SRQSEL_bit.RQ5 = 5;
  ADCSAR->SEQ[0].SRQSEL_bit.RQ6 = 6;
  ADCSAR->SEQ[0].SRQSEL_bit.RQ7 = 7;

  //Включаем секвенсоры
  ADCSAR->SEQSYNC = ADCSAR_SEQSYNC_SYNC0_Msk;
  ADCSAR->SEQEN = ADCSAR_SEQEN_SEQEN0_Msk;

  //Ждем пока АЦП пройдут инициализацию, начатую в самом начале
  while (!(ADCSAR->ACTL_bit.ADCRDY)) { };
}


void TMR32_init(uint32_t period)
{
	RCU->CGCFGAPB_bit.TMR32EN = 1;
	RCU->RSTDISAPB_bit.TMR32EN = 1;

	//Записываем значение периода в CAPCOM[0]
	TMR32->CAPCOM[0].VAL = period - 1;
	//Выбираем режим счета от 0 до значения CAPCOM[0]
	TMR32->CTRL_bit.MODE = 1;

	//Разрешаем прерывание по совпадению значения счетчика и CAPCOM[0]
	TMR32->IM = 2;

	// Настраиваем обработчик прерывания для TMR32
	PLIC_SetIrqHandler(Plic_Mach_Target, IsrVect_IRQ_TMR32, TMR32_IRQHandler);
	PLIC_SetPriority(IsrVect_IRQ_TMR32, 0x1);
	PLIC_IntEnable(Plic_Mach_Target, IsrVect_IRQ_TMR32);
}

void TMR1_PWN_init(uint16_t period)
{
  RCU->CGCFGAPB_bit.TMR1EN = 1;
  RCU->RSTDISAPB_bit.TMR1EN = 1;

  //Настраиваем альтернативную функцию для GPIOA.8 и GPIOA.9
  RCU->CGCFGAHB_bit.GPIOAEN = 1;
  //Включаем  GPIOA
  RCU->RSTDISAHB_bit.GPIOAEN = 1;
  // Выбираем льтернативную функцию №2 для GPIOA.8 и GPIOA.9
  GPIOA->ALTFUNCNUM_bit.PIN8 = 2;
  GPIOA->ALTFUNCNUM_bit.PIN9 = 2;
  GPIOA->ALTFUNCSET = (1 << 8) | (1 << 9);

  //Записываем значение периода в CAPCOM[0]
  TMR1->CAPCOM[0].VAL = period-1;

  //Настраиваем режим сравнения для CAPCOM[2] - управление выводом GPIOA.8
  TMR1->CAPCOM[2].CTRL_bit.CAP = 0; // Режим сравнения
  TMR1->CAPCOM[2].CTRL_bit.OUTMODE = 7; /* Выходной сигнал сбрасывается, когда таймер отсчитывает значение CAPCOM[n].VAL.
  Он устанавливается, когда таймер отсчитывает значение CAPCOM[0].VAL.*/
  TMR1->CAPCOM[2].VAL = (period >> 1) - 1;

  //Настраиваем режим сравнения для CAPCOM[3] - управление выводом GPIOA.9
  TMR1->CAPCOM[3].CTRL_bit.CAP = 0; // Режим сравнения
  TMR1->CAPCOM[3].CTRL_bit.OUTMODE = 3; /* Выходной сигнал устанавливается, когда таймер отсчитывает значение CAPCOM[n].VAL.
  Он сбрасывается, когда таймер отсчитывает значение CAPCOM[0].VAL.*/
  TMR1->CAPCOM[3].VAL = (period >> 1) - 1;

  TMR1->CTRL_bit.DIV = 3; // Делитель частоты на 8
  //Выбираем режим счета от 0 до значения CAPCOM[0]
  TMR1->CTRL_bit.MODE = TMR_CTRL_MODE_Up;

  //Разрешаем прерывание по совпадению значения счетчика и CAPCOM[0]
  TMR1->IM = 2;

  // Настраиваем обработчик прерывания для TMR1
  PLIC_SetIrqHandler (Plic_Mach_Target, IsrVect_IRQ_TMR32, TMR32_IRQHandler);
  PLIC_SetPriority   (IsrVect_IRQ_TMR32, 0x1);
  PLIC_IntEnable     (Plic_Mach_Target, IsrVect_IRQ_TMR32);
}

//  UART - Universal Asynchronous Receiver-Transmitter
void UART1_init()
{
	// HSECLK_VAL - частота внешнего высокоскоростного генератора
	// UART1_BAUD - желаемая скорость передачи
	// стр 188
    uint32_t baud_icoef = HSECLK_VAL / (16 * UART1_BAUD);
    uint32_t baud_fcoef = ((HSECLK_VAL / (16.0f * RETARGET_UART_BAUD) - baud_icoef) * 64 + 0.5f);

    // Настраиваем GPIO
    RCU->CGCFGAHB_bit.GPIOAEN = 1;  // включает такирование AHB
    RCU->RSTDISAHB_bit.GPIOAEN = 1; // снимает сброс
    RCU->CGCFGAPB_bit.UART1EN = 1;  // тактирование uart1
    RCU->RSTDISAPB_bit.UART1EN = 1; // снимает сброс с uart1

    GPIOA->ALTFUNCNUM_bit.PIN2 = 1;
    GPIOA->ALTFUNCNUM_bit.PIN3 = 1;

    // ALTFUNCSET – активирует альтернативные функции для PA2 и PA3
    GPIOA->ALTFUNCSET = GPIO_ALTFUNCSET_PIN2_Msk | GPIO_ALTFUNCSET_PIN3_Msk;

    // Настраиваем UART1
    // UARTCLKCFG - Регистры настройки тактирования UART
    // CLKSEL - выбирает источник тактирования UART1
    // HSE внешний генератор
    RCU->UARTCLKCFG[1].UARTCLKCFG_bit.CLKSEL = RCU_UARTCLKCFG_CLKSEL_HSE;

    // DIVEN - отключает дополнительный делитель частоты
    RCU->UARTCLKCFG[1].UARTCLKCFG_bit.DIVEN = 0;
    // RSTDIS - снимает сброс
    RCU->UARTCLKCFG[1].UARTCLKCFG_bit.RSTDIS = 1;
    // включает тактирование
    RCU->UARTCLKCFG[1].UARTCLKCFG_bit.CLKEN = 1;

    UART1->IBRD = baud_icoef; // записывасет дробную часть

    // Fractional Baud Rate Divisor
    UART1->FBRD = baud_fcoef;

    // Line Control Register High
    // UART_LCRH_FEN_Msk включает FIFO (буфер для данных)
    // 3 << UART_LCRH_WLEN_Pos – устанавливает 8 бит данных
    UART1->LCRH = UART_LCRH_FEN_Msk | (3 << UART_LCRH_WLEN_Pos);

    //Interrupt FIFO Level Select
    UART1->IFLS = 0;

    /*
     Control Register
     UART_CR_TXE_Msk разрешает передачу (TX)
     UART_CR_RXE_Msk разрешает приём (RX)
     UART_CR_UARTEN_Msk включает UART
     */
    UART1->CR = UART_CR_TXE_Msk | UART_CR_RXE_Msk | UART_CR_UARTEN_Msk;
}

//-- Peripheral init functions -------------------------------------------------
void periph_init()
{
	BSP_led_init();
	SystemInit();
	SystemCoreClockUpdate();
	UART1_init();
	retarget_init();
	adcsar_init();

	sprintf(buff,"  K1921VG015 SYSCLK = %d MHz\r\n\0",(int)(SystemCoreClock / 1E6)); 	Send_buff(buff);
	sprintf(buff,"  UID[0] = 0x%X  UID[1] = 0x%X  UID[2] = 0x%X  UID[3] = 0x%X\r\n\0",PMUSYS->UID[0],PMUSYS->UID[1],PMUSYS->UID[2],PMUSYS->UID[3]); Send_buff(buff);
    sprintf(buff,"  PartNum = 0x%X\r\n\0",(uint16_t)(PMUSYS->UID[3] >> 16)); Send_buff(buff);
    sprintf(buff,"  Start UART DMA\r\n\0"); Send_buff(buff);
}

//--- USER FUNCTIONS ----------------------------------------------------------------------
volatile uint32_t led_shift;
volatile uint32_t cnt_tgl_meows;

//-- Main ----------------------------------------------------------------------
int main(void)
{
	uint32_t	i;
	periph_init();
	TMR32_init(500000); // 50 000 000 частота ядра, 500 000 10мс
	TMR1_PWN_init(SystemCoreClock>>8);
	InterruptEnable();
	led_shift = LED0_MSK;
	 uint8_t brightness;
	while (1)
	{
		ADCSAR->SEQSYNC_bit.GSYNC = 1;

		    while ((ADCSAR->BSTAT));
		    while (!(ADCSAR->RIS_bit.SEQRIS0)); // Ожидание флага прерывания секвенсора 0
		    int chn = 1;
		    int ch_res = ADCSAR->SEQ[0].SFIFO;

		    int curr_voltage_mV = (ch_res * 3300) / 4095;


		    // на 2.5 В показывает плохо дальше
		    // aref 2.77 в
		    // 3.3 там 3.3
		    /*
		     * Про выбор опорного напряжения в документации не нашла
		     */

		    printf("  CH%d=%d.%dV\r", chn, curr_voltage_mV / 1000, (curr_voltage_mV % 1000) / 100);
		    printf("\r");


		    ADCSAR->IC = ADCSAR_IC_SEQIC0_Msk; // Сброс флага прерывания секвенсора 0
	}
	return 0;
}


void update_leds()
{
	switch(button_click_counter)
	{
	case MODE_RUNNING_LIGHT:
		GPIOA->DATAOUTTGL = led_shift;
		led_shift = led_shift << 1;
		if(led_shift > LED7_MSK) led_shift = LED0_MSK;
		break;

	case MODE_BLINKING:
		GPIOA->DATAOUTTGL = LEDS_MSK;
		break;

	case MODE_ALL_ON:
		GPIOA->DATAOUTSET = LEDS_MSK;
		break;
	case MODE_ALL_OFF:
		GPIOA->DATAOUTCLR = LEDS_MSK;
		break;

	default:
		break;
	}
}

//-- IRQ INTERRUPT HANDLERS ---------------------------------------------------------------
void TMR32_IRQHandler() // раз в 10 мс
{
	static uint8_t d = 0;
	static uint8_t last_button_state = 1;
	static uint32_t last_led_update = 0;

	if (cnt_tgl_meows < 100){ cnt_tgl_meows++; }
	else
	{
		flag_meows = (flag_meows == 0) ? 1 : 0;
		meows_barks_counter++;
		cnt_tgl_meows = 0;
		memset(buff, 0, sizeof(buff));
		// тут вывод со счетчиком каждую секунду
		//sprintf(buff, "%d %s\r\n", (int)meows_barks_counter, flag_meows ? "meow \0\0" : "bark \0\0");
		Send_buff(buff);
	}

	uint8_t x = (GPIOA->DATA & PA7_MSK) ? 0 : 1;
	static float Ysm = 1000.0f;
	const float k = 0.2f;

	Ysm = Ysm + k * (x * 1000.0f - Ysm);

	if(Ysm > 1000.0f) Ysm = 1000.0f;
	else if(Ysm < 0.0f) Ysm = 0.0f;

	// 1 отпущена, 0 нажата
	static uint8_t button_state = 1;
	static uint8_t prev_button_state = 1;

	if (Ysm > 700)
	{
		button_state = 1;
	}
	else if (Ysm < 500)
	{
		button_state = 0;
	}


	if (button_state == 0 && prev_button_state == 1)
	{

		if(button_click_counter < (MODES_COUNT - 1))
		{
			button_click_counter++;
		}
		else
		{
			button_click_counter = 1;
		}
	}

	prev_button_state = button_state;

	if (last_led_update >= 30)
	{
		update_leds();
		// GPIOB->DATAOUTTGL = PB0_MSK;
		last_led_update = 0;
	}
	else{last_led_update++;}

	TMR32->IC = 3;
}

void SPI0_IRQHandler()
{
	GPIOA->DATAOUTTGL = 0xFF00;
	SPI0->ICR = 0x3;
}
