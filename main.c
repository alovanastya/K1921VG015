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


/*
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
*/

void BSP_led_init()
{
	//Разрешаем тактирование GPIOA
	RCU->CGCFGAHB_bit.GPIOAEN = 1;
	//Включаем  GPIOA
	RCU->RSTDISAHB_bit.GPIOAEN = 1;
    GPIOA->OUTENSET = LEDS_MSK;
	GPIOA->DATAOUTSET = LEDS_MSK;
}

//-- Peripheral init functions -------------------------------------------------
/*
 * PC4  CS
 * PC5  CLK
 * PC6  MOSI
 * PC7  MISO
 */

void SPI1_IRQHandler()
{
	// GPIOA->DATAOUTTGL = 0xFF00;
	SPI1->ICR = 0x3;
}

void spi1_init()
{
	RCU->CGCFGAHB_bit.GPIOCEN = 1;  //Разрешение тактирования порта GPIOB
	RCU->RSTDISAHB_bit.GPIOCEN = 1; //Вывод из состояния сброса порта GPIOB
	RCU->CGCFGAHB_bit.SPI1EN = 1;  //Разрешение тактирования SPI0
	RCU->RSTDISAHB_bit.SPI1EN = 1; //Вывод из состояния сброса SPI0
	RCU->SPICLKCFG[1].SPICLKCFG_bit.CLKSEL = RCU_SPICLKCFG_CLKSEL_HSE; //Источник сигнала внешний кварц
	RCU->SPICLKCFG[1].SPICLKCFG_bit.CLKEN = 1; 	//Разрешение тактирования
	RCU->SPICLKCFG[1].SPICLKCFG_bit.RSTDIS = 1; //Вывод из сброса
	SPI1->CPSR_bit.CPSDVSR = 8;//Коэффициент деления первого делителя
	SPI1->CR0_bit.SCR = 4; //Коэффициент деления второго делителя. Результирующий коэффициент SCK/((SCR+1)*CPSDVSR) 16/((4+1)*8)=400кГц

	SPI1->CR0_bit.SPO = 0; //Полярность сигнала. В режиме ожидания линия в состоянии логического нуля.
	SPI1->CR0_bit.SPH = 1; //Фаза сигнала. Выборка данных по заднему фронту синхросигнала, а установка по переднему
	SPI1->CR0_bit.FRF = 0; //Выбор протокола обмена информацией 0-SPI
	SPI1->CR0_bit.DSS = 7; //Размер слова данных 8 бит
	SPI1->CR1_bit.MS = 0; //Режим работы - Мастер
	GPIOC->ALTFUNCSET = GPIO_ALTFUNCSET_PIN4_Msk | GPIO_ALTFUNCSET_PIN5_Msk | GPIO_ALTFUNCSET_PIN6_Msk | GPIO_ALTFUNCSET_PIN7_Msk;//Переводим младшие 4 пина порта GPIOB в режим альтернативной функции
	GPIOC->ALTFUNCNUM = (GPIO_ALTFUNCNUM_PIN4_AF2<<GPIO_ALTFUNCNUM_PIN4_Pos) | (GPIO_ALTFUNCNUM_PIN5_AF2<<GPIO_ALTFUNCNUM_PIN5_Pos) |
						(GPIO_ALTFUNCNUM_PIN6_AF2<<GPIO_ALTFUNCNUM_PIN6_Pos) | (GPIO_ALTFUNCNUM_PIN7_AF2<<GPIO_ALTFUNCNUM_PIN7_Pos); //Выбор номера альтернативной функции
	SPI1->IMSC = 0x1; //Разрешаем прерывания по переполнению приемного буфера
	// Настраиваем обработчик прерывания для SPI0
	PLIC_SetIrqHandler (Plic_Mach_Target, IsrVect_IRQ_SPI1, SPI1_IRQHandler);
	PLIC_SetPriority(IsrVect_IRQ_SPI1, 0x1);
	PLIC_IntEnable     (Plic_Mach_Target, IsrVect_IRQ_SPI1);

	SPI1->CR1_bit.SSE = 1; //Разрешение работы приемопередатчика
}

void MR45V256_CS_Enable()
{
    GPIOC->DATAOUTCLR = (1 << 4); // PC4 - CS
}

void MR45V256_CS_Disable()
{
    GPIOC->DATAOUTSET = (1 << 4);
}

void MR45V256_WREN()
{
    MR45V256_CS_Enable();
    // Transmit FIFO Not Ful
    // TNF = 0 буфер полон
    while (!(SPI1->SR & SPI_SR_TNF_Msk));
    SPI1->DR = 0x06; // WREN 0110
    while (SPI1->SR & SPI_SR_BSY_Msk);
    MR45V256_CS_Disable();
}

void MR45V256_WRDI()
{
    MR45V256_CS_Enable();
    while (!(SPI1->SR & SPI_SR_TNF_Msk));
    SPI1->DR = 0x04; // WRDI 100
    while (SPI1->SR & SPI_SR_BSY_Msk);
    MR45V256_CS_Disable();
}

uint8_t MR45V256_RDSR()
{
    uint8_t status;
    MR45V256_CS_Enable();
    while (!(SPI1->SR & SPI_SR_TNF_Msk));
    SPI1->DR = 0x05; // RDSR 101
    // Receive FIFO Not Empty
    while (!(SPI1->SR & SPI_SR_RNE_Msk));
    status = SPI1->DR;
    while (SPI1->SR & SPI_SR_BSY_Msk);
    MR45V256_CS_Disable();
    return status;
}

void MR45V256_WRSR(uint8_t status)
{
    MR45V256_WREN();
    MR45V256_CS_Enable();
    while (!(SPI1->SR & SPI_SR_TNF_Msk));
    SPI1->DR = 0x01; // WRSR 0001
    while (!(SPI1->SR & SPI_SR_TNF_Msk));
    SPI1->DR = status;
    while (SPI1->SR & SPI_SR_BSY_Msk);
    MR45V256_CS_Disable();
}


void MR45V256_Write(uint32_t address, uint8_t *data, uint32_t len)
{
	MR45V256_WREN();

	uint8_t status = MR45V256_RDSR();
	if (!(status & 0x02))
	{
	    printf(" Write failed\r\n");
	}

    while (MR45V256_RDSR() & 0x01);
    MR45V256_CS_Enable();

    while (!(SPI1->SR & SPI_SR_TNF_Msk));
    SPI1->DR = 0x02; // WRITE

    while (!(SPI1->SR & SPI_SR_TNF_Msk));
    SPI1->DR = (address >> 16) & 0xFF;
    while (!(SPI1->SR & SPI_SR_TNF_Msk));
    SPI1->DR = (address >> 8) & 0xFF;
    while (!(SPI1->SR & SPI_SR_TNF_Msk));
    SPI1->DR = address & 0xFF;


    for (uint32_t i = 0; i < len; i++)
    {
        while (!(SPI1->SR & SPI_SR_TNF_Msk));
        SPI1->DR = data[i];
    }

    while (SPI1->SR & SPI_SR_BSY_Msk);
    MR45V256_CS_Disable();

    while (MR45V256_RDSR() & 0x01);
}

void MR45V256_Read(uint32_t address, uint8_t *data, uint32_t len)
{
    MR45V256_CS_Enable();

    while (!(SPI1->SR & SPI_SR_TNF_Msk));
    SPI1->DR = 0x03; // READ

    while (!(SPI1->SR & SPI_SR_TNF_Msk));
    SPI1->DR = (address >> 16) & 0xFF;
    while (!(SPI1->SR & SPI_SR_TNF_Msk));
    SPI1->DR = (address >> 8) & 0xFF;
    while (!(SPI1->SR & SPI_SR_TNF_Msk));
    SPI1->DR = address & 0xFF;

    for (uint32_t i = 0; i < len; i++)
    {
        while (!(SPI1->SR & SPI_SR_TNF_Msk));
        SPI1->DR = 0x00;

        while (!(SPI1->SR & SPI_SR_RNE_Msk));
        data[i] = SPI1->DR;
    }

    while (SPI1->SR & SPI_SR_BSY_Msk);
    MR45V256_CS_Disable();
}

void adcsar_init()
{
  // настройка питания ADCSAR
  PMUSYS->ADCPWRCFG_bit.LDOEN = 1;
  PMUSYS->ADCPWRCFG_bit.LVLDIS = 0;

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
  ADCSAR->ACTL_bit.CALEN = 1;
  ADCSAR->ACTL_bit.ADCEN = 1;

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

void UART1_init()
{
    uint32_t baud_icoef = HSECLK_VAL / (16 * UART1_BAUD);
    uint32_t baud_fcoef = ((HSECLK_VAL / (16.0f * RETARGET_UART_BAUD) - baud_icoef) * 64 + 0.5f);

    RCU->CGCFGAHB_bit.GPIOAEN = 1;  // включает такирование AHB
    RCU->RSTDISAHB_bit.GPIOAEN = 1; // снимает сброс
    RCU->CGCFGAPB_bit.UART1EN = 1;  // тактирование uart1
    RCU->RSTDISAPB_bit.UART1EN = 1; // снимает сброс с uart1
    GPIOA->ALTFUNCNUM_bit.PIN2 = 1;
    GPIOA->ALTFUNCNUM_bit.PIN3 = 1;
    GPIOA->ALTFUNCSET = GPIO_ALTFUNCSET_PIN2_Msk | GPIO_ALTFUNCSET_PIN3_Msk;
    RCU->UARTCLKCFG[1].UARTCLKCFG_bit.CLKSEL = RCU_UARTCLKCFG_CLKSEL_HSE;

    // DIVEN - отключает дополнительный делитель частоты
    RCU->UARTCLKCFG[1].UARTCLKCFG_bit.DIVEN = 0;
    // RSTDIS - снимает сброс
    RCU->UARTCLKCFG[1].UARTCLKCFG_bit.RSTDIS = 1;
    // включает тактирование
    RCU->UARTCLKCFG[1].UARTCLKCFG_bit.CLKEN = 1;

    UART1->IBRD = baud_icoef; // записывасет дробную часть

    UART1->FBRD = baud_fcoef;
    UART1->LCRH = UART_LCRH_FEN_Msk | (3 << UART_LCRH_WLEN_Pos);
    UART1->IFLS = 0;
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
	spi1_init();

	sprintf(buff, "-------------------------------------------------------------------------------"); Send_buff(buff);
	sprintf(buff, "   K1921VG015 SYSCLK = %d MHz\r\n\0",(int)(SystemCoreClock / 1E6)); 	Send_buff(buff);
	sprintf(buff, "  UID[0] = 0x%X  UID[1] = 0x%X  UID[2] = 0x%X  UID[3] = 0x%X\r\n\0",PMUSYS->UID[0],PMUSYS->UID[1],PMUSYS->UID[2],PMUSYS->UID[3]); Send_buff(buff);
    sprintf(buff, "  PartNum = 0x%X\r\n\0",(uint16_t)(PMUSYS->UID[3] >> 16)); Send_buff(buff);
    sprintf(buff, "  Start UART DMA\r\n\0"); Send_buff(buff);
    sprintf(buff, "-------------------------------------------------------------------------------\n"); Send_buff(buff);
}

//--- USER FUNCTIONS ----------------------------------------------------------------------
volatile uint32_t led_shift;
volatile uint32_t cnt_tgl_meows;

//-- Main ----------------------------------------------------------------------
int main(void)
{
	periph_init();
	TMR32_init(500000); // 50 000 000 частота ядра, 500 000 10мс
	TMR1_PWN_init(SystemCoreClock>>1);
	InterruptEnable();
	led_shift = LED0_MSK;

	/* uint8_t write_data[4] = {0xAA, 0xBB, 0xCC, 0xDD};
	uint8_t read_data[4] = {0};

	MR45V256_Write(0x000000, write_data, 4);
	MR45V256_Read(0x000000, read_data, 4);

	printf("   Read data: %02X %02X %02X %02X\r\n", read_data[0], read_data[1], read_data[2], read_data[3]);
*/

	printf("    Testing MR45V256...\r\n");
	uint8_t test_byte = 0xA5;
	uint8_t read_byte = 0;
	MR45V256_Write(0x000000, &test_byte, 1);
	MR45V256_Read(0x000000, &read_byte, 1);
	printf("   Wrote 0x%02X, Read 0x%02X\r\n", test_byte, read_byte);

	uint8_t test_data[4] = {0x01, 0x02, 0x03, 0x04};
	uint8_t read_data[4] = {0};
	MR45V256_Write(0x000100, test_data, 4);
	MR45V256_Read(0x000100, read_data, 4);
	printf("   Wrote %02X %02X %02X %02X, Read %02X %02X %02X %02X\r\n",
	       test_data[0], test_data[1], test_data[2], test_data[3],
	       read_data[0], read_data[1], read_data[2], read_data[3]);

	while (1)
	{
		ADCSAR->SEQSYNC_bit.GSYNC = 1;

		    while ((ADCSAR->BSTAT));
		    while (!(ADCSAR->RIS_bit.SEQRIS0)); // Ожидание флага прерывания секвенсора 0
		    int chn = 1;
		    int ch_res = ADCSAR->SEQ[0].SFIFO;

		    //int curr_voltage_mV = (ch_res * 3300) / 4095;
		    //printf("  CH%d=%d.%dV\r", chn, curr_voltage_mV / 1000, (curr_voltage_mV % 1000) / 100);

		    ADCSAR->IC = ADCSAR_IC_SEQIC0_Msk; // Сброс флага прерывания секвенсора 0
		    uint8_t status = MR45V256_RDSR();
		    printf("   status: 0x%02X\r   ", status);
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
	GPIOA->DATAOUTTGL = led_shift;
		led_shift = led_shift << 1;
	    if(led_shift > LED7_MSK) {
	      led_shift = LED2_MSK;
	      // Изменяем скважность сигнала ШИМ
	      TMR1->CAPCOM[2].VAL += 0x1000;
	      TMR1->CAPCOM[3].VAL -= 0x1000;
	    }
	    //Сбрасываем флаг прерывания таймера
	    TMR32->IC = 3;
	/*
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

	if (Ysm > 700){	button_state = 1;}
	else if (Ysm < 500){ button_state = 0; }

	if (button_state == 0 && prev_button_state == 1)
	{
		if(button_click_counter < (MODES_COUNT - 1))	{button_click_counter++; }
		else{	button_click_counter = 1;}
	}

	prev_button_state = button_state;
	if (last_led_update >= 30)
	{
		update_leds();
		last_led_update = 0;
	}
	else{last_led_update++;}

	TMR32->IC = 3;
	*/
}

