#include <K1921VG015.h>
#include <stdint.h>
#include <stdio.h>
#include <system_k1921vg015.h>
#include "retarget.h"

#define CNT_SAMPLE 5

#define UART1_BAUD  115200


#define GPIOA_ALL_Msk  0xFFFF
#define GPIOB_ALL_Msk  0xFFFF

#define LEDS_MSK  0xFF00
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

char buff[120];

void TMR32_IRQHandler();

void Send_buff(char* a)
{
	uint8_t i = 0;
	while ((i < 120) && (a[i] != 0))
	{
		retarget_put_char(a[i]);
		i++;
	}
}

void BSP_led_init()
{
	// CGCFGAHB регистр конфигурации тактирования для шины AHB
	RCU->CGCFGAHB_bit.GPIOAEN = 1; //  Разрешаем тактирование GPIOA

	// RSTDISAHB - регистр снятия сброса для шины AHB
	RCU->RSTDISAHB_bit.GPIOAEN = 1; //Включаем  GPIOA

	GPIOA->OUTENSET = LEDS_MSK; // OUTENSET - регистр, который настраивает выводы как выходы
	// GPIOA->DATAOUTSET = LEDS_MSK;
	GPIOA->DATAOUTCLR = LEDS_MSK;   // гасит диоды


	GPIOA->OUTENCLR = PA7_MSK;    // Кнопка как вход
	// PULLMODE – регистр выбора режима подтяжки порта
	// 0b00111..11 -> 14 и 15 зануляюся
	GPIOA->PULLMODE &= ~(0b11 << (7 * 2));  // стр 367
	GPIOA->PULLMODE |= (0b01 << (7 * 2));  // Подтяжка к уровню логической единицы

	GPIOA->OUTENSET = PA5_MSK;  // включение вывода, теперь как выход

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

    // Fractional Baud Rate Divisor
    UART1->IBRD = baud_icoef; // записывасет дробную часть

    // Line Control Register High
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
	//BSP_led_init();
	SystemInit();
	SystemCoreClockUpdate();
	//BSP_led_init();
	//retarget_init();
	UART1_init();
	// printf("K1921VG015 SYSCLK = %d MHz\n", (int)(SystemCoreClock / 1E6));
	// printf("  UID[0] = 0x%X  UID[1] = 0x%X  UID[2] = 0x%X  UID[3] = 0x%X\n", (unsigned int)PMUSYS->UID[0], (unsigned int)PMUSYS->UID[1], (unsigned int)PMUSYS->UID[2], (unsigned int)PMUSYS->UID[3]);
	// printf("  Start RunLeds\n");
}

//--- USER FUNCTIONS ----------------------------------------------------------------------
volatile uint32_t led_shift;

//-- Main ----------------------------------------------------------------------
int main(void)
{
	uint32_t	i;
	periph_init();
	// TMR32_init(SystemCoreClock>>10);  // если << 1, медленно
	//TMR32_init(500000); // 50 000 000 частота ядра, 500 000 10мс
	InterruptEnable();
	led_shift = LED0_MSK;
	while (1)
	{
		UART1->DR = 0x053;
	    for(i=0;i<100000; ++i)
	    {}
	}

	return 0;
}

//-- IRQ INTERRUPT HANDLERS ---------------------------------------------------------------
void TMR32_IRQHandler()
{
	static uint8_t d = 0;
	static uint8_t last_button_state = 1;
	uint8_t curr_button_state = (GPIOA->DATA & PA7_MSK) ? 1 : 0;


	if (curr_button_state != last_button_state)
	{
		d = 5;
	}
	else if (d > 0)
	{
		d--;

		// если счётчик дошёл до 0 и кнопка нажата
		if (d == 0 && curr_button_state == 0)
		{
			GPIOA->DATAOUTTGL = PA5_MSK; // PA5_MSK - фонарик, LED7_MSK - диод
			//Send_buff("Button pressed\n");
		}
	}

	last_button_state = curr_button_state;

	//Сбрасываем флаг прерывания таймера
	TMR32->IC = 3;
}

// GPIOA->DATAOUTTGL = led_shift;
// led_shift = led_shift << 1;  // бегающий огонек
// if(led_shift > LED7_MSK) led_shift = LED0_MSK;
//GPIOA->DATAOUTTGL = led_shift;
// GPIOA->DATAOUTTGL = LED7_MSK;  // 12
//GPIOA->DATAOUT = (LED4_MSK | LED5_MSK);  // горят оба диода
