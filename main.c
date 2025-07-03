/*==============================================================================
 * Пример работы GPIO для K1921VG015
 *------------------------------------------------------------------------------
 * НИИЭТ, Александр Дыхно <dykhno@niiet.ru>
 *==============================================================================
 * ДАННОЕ ПРОГРАММНОЕ ОБЕСПЕЧЕНИЕ ПРЕДОСТАВЛЯЕТСЯ «КАК ЕСТЬ», БЕЗ КАКИХ-ЛИБО
 * ГАРАНТИЙ, ЯВНО ВЫРАЖЕННЫХ ИЛИ ПОДРАЗУМЕВАЕМЫХ, ВКЛЮЧАЯ ГАРАНТИИ ТОВАРНОЙ
 * ПРИГОДНОСТИ, СООТВЕТСТВИЯ ПО ЕГО КОНКРЕТНОМУ НАЗНАЧЕНИЮ И ОТСУТСТВИЯ
 * НАРУШЕНИЙ, НО НЕ ОГРАНИЧИВАЯСЬ ИМИ. ДАННОЕ ПРОГРАММНОЕ ОБЕСПЕЧЕНИЕ
 * ПРЕДНАЗНАЧЕНО ДЛЯ ОЗНАКОМИТЕЛЬНЫХ ЦЕЛЕЙ И НАПРАВЛЕНО ТОЛЬКО НА
 * ПРЕДОСТАВЛЕНИЕ ДОПОЛНИТЕЛЬНОЙ ИНФОРМАЦИИ О ПРОДУКТЕ, С ЦЕЛЬЮ СОХРАНИТЬ ВРЕМЯ
 * ПОТРЕБИТЕЛЮ. НИ В КАКОМ СЛУЧАЕ АВТОРЫ ИЛИ ПРАВООБЛАДАТЕЛИ НЕ НЕСУТ
 * ОТВЕТСТВЕННОСТИ ПО КАКИМ-ЛИБО ИСКАМ, ЗА ПРЯМОЙ ИЛИ КОСВЕННЫЙ УЩЕРБ, ИЛИ
 * ПО ИНЫМ ТРЕБОВАНИЯМ, ВОЗНИКШИМ ИЗ-ЗА ИСПОЛЬЗОВАНИЯ ПРОГРАММНОГО ОБЕСПЕЧЕНИЯ
 * ИЛИ ИНЫХ ДЕЙСТВИЙ С ПРОГРАММНЫМ ОБЕСПЕЧЕНИЕМ.
 *
 *                              2024 АО "НИИЭТ"
 *==============================================================================
 */

//-- Includes ------------------------------------------------------------------
#include <K1921VG015.h>
#include <stdint.h>
#include <stdio.h>
#include <system_k1921vg015.h>
#include "retarget.h"

//-- Defines -------------------------------------------------------------------
#define GPIOA_ALL_Msk	0xFFFF
#define GPIOB_ALL_Msk	0xFFFF

#define LEDS_MSK	0xFF00
#define PA7_MSK	    (1 << 7)
#define LED0_MSK	(1 << 8)
#define LED1_MSK	(1 << 9)
#define LED2_MSK	(1 << 10)
#define LED3_MSK	(1 << 11)
#define LED4_MSK	(1 << 12)  // PA12
#define LED5_MSK	(1 << 13)
#define LED6_MSK	(1 << 14)
#define LED7_MSK	(1 << 15)  // PA15

char buff[120];

void TMR32_IRQHandler();

void Send_buff(char *a)
{
  uint8_t i=0;
  while ((i<120) && (a[i]!=0))
  {
	  retarget_put_char(a[i]);
    i++;
  }
}

void BSP_led_init()
{

	RCU->CGCFGAHB_bit.GPIOAEN = 1; //Разрешаем тактирование GPIOA

	RCU->RSTDISAHB_bit.GPIOAEN = 1; //Включаем  GPIOA

    GPIOA->OUTENSET = LEDS_MSK;
	// GPIOA->DATAOUTSET = LEDS_MSK;
    GPIOA->DATAOUTCLR = LEDS_MSK;

    GPIOB->OUTENCLR = PA7_MSK;    // Кнопка
    GPIOA->DATAOUTCLR = PA7_MSK;



}

void TMR32_init(uint32_t period)
{
  RCU->CGCFGAPB_bit.TMR32EN = 1;
  RCU->RSTDISAPB_bit.TMR32EN = 1;

  //Записываем значение периода в CAPCOM[0]
  TMR32->CAPCOM[0].VAL = period-1;
  //Выбираем режим счета от 0 до значения CAPCOM[0]
  TMR32->CTRL_bit.MODE = 1;

  //Разрешаем прерывание по совпадению значения счетчика и CAPCOM[0]
  TMR32->IM = 2;

  // Настраиваем обработчик прерывания для TMR32
  PLIC_SetIrqHandler (Plic_Mach_Target, IsrVect_IRQ_TMR32, TMR32_IRQHandler);
  PLIC_SetPriority   (IsrVect_IRQ_TMR32, 0x1);
  PLIC_IntEnable     (Plic_Mach_Target, IsrVect_IRQ_TMR32);
}

//-- Peripheral init functions -------------------------------------------------
void periph_init()
{
	BSP_led_init();
	SystemInit();
	SystemCoreClockUpdate();
	BSP_led_init();
	retarget_init();
	printf("K1921VG015 SYSCLK = %d MHz\n",(int)(SystemCoreClock / 1E6));
	printf("  UID[0] = 0x%X  UID[1] = 0x%X  UID[2] = 0x%X  UID[3] = 0x%X\n",(unsigned int)PMUSYS->UID[0], (unsigned int)PMUSYS->UID[1], (unsigned int)PMUSYS->UID[2], (unsigned int)PMUSYS->UID[3]);
  printf("  Start RunLeds\n");
}

//--- USER FUNCTIONS ----------------------------------------------------------------------


volatile uint32_t led_shift;
//-- Main ----------------------------------------------------------------------
int main(void)
{
  periph_init();
  TMR32_init(SystemCoreClock>>1);  // если << 1, медленно
  InterruptEnable();
  led_shift = LED0_MSK;
  while(1)
  {
  }

  return 0;
}


//-- IRQ INTERRUPT HANDLERS ---------------------------------------------------------------
void TMR32_IRQHandler()
{
    // GPIOA->DATAOUTTGL = led_shift;
    // led_shift = led_shift << 1;
    // if(led_shift > LED7_MSK) led_shift = LED0_MSK;

	//GPIOA->DATAOUTTGL = led_shift;

	  GPIOA->DATAOUTTGL = LED7_MSK;  // 12
	//GPIOA->DATAOUTSET = LED5_MSK;  // 13


	//GPIOA->DATAOUT = (LED4_MSK | LED5_MSK);

    //Сбрасываем флаг прерывания таймера
    TMR32->IC = 3;
}
