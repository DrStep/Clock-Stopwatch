#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "system_stm32f10x.h"
#include "stm32f10x_conf.h"
#include <stdio.h>
#include <stdlib.h>
#define FLASH_KEY1 ((uint32_t)0x45670123)
#define FLASH_KEY2 ((uint32_t)0xCDEF89AB)
#define PAGE_127 0x0801FC00
#define BYTES 4
// ���� � �������� ��������� ���������
#define IND_PORT GPIOB
// ����� ������ ����������
#define D0 GPIO_Pin_8
#define D1 GPIO_Pin_9
#define D2 GPIO_Pin_10
#define D3 GPIO_Pin_11
// � ����� ���� ����� ������� ���������
#define SEG_A GPIO_Pin_0
#define SEG_B GPIO_Pin_1
#define SEG_C GPIO_Pin_2
#define SEG_D GPIO_Pin_3
#define SEG_E GPIO_Pin_4
#define SEG_F GPIO_Pin_5
#define SEG_G GPIO_Pin_6
#define SEG_DT GPIO_Pin_7
//�������� ����� �� ���������
#define DIG0 ( SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F )
#define DIG1 ( SEG_B | SEG_C )
#define DIG2 ( SEG_A | SEG_B | SEG_G | SEG_E | SEG_D )
#define DIG3 ( SEG_A | SEG_B | SEG_G | SEG_C | SEG_D )
#define DIG4 ( SEG_F | SEG_G | SEG_B | SEG_C)
#define DIG5 ( SEG_A | SEG_F | SEG_G | SEG_C | SEG_D )
#define DIG6 ( SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G )
#define DIG7 ( SEG_A | SEG_B | SEG_C )
#define DIG8 ( SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G)
#define DIG9 ( SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G)
#define ALL_PINS (DIG8 | D0 | D1 | D2 | D3 )

typedef struct
{
	uint32_t hour; // ����
	uint32_t min; // ������
	uint32_t sec; // �������
} RTC_Time;
uint32_t clock_mode, settings_mode, timer_started, timer_cnt, current_address,
current_setup_numb, seck_result_num;
RTC_Time time = { 0, 0, 0};

//������������� ������� � ������
void flash_unlock(void) {
  FLASH->KEYR = FLASH_KEY1;
  FLASH->KEYR = FLASH_KEY2;
}
//���������� ������� � ������
void flash_lock() {
  FLASH->CR |= FLASH_CR_LOCK;
}

//������� ��������� true ����� ����� ������� ��� ������ ������.
uint8_t flash_ready(void) {
  return !(FLASH->SR & FLASH_SR_BSY);
}

//������� ������� ���� ��������. � �������� ������ ����� ������������ �����
//������������� ��������� ������� ��� �������� ������� ����� ��������.
void flash_erase_page(uint32_t address) {
  FLASH->CR|= FLASH_CR_PER; //������������� ��� �������� ����� ��������
  FLASH->AR = address; // ������ � �����
  FLASH->CR|= FLASH_CR_STRT; // ��������� ��������
  while(!flash_ready());  //���� ���� �������� ��������.
  FLASH->CR&= ~FLASH_CR_PER; //���������� ��� �������
}

//������ � ������
void flash_write(uint32_t address,uint32_t data) {
  flash_unlock();
  FLASH->CR |= FLASH_CR_PG; //��������� ���������������� �����
  while(!flash_ready()); //������� ���������� ����� � ������
  *(__IO uint16_t*)address = (uint16_t)data; //����� ������� 2 ����
  while(!flash_ready());
  address+=2;
  data>>=16;
  *(__IO uint16_t*)address = (uint16_t)data; //����� ������� 2 �����
  while(!flash_ready());
  FLASH->CR &= ~(FLASH_CR_PG); //��������� ���������������� �����
  flash_lock();
}

uint32_t flash_read(uint32_t address) {
  return (*(__IO uint32_t*) address);
}

void delay(void) {
	volatile uint32_t i;
	for (i=0; i != 0x1000; i++);
}

//������� ������� � ���� ������ �����
void digit_to_port (uint8_t digit) {
	uint8_t digitsp[]={DIG0,DIG1,DIG2,DIG3,DIG4,DIG5,DIG6,DIG7,DIG8,DIG9};
	IND_PORT->ODR &= ~DIG8; //��������� ��� ��������
	IND_PORT->ODR |= digitsp[digit]; //�������� ������
}

//���������� �������
void TIM6_DAC_IRQHandler(void)
{
	++timer_cnt;
	TIM6->SR &= ~TIM_SR_UIF; //���������� ���� UIF
}

//���������� ������
void EXTI0_IRQHandler(void) {		 // ���� | ����������
	 EXTI->PR|=0x01;				//clear flag
	 clock_mode = !clock_mode;

}

void EXTI1_IRQHandler(void) {		// ����� ��������� �����/������� ����� | ����/�����
	uint32_t i, result[29];
	 EXTI->PR|=0x02;
	 if (clock_mode){
		 settings_mode = !settings_mode;
	 } else {
		 timer_started = !timer_started;
		 if (timer_started) {
			 TIM6->CR1 |= TIM_CR1_CEN;		//������ �������
		 } else {
			 TIM6->CR1 &= ~TIM_CR1_CEN;		//���������� �������
			 for (i=0; i<29; i++) {			//�������� 29 ���������� �� 1 ������, � �������������� ������ ����� ����� ���������
				 result[i] = flash_read(PAGE_127 + BYTES*i);
			 }
			 flash_erase_page(PAGE_127);
			 flash_write(PAGE_127, timer_cnt);
			 for (i=1; i<30; i++) {
			 	 flash_write(PAGE_127 + BYTES*i, result[i]);
			 }
			 //����� �������
			 timer_cnt = 0;
		 }
	 }
}

void EXTI2_IRQHandler(void) {		// ����� ����� ��� ��������� ����� | ���������� ����������
	 EXTI->PR|=0x03;
	 GPIOC->BSRR = GPIO_Pin_9;		//������� ����� �����������, ����� ��������, ��� ������� ���������� �����
	 delay();
	 delay();
	 GPIOC->BRR = GPIO_Pin_9;
	 if (clock_mode && settings_mode) {
		 current_setup_numb++;
	 } else if (!clock_mode) {
		 seck_result_num++;
	 }
}

void EXTI3_IRQHandler(void) {		// ����� ����� ��� ��������� ������ | ��������� ����������
	 EXTI->PR|=0x04;
	 GPIOC->BSRR = GPIO_Pin_8;		//������� ������ �����������, ����� ��������, ��� ������� ���������� ������
	 delay();
	 delay();
	 GPIOC->BRR = GPIO_Pin_8;
	 if (clock_mode && settings_mode) {
	 		 current_setup_numb--;
	 	 } else if (!clock_mode) {
	 		 seck_result_num--;
	 	 }
}

void EXTI4_IRQHandler(void) {		// ��������� ��������� ������� �������
	 EXTI->PR|=0x05;
	 if (clock_mode && settings_mode) {
		 switch(current_setup_numb) {
		 case 0:
			 time.min++;
			 if (time.min == 60) {
				 time.min = 0;
			 }
			 break;
		 case 1:
			 if (time.min < 50) {
				 time.min +=10;
			 } else {
				time.min = time.min%10;
			 }
			 break;
		 case 2:
			 time.hour++;
			 if (time.hour == 24) {
				 time.hour = 0;
			 }
			 break;
		 case 3:
			 if (time.hour < 14) {
				 time.hour += 10;
			 } else {
				 time.hour = time.hour + 10 - 24;
			 }
		 }
	 }
}

void EXTI9_5_IRQHandler(void) {		// ��������� ��������� ������� �������
	 EXTI->PR|=0x06;
	 if (clock_mode && settings_mode) {
	 		 switch(current_setup_numb) {
	 		 case 0:
	 			 if (time.min != 0) {
	 				 time.min--;
	 			 } else {
	 				 time.min = 59;
	 			 }
	 			 break;
	 		 case 1:
	 			 if (time.min > 10) {
	 				 time.min -=10;
	 			 } else {
	 				time.min = time.min + 50;
	 			 }
	 			 break;
	 		 case 2:
	 			 if (time.hour != 0) {
	 				 time.hour--;
	 			 } else {
	 				 time.hour = 23;
	 			 }
	 			 break;
	 		 case 3:
	 			 if (time.hour > 10) {
	 				 time.hour -= 10;
	 			 } else {
	 				 time.hour = 24 - (10 - time.hour);
	 			 }
	 		 }
	 	 }
}

void RTC_IRQHandler(void)
{
 	if (++time.sec == 60) {
		time.sec = 0;
		if (++time.min == 60) {
			time.min = 0;
			if (++time.hour == 24) {
				time.hour = 0;
			}
		}
	}
    RTC->CRL &= ~RTC_CRL_SECF;
}

void initRTC(void)
{
  //if ((RCC->BDCR & RCC_BDCR_RTCEN) != RCC_BDCR_RTCEN) {
    RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;   //enable tackts and backup
    PWR->CR |= PWR_CR_DBP;					//enable access to backup zone
	RCC->BDCR |= RCC_BDCR_BDRST;
	RCC->BDCR &= ~RCC_BDCR_BDRST;		//reset backup

	RCC->BDCR |= RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_LSE;		//choose LSE for clock
    RCC->BDCR |= RCC_BDCR_LSEON;		//start Lse

	  //while( (RCC->BDCR & RCC_BDCR_LSERDY) == 0 ){}; // Wait for LSERDY = 1 (LSE is ready)
	while ( (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) ) {
		RCC_LSEConfig(RCC_LSE_ON);
	}
	BKP->RTCCR |= 3;    //calibrate
    while( (RTC->CRL & RTC_CRL_RTOFF) == 0 ) ; // Poll RTOFF, wait until its value goes to �1�

    RTC->CRL |= RTC_CRL_CNF;		//access to write in registers
    RTC->PRLH = 0;
    RTC->PRLL = 0x7FFF;				 //32768 Hz

    RTC->CRH &= ~RTC_CRH_OWIE; // ��������� ���������� ��� ������������ �������� ��������
    RTC->CRH &= ~RTC_CRH_ALRIE; // ��������� ���������� ��� ���������� �������� � ����������� ��������
    RTC->CRH |= RTC_CRH_SECIE;       //���������� ������ �� ��������� ���������

	/* NVIC_SetPriority & NVIC_EnableIRQ defined in core_cm3.h */
    //NVIC_SetPriority (RTC_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
    NVIC_EnableIRQ (RTC_IRQn);

    RTC->CRL &= ~RTC_CRL_CNF;
    while(  (RTC->CRL & RTC_CRL_RTOFF) == 0 ) ; // Poll RTOFF, wait until its value goes to �1� to check the end of the write operation.

    PWR->CR &= ~PWR_CR_DBP;

  //}
}

int main(void) {
	GPIO_InitTypeDef PORT;
	uint32_t temp, mig_cnt, mig_flag;

	RCC->CR |= RCC_CR_CSSON;

	//��������� JTAG (�� �������� ���� ������ ���)
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO , ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	//making tact for ports
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);

	//interruption is alternative function of ports
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_AFIOEN , ENABLE);

	//by default Port A generit interrupts, we say, that 0-5 pins can generates
	EXTI->IMR|=(EXTI_IMR_MR0 | EXTI_IMR_MR1 | EXTI_IMR_MR2 | EXTI_IMR_MR3 | EXTI_IMR_MR4 | EXTI_IMR_MR5);

	//interrupts when log1
	EXTI->RTSR|=(EXTI_RTSR_TR0 | EXTI_RTSR_TR1 | EXTI_RTSR_TR2 | EXTI_RTSR_TR3| EXTI_RTSR_TR4 | EXTI_RTSR_TR5);

	//Razreshaem interrupts
	NVIC_EnableIRQ (EXTI0_IRQn);
	NVIC_EnableIRQ (EXTI1_IRQn);
	NVIC_EnableIRQ (EXTI2_IRQn);
	NVIC_EnableIRQ (EXTI3_IRQn);
	NVIC_EnableIRQ (EXTI4_IRQn);
	NVIC_EnableIRQ (EXTI9_5_IRQn);

	//LED init
	PORT.GPIO_Pin = ALL_PINS; //��������� ����� ���� ����� ���������
	PORT.GPIO_Mode = GPIO_Mode_Out_PP; 	// ����������� ��� ����� Push-pull
	PORT.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init( IND_PORT , &PORT);

	PORT.GPIO_Pin = (GPIO_Pin_9 | GPIO_Pin_8);
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init( GPIOC , &PORT);

	//enable clock
	initRTC();

	//��������� �������
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
	TIM6->PSC = 24000 - 1; // ����������� �������� ��� ������ ����� 1000 ��� � �������
	timer_cnt = 0;			//������� �������
	TIM6->ARR = 100 ; // ���� ���������� ��������� ��� � ������� ���� �������
	TIM6->DIER |= TIM_DIER_UIE; //��������� ���������� �� �������
	NVIC_EnableIRQ(TIM6_DAC_IRQn); //���������� TIM6_DAC_IRQn ����������

	//others inits
	clock_mode = 1;			//��������
	settings_mode = 0;
	timer_started = 0;
	timer_cnt = 0;
	mig_cnt = 0;  			//������� ��� �������
	mig_flag = 0;
	current_setup_numb = 0;

	while (1) {
 		if (clock_mode){

			//��������� ��� �������
			IND_PORT->ODR &= ~(D0|D1|D2|D3);
			//�������� ������� ������ ����������
			IND_PORT->ODR |= D3;
			//������� ����� � ������� ������
			temp = time.min%10;
			if (settings_mode && current_setup_numb == 0) {
				if (mig_cnt != 0x5000) {
					mig_cnt++;
				} else {
					mig_flag = !mig_flag;
				}
				if (!mig_flag) {
					digit_to_port(temp);
				}
			} else {
				digit_to_port(temp);
			}
			delay();

			IND_PORT->ODR &= ~(D0|D1|D2|D3);
			IND_PORT->ODR |= D2;
			if (!clock_mode) {
				IND_PORT->ODR |= SEG_DT;
			} else {
				IND_PORT->ODR &= ~SEG_DT;
			}
			temp = time.min/10;
			digit_to_port(temp);
			delay();

			IND_PORT->ODR &= ~(D0|D1|D2|D3);
			IND_PORT->ODR |= D1;
			temp = time.hour%10;
			if (clock_mode) {
				IND_PORT->ODR |= SEG_DT;
			} else {
				IND_PORT->ODR &= ~SEG_DT;
			}
			digit_to_port(temp);
			delay();

			IND_PORT->ODR &= ~(D0|D1|D2|D3);
			IND_PORT->ODR |= D0;
			temp = time.hour/10;
			digit_to_port(temp);
			delay();
		}
	}
}
