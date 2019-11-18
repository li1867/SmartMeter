/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "stm32f0xx.h"
//#include "stm32f0_discovery.h"
#include "diag/Trace.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_usart.h"

#include "timer.h"
#include "BlinkLed.h"

// ----------------------------------------------------------------------------
//
// Standalone STM32F0 led blink sample (trace via DEBUG).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// Then demonstrates how to blink a led with 1 Hz, using a
// continuous loop and SysTick delays.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=8000000.
//
// Note: The default clock settings take the user defined HSE_VALUE and try
// to reach the maximum possible system clock. For the default 8 MHz input
// the result is guaranteed, but for other values it might not be possible,
// so please adjust the PLL settings in system/src/cmsis/system_stm32f0xx.c
//

// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 3 / 4)
#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ - BLINK_ON_TICKS)

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

char line[3];
int seconds = 0;

int currentBuff = 0;
int voltageBuff = 0;

volatile uint32_t curPowerFactorCt = 0;
volatile uint32_t volPowerFactorCt = 0;
volatile int counter = 0;

volatile int serverConnected = 1;
int out = 0;

volatile int ts = 0;

volatile clock_t start;

void uart2_send_buff(uint8_t buf[],uint32_t len) {
    uint32_t i;
    for(i=0;i<len;i++) {
    	USART_SendData(USART2,(uint16_t)buf[i]);
    	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
    }
    USART_SendData(USART2,(uint16_t)'\r');
    while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
    USART_SendData(USART2,(uint16_t)'\n');
    while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
}

int receive_uart_frame() {
    uint32_t n = 1;
    char ch[20] = "";
    ch[0]=' ';
    ch[19]='\0';
    do {
    	while(!USART_GetFlagStatus(USART2,USART_FLAG_RXNE) && !USART_GetFlagStatus(USART2,USART_FLAG_ORE));
        USART_ClearFlag(USART2,USART_FLAG_ORE);
    	ch[n] =  (char)USART_ReceiveData(USART2);
        if(n==18){
        	n = 1;
        	ch[0] = ch[18];
        }
        else{
        	n++;
        }
        //check the reply message from ESP8266, if the message ended with OK, then move to the next command.
        if(ch[n-1]=='K' || ch[n-1] == 'O'){
        	return 0;
        }
    } while (1);
    return 1;
}


int ESP8266_SendCmd(char* cmd)
{
    uart2_send_buff((uint8_t*)cmd, strlen(cmd));
    if (receive_uart_frame("OK") == 0) {
    	return 0;
    }
    return 1;
}


void ESP8266_JoinAP(char* ssid, char* psd)
{
    int ret = 0;
    char ssid_psd[120] = {0};
    char requestIPCmd[120] = {0};
    sprintf(ssid_psd,"AT+CWJAP=\"%s\",\"%s\"",ssid, psd);
    ESP8266_SendCmd(ssid_psd);
}


void serverCommunication(char * data) {
	char sendCmd[120] = {0};
	sprintf(sendCmd, "AT+CIPSEND=%u",strlen(data));
	ESP8266_SendCmd(sendCmd);
	uart2_send_buff((uint8_t*)data, strlen(data));
}


void ESP8266_ConnectToServer(char* ip, char* port) {
	char destination[120] = {0};
	sprintf(destination, "AT+CIPSTART=\"TCP\",\"%s\",%s", ip, port);
	ESP8266_SendCmd(destination);
}


void ESP8266_Init() {
    ESP8266_SendCmd("AT+CWMODE=1");
    receive_uart_frame("OK");

}

void ESP8266_CloseServer() {
	ESP8266_SendCmd("AT+CIPCLOSE");
}

int vbuffer[40] = {0};
int cbuffer[40] = {0};
int bufferCount = 0;

int pfCount = 0;
int vpf = 0;  //sum of voltage power factor
int cpf = 0;  //sum of current power factor
int pfSum = 0;
int volOffset = 0;

void TIM2_IRQHandler(void) {
//	if (counter == 40000) {
////		ESP8266_ConnectToServer("54.191.41.90", "1234");
//		//trace_printf("here1\n");
//		ts += 1;
//		if (ts == 86400) {
//			ts = 0;
//		}
//		counter = 0;
//		char * dataSend[120];
//		int powerFactor = 0;
//		powerFactor = abs(volPowerFactorCt - curPowerFactorCt);
////		trace_printf("%d\n", voltageBuff);
////		sprintf(dataSend, "POST / ?meter_id=1&timestamp=%d&voltage=%d&current=%d&power_factor=%d HTTP/1.0\r\n \r\n", ts, voltageBuff, currentBuff, powerFactor);
//		char sendCmd[120] = {0};
////		sprintf(sendCmd, "AT+CIPSEND=%u",strlen(dataSend));
////		ESP8266_SendCmd(sendCmd);
//		trace_printf("here\n");
////		uart2_send_buff((uint8_t*)dataSend, strlen(dataSend));
//		currentBuff = 0;
//		voltageBuff = 0;
////		ESP8266_CloseServer();
//	}

	if (counter == 1000) {
		vbuffer[bufferCount] = voltageBuff;
		cbuffer[bufferCount] = currentBuff;
		voltageBuff = 0;
		currentBuff = 0;
		bufferCount += 1;
		counter = 0;
	}

	if (bufferCount == 40) {
		//ESP8266_ConnectToServer("10.186.149.12", "3000");
		trace_printf("%d\n", pfCount);
		pfCount = 0;
		ts += 1;
		if (ts == 86400) {
			ts = 0;
		}
		char * vdataTemp[150] = {};
		char * cdataTemp[150] = {};
		for (int i = 0; i < 40; i++) {
			char * data[12] = {};
			sprintf(data, "%d ", vbuffer[i]);
			strcat(vdataTemp, data);
			char * data2[12] = {};
			sprintf(data2, "%d ", cbuffer[i]);
			strcat(cdataTemp, data2);
		}

		char * strHeader[1000] = {};
		sprintf(strHeader, "POST / ?meter_id=1&");

		char * timestamp[20] = {};
		sprintf(timestamp, "timestamp=%d", ts);
		strcat(strHeader, timestamp);

		char * volStr[100] = {};
		strcpy(volStr, "&voltage=");
		strcat(volStr, vdataTemp);
		strcat(strHeader, volStr);

		char * voltageOffset[10] = {};
		sprintf(voltageOffset, "&volOffset=%d", volOffset);
		strcat(strHeader, voltageOffset);
		trace_printf("%d\n", volOffset);
		volOffset = 0;

		char * curStr[100] = {};
		strcpy(curStr, "&current=");
		strcat(curStr, cdataTemp);
		strcat(strHeader, curStr);

		char * powerFactor[10] = {};
		sprintf(powerFactor, "&power_factor=%d", pfSum);
		strcat(strHeader, powerFactor);
		pfSum = 0;

		char * strEnd = " HTTP/1.0\r\n \r\n";
		strcat(strHeader, strEnd);

		char * sendCmd[10] = {};
		sprintf(sendCmd, "AT+CIPSEND=%u",strlen(strHeader));
		//ESP8266_SendCmd(sendCmd);
		uart2_send_buff((uint8_t*)strHeader, strlen(strHeader));
		trace_printf("here\n");
		bufferCount = 0;
		//ESP8266_CloseServer();
	}

	//PA4 for current
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);
	ADC1->CHSELR = 0;  //reset ADC1
	ADC1->CHSELR |= 1 << 4;  //enable channel 4 as ADC input
	//wait until ADC is ready
	while (!(ADC1->ISR & ADC_ISR_ADRDY));
	ADC1->CR |= ADC_CR_ADSTART;
	while(!(ADC1->ISR & ADC_ISR_EOC));
	//take the ADC1 channel 4 input and save the input to the currentValue array
	out=ADC1->DR;
	if (out >= 1930 || out <= 1950) {
		cpf = counter;
	}
	out = out - 1940;
	currentBuff += out * out;

	//PA5 for voltage
	ADC1->CHSELR = 0;
	ADC1->CHSELR |= 1 << 5;
	while (!(ADC1->ISR & ADC_ISR_ADRDY));
	ADC1->CR |= ADC_CR_ADSTART;
	while(!(ADC1->ISR & ADC_ISR_EOC));
	//take the ADC1 channel 5 input and save the input to the currentValue array
	out=ADC1->DR;
	out = out - 2048;
	volOffset += out;
	voltageBuff += out * out;
	if ((2044 - 2048) <= (out) && (2052 - 2048) >= (out)) {
		vpf = counter;
		pfCount += 1;
	}
	pfSum += abs(vpf - cpf);
	counter += 1;
}

void tim2_init(void) {
	RCC->APB1ENR|=RCC_APB1ENR_TIM2EN;
	TIM_TimeBaseInitTypeDef Tinit;
	Tinit.TIM_ClockDivision=0;
	Tinit.TIM_CounterMode=TIM_CounterMode_Up;
	Tinit.TIM_Period=1;
	Tinit.TIM_Prescaler = 120;
	Tinit.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM2,&Tinit);
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    NVIC->ISER[0]|=1<<TIM2_IRQn ;
}

void ADC_init(void) {
	//ADC enable
   RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
   GPIOA->MODER |= 0xf00;
   RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
   RCC->CR2 |= RCC_CR2_HSI14ON;
   while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0);
   ADC1->CR |= ADC_CR_ADEN;
   while ((ADC1->ISR & ADC_ISR_ADRDY) == 0);
   while((ADC1->CR & ADC_CR_ADSTART) == 1);
   tim2_init();
}

int main(int argc, char* argv[])
{
  // Send a greeting to the trace device (skipped on Release).
  trace_puts("Hello ARM World!");

  // At this stage the system clock should have already been configured
  // at high speed.
  trace_printf("System clock: %u Hz\n", SystemCoreClock);
  //ADC calibration
  ADC1->CR &= ~ADC_CR_ADEN;
  ADC1->CR |= ADC_CR_ADCAL;
  while((ADC1->CR & ADC_CR_ADCAL) != 0) {}
  trace_printf("ADC calibrated.\n");
  //ESP8266_ConnectToServer("54.191.41.90", "1234");

  //  USART2 enable PA2-TX, PA3-RX
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);
  USART_Cmd(USART2,ENABLE);

  //ESP8266_Init();
  //ESP8266_JoinAP("TP-LINK_46D994", "8446D994");

  //ESP8266_ConnectToServer("128.46.4.88", "34343");
  //ESP8266_ConnectToServer("54.191.41.90", "80");
  ADC_init();


  // Infinite loop
  while (1) {
  }
  // Infinite loop, never return.
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

