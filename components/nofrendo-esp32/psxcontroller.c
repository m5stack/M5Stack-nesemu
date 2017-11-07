// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"


#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "psxcontroller.h"
#include "sdkconfig.h"

#define PSX_CLK CONFIG_HW_PSX_CLK
#define PSX_DAT CONFIG_HW_PSX_DAT
#define PSX_ATT CONFIG_HW_PSX_ATT
#define PSX_CMD CONFIG_HW_PSX_CMD

#define DELAY() asm("nop; nop; nop; nop;nop; nop; nop; nop;nop; nop; nop; nop;nop; nop; nop; nop;")


#if CONFIG_HW_PSX_ENA

/* Sends and receives a byte from/to the PSX controller using SPI */
static int psxSendRecv(int send) {
	int x;
	int ret=0;
	volatile int delay;
	
#if 0
	while(1) {
		GPIO.out_w1ts=(1<<PSX_CMD);
		GPIO.out_w1ts=(1<<PSX_CLK);
		GPIO.out_w1tc=(1<<PSX_CMD);
		GPIO.out_w1tc=(1<<PSX_CLK);
	}
#endif

	GPIO.out_w1tc=(1<<PSX_ATT);
	for (delay=0; delay<100; delay++);
	for (x=0; x<8; x++) {
		if (send&1) {
			GPIO.out_w1ts=(1<<PSX_CMD);
		} else {
			GPIO.out_w1tc=(1<<PSX_CMD);
		}
		DELAY();
		for (delay=0; delay<100; delay++);
		GPIO.out_w1tc=(1<<PSX_CLK);
		for (delay=0; delay<100; delay++);
		GPIO.out_w1ts=(1<<PSX_CLK);
		ret>>=1;
		send>>=1;
		if (GPIO.in&(1<<PSX_DAT)) ret|=128;
	}
	return ret;
}

static void psxDone() {
	DELAY();
	GPIO_REG_WRITE(GPIO_OUT_W1TS_REG, (1<<PSX_ATT));
}


int psxReadInput() {
	int b1, b2;

	psxSendRecv(0x01); //wake up
	psxSendRecv(0x42); //get data
	psxSendRecv(0xff); //should return 0x5a
	b1=psxSendRecv(0xff); //buttons byte 1
	b2=psxSendRecv(0xff); //buttons byte 2
	psxDone();
	return (b2<<8)|b1;

}


void psxcontrollerInit() {
	volatile int delay;
	int t;
	gpio_config_t gpioconf[2]={
		{
			.pin_bit_mask=(1<<PSX_CLK)|(1<<PSX_CMD)|(1<<PSX_ATT), 
			.mode=GPIO_MODE_OUTPUT, 
			.pull_up_en=GPIO_PULLUP_DISABLE, 
			.pull_down_en=GPIO_PULLDOWN_DISABLE, 
			.intr_type=GPIO_PIN_INTR_DISABLE
		},{
			.pin_bit_mask=(1<<PSX_DAT), 
			.mode=GPIO_MODE_INPUT, 
			.pull_up_en=GPIO_PULLUP_ENABLE, 
			.pull_down_en=GPIO_PULLDOWN_DISABLE, 
			.intr_type=GPIO_PIN_INTR_DISABLE
		}
	};
	gpio_config(&gpioconf[0]);
	gpio_config(&gpioconf[1]);
	
	//Send a few dummy bytes to clean the pipes.
	psxSendRecv(0);
	psxDone();
	for (delay=0; delay<500; delay++) DELAY();
	psxSendRecv(0);
	psxDone();
	for (delay=0; delay<500; delay++) DELAY();
	//Try and detect the type of controller, so we can give the user some diagnostics.
	psxSendRecv(0x01);
	t=psxSendRecv(0x00);
	psxDone();
	if (t==0 || t==0xff) {
		printf("No PSX/PS2 controller detected (0x%X). You will not be able to control the game.\n", t);
	} else {
		printf("PSX controller type 0x%X\n", t);
	}
}


#else

//返回值:
//[0]:右
//[1]:左
//[2]:下
//[3]:上
//[4]:Start
//[5]:Select
//[6]:B
//[7]:A

#define bit_joypad1_select 0
#define bit_joypad1_start  3
#define bit_joypad1_up     4
#define bit_joypad1_right  5
#define bit_joypad1_down   6
#define bit_joypad1_left   7
#define bit_soft_reset     12
#define bit_joypad1_a      13
#define bit_joypad1_b      14
#define bit_hard_reset     15

#define KEY_A_PIN      35
#define KEY_B_PIN      36
#define KEY_UP_PIN     13
#define KEY_DOWN_PIN   15
#define KEY_LEFT_PIN   34
#define KEY_RIGHT_PIN  17
#define KEY_SELECT_PIN 16
#define KEY_START_PIN  5

int psxReadInput() {
	uint16_t retval = 0;
	// static int pre_val;

	if(gpio_get_level(KEY_A_PIN)) {
		retval |=  1<<bit_joypad1_a;
	} else {
		retval &= ~(1<<bit_joypad1_a);
	}

	if(gpio_get_level(KEY_B_PIN)) {
		retval |=  1<<bit_joypad1_b;
	} else {
		retval &= ~(1<<bit_joypad1_b);
	}

	if(gpio_get_level(KEY_UP_PIN)) {
		retval |=  1<<bit_joypad1_up;
	} else {
		retval &= ~(1<<bit_joypad1_up);
	}

	if(gpio_get_level(KEY_DOWN_PIN)) {
		retval |=  1<<bit_joypad1_down;
	} else {
		retval &= ~(1<<bit_joypad1_down);
	}

	if(gpio_get_level(KEY_LEFT_PIN)) {
		retval |=  1<<bit_joypad1_left;
	} else {
		retval &= ~(1<<bit_joypad1_left);
	}

	if(gpio_get_level(KEY_RIGHT_PIN)) {
		retval |=  1<<bit_joypad1_right;
	} else {
		retval &= ~(1<<bit_joypad1_right);
	}

	if(gpio_get_level(KEY_SELECT_PIN)) {
		retval |=  1<<bit_joypad1_select;
	} else {
		retval &= ~(1<<bit_joypad1_select);
	}

	if(gpio_get_level(KEY_START_PIN)) {
		retval |=  1<<bit_joypad1_start;
	} else {
		retval &= ~(1<<bit_joypad1_start);
	}
	
	printf("press key: 0x%x \r\n", retval);
	// printf("press key: 0x%x \r\n", retval);
	return (int)retval;
}


void psxcontrollerInit() {
	printf("PSX controller disabled in menuconfig; no input enabled.\n");
	 
	gpio_set_direction(KEY_A_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(KEY_B_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(KEY_UP_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(KEY_DOWN_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(KEY_LEFT_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(KEY_RIGHT_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(KEY_SELECT_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(KEY_START_PIN, GPIO_MODE_INPUT);
	
	gpio_pullup_en(KEY_A_PIN);
	gpio_pullup_en(KEY_B_PIN);
	gpio_pullup_en(KEY_UP_PIN);
	gpio_pullup_en(KEY_DOWN_PIN);
	gpio_pullup_en(KEY_LEFT_PIN);
	gpio_pullup_en(KEY_RIGHT_PIN);
	gpio_pullup_en(KEY_SELECT_PIN);
	gpio_pullup_en(KEY_START_PIN);
}

#endif