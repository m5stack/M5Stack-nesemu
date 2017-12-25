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
#include "i2c_keyboard.h"

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

// #define KEY_A_PIN      35
// #define KEY_B_PIN      36
// #define KEY_UP_PIN     13
// #define KEY_DOWN_PIN   15
// #define KEY_LEFT_PIN   34
// #define KEY_RIGHT_PIN  17
// #define KEY_SELECT_PIN 16
// #define KEY_START_PIN  5

#define KEY_UP         0X01
#define KEY_DOWN       0X02
#define KEY_LEFT       0X04
#define KEY_RIGHT      0X08
#define KEY_A          0X10
#define KEY_B          0X20
#define KEY_SELECT     0X40
#define KEY_START      0X80


int psxReadInput() {
	static uint16_t pre_retval = 0;
	uint16_t retval = 0;
	uint8_t key_value = 0xff;

	if(gpio_get_level(5) == 0) {
		key_value = i2c_keyboard_read();
		printf("key is:0x%x \r\n", key_value);

		if((key_value & KEY_A) != 0) {
			retval |=  1<<bit_joypad1_a;
		} else {
			retval &= ~(1<<bit_joypad1_a);
		}

		if(key_value & KEY_B) {
			retval |=  1<<bit_joypad1_b;
		} else {
			retval &= ~(1<<bit_joypad1_b);
		}

		if(key_value & KEY_UP) {
			retval |=  1<<bit_joypad1_up;
		} else {
			retval &= ~(1<<bit_joypad1_up);
		}

		if(key_value & KEY_DOWN) {
			retval |=  1<<bit_joypad1_down;
		} else {
			retval &= ~(1<<bit_joypad1_down);
		}

		if(key_value & KEY_LEFT) {
			retval |=  1<<bit_joypad1_left;
		} else {
			retval &= ~(1<<bit_joypad1_left);
		}

		if(key_value & KEY_RIGHT) {
			retval |=  1<<bit_joypad1_right;
		} else {
			retval &= ~(1<<bit_joypad1_right);
		}

		if(key_value & KEY_SELECT) {
			retval |=  1<<bit_joypad1_select;
		} else {
			retval &= ~(1<<bit_joypad1_select);
		}

		if((key_value & KEY_START) != 0) {
			retval |=  1<<bit_joypad1_start;
		} else {
			retval &= ~(1<<bit_joypad1_start);
		}
	} else {
		retval = pre_retval;
	}

	pre_retval = retval;
	return (int)retval;
}


void psxcontrollerInit() {
	printf("PSX controller disabled in menuconfig; no input enabled.\n");
	
	gpio_set_direction(5, GPIO_MODE_INPUT);
	gpio_pullup_en(5);

	i2c_keyboard_master_init();
}

#endif