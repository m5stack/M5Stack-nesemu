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


#define DATA_LENGTH                        512              /*!<Data buffer length for test buffer*/
#define RW_TEST_LENGTH                     129              /*!<Data length for r/w test, any value from 0-DATA_LENGTH*/
#define DELAY_TIME_BETWEEN_ITEMS_MS        1234             /*!< delay time between different test items */

#define I2C_KEYBOARD_SCL_IO          19               /*!< gpio number for I2C master clock */
#define I2C_KEYBOARD_SDA_IO          18               /*!< gpio number for I2C master data  */
#define I2C_KEYBOARD_NUM             I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_KEYBOARD_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_KEYBOARD_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_KEYBOARD_FREQ_HZ         100000           /*!< I2C master clock frequency */
#define I2C_KEYBOARD_ADDR            0x88

#define ESP_SLAVE_ADDR                     0x28             /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */

// SemaphoreHandle_t print_mux = NULL;

void i2c_keyboard_master_init()
{
    int i2c_master_port = I2C_KEYBOARD_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_21;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = GPIO_NUM_22;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_KEYBOARD_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_KEYBOARD_RX_BUF_DISABLE,
                       I2C_KEYBOARD_TX_BUF_DISABLE, 0);
}

/**
 * @brief test code to write esp-i2c-slave
 *
 * 1. set mode
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|------|
 * 2. wait more than 24 ms
 * 3. read data
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */
uint8_t i2c_keyboard_read()
{
    uint8_t ret;
    // i2c_port_t i2c_num = I2C_KEYBOARD_NUM;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (uint8_t)(I2C_KEYBOARD_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &ret, NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_KEYBOARD_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

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