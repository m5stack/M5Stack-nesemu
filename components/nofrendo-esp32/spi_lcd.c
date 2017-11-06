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

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "rom/ets_sys.h"
#include "rom/gpio.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/spi_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "spi_lcd.h"
#include "driver/spi_master.h"

#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_14
#define PIN_NUM_DC   GPIO_NUM_27
#define PIN_NUM_RST  GPIO_NUM_33
#define PIN_NUM_BCKL GPIO_NUM_32

#define SPI_NUM  0x3

#define LCD_TYPE_ILI 0
#define LCD_TYPE_ST 1

//----------
#define LCD_SEL_CMD()   gpio_set_level(GPIO_NUM_27, 0) // Low to send command 
#define LCD_SEL_DATA()  gpio_set_level(GPIO_NUM_27, 1) // High to send data
#define LCD_RST_SET()   gpio_set_level(GPIO_NUM_33, 1)
#define LCD_RST_CLR()   gpio_set_level(GPIO_NUM_33, 0)
#define LCD_BKG_ON()    gpio_set_level(GPIO_NUM_32, 1) // Backlight ON
#define LCD_BKG_OFF()   gpio_set_level(GPIO_NUM_32, 0) //Backlight OFF
//----------
/*
 The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

spi_device_handle_t spi;

DRAM_ATTR static const lcd_init_cmd_t ili_init_cmds[]={
    {0xEF, {0x03, 0x80, 0x02}, 3},
    {0xCF, {0x00, 0XC1, 0X30}, 3},
    {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
    {0xE8, {0x85, 0x00, 0x78}, 3},
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
    {0xF7, {0x20}, 1},
    {0xEA, {0x00, 0x00}, 2},
    {0xC0, {0x23}, 1},
    {0xC1, {0x10}, 1},
    {0xC5, {0x3e, 0x28}, 2},
    {0xC7, {0x86}, 1},
    // {0x36, {0x48}, 1},
    {0x36, {0x08}, 1},
    {0x3A, {0x55}, 1},
    // {0xB1, {0x00, 0x18}, 2},
    {0xB1, {0x00, 0x1B}, 2},
    {0xB6, {0x08, 0x82, 0x27}, 3},
    {0xF2, {0x00}, 1},
    {0x26, {0x01}, 1},
    {0xE0, {0x0F,0x31,0x2B,0x0C,0x0E,0x08,0x4E,0xF1,0x37,0x07,0x10,0x03,0x0E,0x09,0x00}, 15},
    {0XE1, {0x00,0x0E,0x14,0x03,0x11,0x07,0x31,0xC1,0x48,0x08,0x0F,0x0C,0x31,0x36,0x0F}, 15},

    // {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
    // {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4}, 
    // {0x2C, {0}, 0},
    // {0xB7, {0x07}, 1},
    {0x11, {0}, 0x80},
    {0x29, {0}, 0x80},
    {0, {0}, 0xff},
};

//Send a command to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd) 
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//Send data to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len) 
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}
//------

//Initialize the display
void lcd_init(spi_device_handle_t spi) 
{
    int cmd=0;
    const lcd_init_cmd_t* lcd_init_cmds;

    //Initialize non-SPI GPIOs
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    //Reset the display
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);

    //detect LCD type
    // uint32_t lcd_id = lcd_get_id(spi);
    uint32_t lcd_id = 0;
    int lcd_detected_type = 0;
    int lcd_type;

    printf("LCD ID: %08X\n", lcd_id);
    lcd_detected_type = LCD_TYPE_ILI;
    printf("ILI9341 detected...\n");   

    lcd_type = lcd_detected_type; 
    lcd_init_cmds = ili_init_cmds;

    //Send all the commands
    while (lcd_init_cmds[cmd].databytes!=0xff) {
        lcd_cmd(spi, lcd_init_cmds[cmd].cmd);
        lcd_data(spi, lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes&0x1F);
        if (lcd_init_cmds[cmd].databytes&0x80) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }

    ///Enable backlight
    gpio_set_level(PIN_NUM_BCKL, 1);
}

void lcd_spi_pre_transfer_callback(spi_transaction_t *t) 
{
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

void ili9341_spi_init()
{
    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    spi_device_interface_config_t devcfg={
        // .clock_speed_hz=10*1000*1000,               //Clock out at 10 MHz
        .clock_speed_hz=32*1000*1000,               //Clock out at 10 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(VSPI_HOST, &buscfg, 1);
    // assert(ret==ESP_OK);

    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
    // assert(ret==ESP_OK);

    //Initialize the LCD
    lcd_init(spi);
}
//------


static void spi_write_byte(const uint8_t data) {
    SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 0x7, SPI_USR_MOSI_DBITLEN_S);
    WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), data);
    SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
    while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
}

static void LCD_WriteCommand(const uint8_t cmd)
{
    LCD_SEL_CMD();
    spi_write_byte(cmd);
}

static void LCD_WriteData(const uint8_t data)
{
    LCD_SEL_DATA();
    spi_write_byte(data);
}

static void  ILI9341_INITIAL ()
{
    LCD_BKG_ON();
    //------------------------------------Reset Sequence-----------------------------------------//

    LCD_RST_SET();
    ets_delay_us(100000);                                                              

    LCD_RST_CLR();
    ets_delay_us(200000);                                                              

    LCD_RST_SET();
    ets_delay_us(200000);                                                             

    //************* Start Initial Sequence **********//
    LCD_WriteCommand(0xCF);
    LCD_WriteData(0x00);
    LCD_WriteData(0xC1);
    LCD_WriteData(0X30);

    LCD_WriteCommand(0xED);
    LCD_WriteData(0x64);
    LCD_WriteData(0x03);
    LCD_WriteData(0X12);
    LCD_WriteData(0X81);

    LCD_WriteCommand(0xE8);
    LCD_WriteData(0x85);
    LCD_WriteData(0x00); //i
    LCD_WriteData(0x78); //i

    LCD_WriteCommand(0xCB);
    LCD_WriteData(0x39);
    LCD_WriteData(0x2C);
    LCD_WriteData(0x00);
    LCD_WriteData(0x34);
    LCD_WriteData(0x02);

    LCD_WriteCommand(0xF7);
    LCD_WriteData(0x20);

    LCD_WriteCommand(0xEA);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);

    LCD_WriteCommand(0xC0);    //Power control
    LCD_WriteData(0x26); //i  //VRH[5:0]

    LCD_WriteCommand(0xC1);    //Power control
    LCD_WriteData(0x11);   //i //SAP[2:0];BT[3:0]

    LCD_WriteCommand(0xC5);    //VCM control
    LCD_WriteData(0x35); //i
    LCD_WriteData(0x3E); //i

    LCD_WriteCommand(0xC7);    //VCM control2
    LCD_WriteData(0xBE); //i   //»òÕß B1h

    LCD_WriteCommand(0x36);    // Memory Access Control
    LCD_WriteData(0x08); //i //was 0x48
    // LCD_WriteData(0x28); //i //was 0x48

    LCD_WriteCommand(0x3A);
    LCD_WriteData(0x55);

    LCD_WriteCommand(0xB1);
    LCD_WriteData(0x00);
    LCD_WriteData(0x1B); //18
    
    LCD_WriteCommand(0xF2);    // 3Gamma Function Disable
    LCD_WriteData(0x08);

    LCD_WriteCommand(0x26);    //Gamma curve selected
    LCD_WriteData(0x01);
        
    LCD_WriteCommand(0xE0);    //Set Gamma
    LCD_WriteData(0x1F);
    LCD_WriteData(0x1A);
    LCD_WriteData(0x18);
    LCD_WriteData(0x0A);
    LCD_WriteData(0x0F);
    LCD_WriteData(0x06);
    LCD_WriteData(0x45);
    LCD_WriteData(0X87);
    LCD_WriteData(0x32);
    LCD_WriteData(0x0A);
    LCD_WriteData(0x07);
    LCD_WriteData(0x02);
    LCD_WriteData(0x07);
    LCD_WriteData(0x05);
    LCD_WriteData(0x00);
 
    LCD_WriteCommand(0XE1);    //Set Gamma
    LCD_WriteData(0x00);
    LCD_WriteData(0x25);
    LCD_WriteData(0x27);
    LCD_WriteData(0x05);
    LCD_WriteData(0x10);
    LCD_WriteData(0x09);
    LCD_WriteData(0x3A);
    LCD_WriteData(0x78);
    LCD_WriteData(0x4D);
    LCD_WriteData(0x05);
    LCD_WriteData(0x18);
    LCD_WriteData(0x0D);
    LCD_WriteData(0x38);
    LCD_WriteData(0x3A);
    LCD_WriteData(0x1F);

    LCD_WriteCommand(0x2A);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);
    LCD_WriteData(0xEF);

    LCD_WriteCommand(0x2B);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);
    LCD_WriteData(0x01);
    LCD_WriteData(0x3f);
    LCD_WriteCommand(0x2C);
    
    LCD_WriteCommand(0xB7); 
    LCD_WriteData(0x07); 
    
    LCD_WriteCommand(0xB6);    // Display Function Control
    LCD_WriteData(0x0A); //8 82 27
    LCD_WriteData(0x82);
    LCD_WriteData(0x27);
    LCD_WriteData(0x00);

    //LCD_WriteCommand(0xF6); //not there
    //LCD_WriteData(0x01);
    //LCD_WriteData(0x30);

    LCD_WriteCommand(0x11);    //Exit Sleep
    ets_delay_us(100000);
    LCD_WriteCommand(0x29);    //Display on
    ets_delay_us(100000);


}
//.............LCD API END----------

static void ili_gpio_init()
{
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
}

static void spi_master_init()
{
    periph_module_enable(PERIPH_VSPI_MODULE);
    periph_module_enable(PERIPH_SPI_DMA_MODULE);

    ets_printf("lcd spi pin mux init ...\r\n");
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_NUM_MISO], 2);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_NUM_MOSI], 2);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_NUM_CLK], 2);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_NUM_CS], 2);
    WRITE_PERI_REG(GPIO_ENABLE_W1TS_REG, BIT(PIN_NUM_MOSI)|BIT(PIN_NUM_CLK)|BIT(PIN_NUM_CS));

    ets_printf("lcd spi signal init\r\n");
    gpio_matrix_in(PIN_NUM_MISO, VSPIQ_IN_IDX,0);
    gpio_matrix_out(PIN_NUM_MOSI, VSPID_OUT_IDX,0,0);
    gpio_matrix_out(PIN_NUM_CLK, VSPICLK_OUT_IDX,0,0);
    gpio_matrix_out(PIN_NUM_CS, VSPICS0_OUT_IDX,0,0);
    ets_printf("Hspi config\r\n");

    CLEAR_PERI_REG_MASK(SPI_SLAVE_REG(SPI_NUM), SPI_TRANS_DONE << 5);
    SET_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_CS_SETUP);
    CLEAR_PERI_REG_MASK(SPI_PIN_REG(SPI_NUM), SPI_CK_IDLE_EDGE);
    CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM),  SPI_CK_OUT_EDGE);
    CLEAR_PERI_REG_MASK(SPI_CTRL_REG(SPI_NUM), SPI_WR_BIT_ORDER);
    CLEAR_PERI_REG_MASK(SPI_CTRL_REG(SPI_NUM), SPI_RD_BIT_ORDER);
    CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_DOUTDIN);
    WRITE_PERI_REG(SPI_USER1_REG(SPI_NUM), 0);
    SET_PERI_REG_BITS(SPI_CTRL2_REG(SPI_NUM), SPI_MISO_DELAY_MODE, 0, SPI_MISO_DELAY_MODE_S);
    CLEAR_PERI_REG_MASK(SPI_SLAVE_REG(SPI_NUM), SPI_SLAVE_MODE);
    
    WRITE_PERI_REG(SPI_CLOCK_REG(SPI_NUM), (1 << SPI_CLKCNT_N_S) | (1 << SPI_CLKCNT_L_S));//40MHz
    //WRITE_PERI_REG(SPI_CLOCK_REG(SPI_NUM), SPI_CLK_EQU_SYSCLK); // 80Mhz
    
    SET_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_CS_SETUP | SPI_CS_HOLD | SPI_USR_MOSI);
    SET_PERI_REG_MASK(SPI_CTRL2_REG(SPI_NUM), ((0x4 & SPI_MISO_DELAY_NUM) << SPI_MISO_DELAY_NUM_S));
    CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_COMMAND);
    SET_PERI_REG_BITS(SPI_USER2_REG(SPI_NUM), SPI_USR_COMMAND_BITLEN, 0, SPI_USR_COMMAND_BITLEN_S);
    CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_ADDR);
    SET_PERI_REG_BITS(SPI_USER1_REG(SPI_NUM), SPI_USR_ADDR_BITLEN, 0, SPI_USR_ADDR_BITLEN_S);
    CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_MISO);
    SET_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_MOSI);
    char i;
    for (i = 0; i < 16; ++i) {
        WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + (i << 2)), 0);
    }
}

#define U16x2toU32(m,l) ((((uint32_t)(l>>8|(l&0xFF)<<8))<<16)|(m>>8|(m&0xFF)<<8))

extern uint16_t myPalette[];

void ili9341_write_frame(const uint16_t xs, const uint16_t ys, const uint16_t width, const uint16_t height, const uint8_t * data[]){
    int x, y;
    int i;
    uint16_t x1, y1;
    uint32_t xv, yv, dc;
    uint32_t temp[16];
    dc = (1 << PIN_NUM_DC);
    
    for (y=0; y<height; y++) {
        //start line
        x1 = xs+(width-1);
        y1 = ys+y+(height-1);
        xv = U16x2toU32(xs,x1);
        yv = U16x2toU32((ys+y),y1);
        
        while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
        GPIO.out_w1tc = dc;
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 7, SPI_USR_MOSI_DBITLEN_S);
        WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), 0x2A);
        SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
        while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
        GPIO.out_w1ts = dc;
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 31, SPI_USR_MOSI_DBITLEN_S);
        WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), xv);
        SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
        while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
        GPIO.out_w1tc = dc;
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 7, SPI_USR_MOSI_DBITLEN_S);
        WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), 0x2B);
        SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
        while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
        GPIO.out_w1ts = dc;
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 31, SPI_USR_MOSI_DBITLEN_S);
        WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), yv);
        SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
        while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
        GPIO.out_w1tc = dc;
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 7, SPI_USR_MOSI_DBITLEN_S);
        WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), 0x2C);
        SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
        while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
        
        x = 0;
        GPIO.out_w1ts = dc;
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 511, SPI_USR_MOSI_DBITLEN_S);
        while (x<width) {
            for (i=0; i<16; i++) {
                if(data == NULL){
                    temp[i] = 0;
                    x += 2;
                    continue;
                }
                x1 = myPalette[(unsigned char)(data[y][x])]; x++;
                y1 = myPalette[(unsigned char)(data[y][x])]; x++;
                temp[i] = U16x2toU32(x1,y1);
            }
            while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
            for (i=0; i<16; i++) {
                WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + (i << 2)), temp[i]);
            }
            SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
        }
    }
    while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
}

void ili9341_init()
{
    // spi_master_init();
    // ili_gpio_init();
    ili9341_spi_init();
    // ILI9341_INITIAL ();
}





