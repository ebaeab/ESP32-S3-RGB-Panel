/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "nv3052c.h"



static const char *TAG = "nv3052c";

#if  0
    spi_device_handle_t spi;
    spi_bus_config_t buscfg={
        .miso_io_num = GPIO_NUM_NC,
        .mosi_io_num = GPIO_LCD_SDA,
        .sclk_io_num = GPIO_LCD_SCK,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = 1024,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg={
        .spics_io_num = GPIO_LCD_CS,               //CS pin        
        .clock_speed_hz = 10*1000*1000,           //Clock out at 10 MHz
        .mode = 0,                                //SPI mode 0
        .command_bits = 8,           //< Default amount of bits in command phase (0-16), used when ``SPI_TRANS_VARIABLE_CMD`` is not used, otherwise ignored.
        .address_bits = 8,  
        .queue_size = 10,            //We want to be able to queue 7 transactions at a time
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

    // esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle);
    // esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle);
#endif
spi_device_handle_t spi;
static void spi_soft_init(void)
{
#if 0
    gpio_pad_select_gpio(GPIO_LCD_CS);
	gpio_set_direction(GPIO_LCD_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_LCD_CS, 1);   

	gpio_pad_select_gpio(GPIO_LCD_SCK);
	gpio_set_direction(GPIO_LCD_SCK, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_LCD_SCK, 1);  

	gpio_pad_select_gpio(GPIO_LCD_SDA);
	gpio_set_direction(GPIO_LCD_SDA, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_LCD_SDA, 1);  
#else
    gpio_config_t io_conf1 = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << GPIO_LCD_CS, 
    };
    gpio_config(&io_conf1);
    gpio_set_level(GPIO_LCD_CS, 1);

    gpio_config_t io_conf2 = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << GPIO_LCD_SCK, 
    };
    gpio_config(&io_conf2);
    gpio_set_level(GPIO_LCD_SCK, 1);

    gpio_config_t io_conf3 = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << GPIO_LCD_SDA,
    };
    gpio_config(&io_conf3);
    gpio_set_level(GPIO_LCD_SDA, 1);
#endif
#if 0
 
    spi_bus_config_t buscfg={
        .miso_io_num = GPIO_NUM_NC,
        .mosi_io_num = GPIO_LCD_SDA,
        .sclk_io_num = GPIO_LCD_SCK,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = 4096,
        .flags = SPICOMMON_BUSFLAG_DUAL,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg={
        .spics_io_num = GPIO_LCD_CS,               //CS pin        
        .clock_speed_hz = 10 * 1000 * 1000,           //Clock out at 10 MHz
        .mode = 0,                                //SPI mode 0
        .command_bits = 1,           //< Default amount of bits in command phase (0-16), used when ``SPI_TRANS_VARIABLE_CMD`` is not used, otherwise ignored. 
        .queue_size = 7,            //We want to be able to queue 7 transactions at a time
        .flags = SPI_DEVICE_3WIRE | SPI_DEVICE_HALFDUPLEX,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));
#endif
}

static void spi_soft_write_9bits(uint16_t data)
{
	uint8_t i;
	LCD_CS_Clr();
	for(i = 0; i < 9; i++)
	{
        if(data & 0x100)   LCD_SDA_Set();
		else               LCD_SDA_Clr();
        LCD_SCK_Clr();
        LCD_SCK_Set();
		data <<= 1;
	}
	LCD_CS_Set();
}
#if 0
static void nv3052c_write_only_cmd(uint8_t cmd)
{
	uint16_t temp = 0;
	temp = temp | cmd;
    LCD_CS_Clr();
	spi_soft_write_9bits(temp);
    LCD_CS_Set();
}
#endif
static void nv3052c_write_only_cmd(uint8_t cmd)
{
    spi_transaction_t sendcfg;
 
    //发送cmd
    memset(&sendcfg, 0, sizeof(sendcfg));  
    sendcfg.cmd = 0;                        //cmd = 0
    sendcfg.length=8;                      
    sendcfg.tx_buffer = &cmd;
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &sendcfg));    
}

static void nv3052c_write_cmd(uint8_t cmd)
{
	uint16_t temp = 0;
	temp = temp | cmd;
	spi_soft_write_9bits(temp);
}

static void nv3052c_write_data(uint8_t data)
{
	uint16_t temp = 0x100;
	temp = temp | data;
	spi_soft_write_9bits(temp);
}

static void spi_write_reg(uint8_t addr,uint8_t data)
{
    //LCD_CS_Clr();
    nv3052c_write_cmd(addr);
    nv3052c_write_data(data);
    //LCD_CS_Set();
}

#if 0
static void spi_write_reg(const uint8_t cmd, const uint8_t data)
{
    spi_transaction_t sendcfg;
 
    //发送cmd
    memset(&sendcfg, 0, sizeof(sendcfg));  
    sendcfg.cmd = 0;                        //cmd = 0
    sendcfg.length=8;                      
    sendcfg.tx_buffer = &cmd;
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &sendcfg));    
                
 
    //发送data
    memset(&sendcfg, 0, sizeof(sendcfg));  
    sendcfg.cmd = 1;                        //data = 1
    sendcfg.length=8;                      
    sendcfg.tx_buffer = &data;
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &sendcfg));    

}
#endif

void nv3052c_reg_init(void)
{
    spi_soft_init();   //GPIO init
    ESP_LOGI(TAG, "nv3052c register init");

    gpio_config_t io_conf1 = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << GPIO_LCD_RST, 
    };
    gpio_config(&io_conf1);
    gpio_set_level(GPIO_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(GPIO_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(GPIO_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120)); 

    spi_write_reg(0xFF,0x30);
    spi_write_reg(0xFF,0x52);
    spi_write_reg(0xFF,0x01);  
    spi_write_reg(0xE3,0x00);  
    spi_write_reg(0x40,0x00);
    spi_write_reg(0x03,0x40);
    spi_write_reg(0x04,0x00);
    spi_write_reg(0x05,0x03);
    spi_write_reg(0x08,0x00);
    spi_write_reg(0x09,0x07);
    spi_write_reg(0x0A,0x01);
    spi_write_reg(0x0B,0x32);
    spi_write_reg(0x0C,0x32);
    spi_write_reg(0x0D,0x0B);
    spi_write_reg(0x0E,0x00);
    //spi_write_reg(0x23,0xA2);
    spi_write_reg(0x23,0x14);

    spi_write_reg(0x24,0x0c);
    spi_write_reg(0x25,0x06);
    spi_write_reg(0x26,0x14);
    spi_write_reg(0x27,0x14);

    spi_write_reg(0x38,0x9C); 
    spi_write_reg(0x39,0xA7); 
    spi_write_reg(0x3A,0x47); 

    spi_write_reg(0x28,0x40);
    spi_write_reg(0x29,0x01);
    spi_write_reg(0x2A,0xdf);
    spi_write_reg(0x49,0x3C);   
    spi_write_reg(0x91,0x57); 
    spi_write_reg(0x92,0x57); 

    spi_write_reg(0x99,0x5F); 

    spi_write_reg(0xA0,0x55);
    spi_write_reg(0xA1,0x50);
    spi_write_reg(0xA4,0x9C);
    spi_write_reg(0xA7,0x02);  
    spi_write_reg(0xA8,0x01);  
    spi_write_reg(0xA9,0x01);  
    spi_write_reg(0xAA,0xFC);  
    spi_write_reg(0xAB,0x28);  
    spi_write_reg(0xAC,0x06);  
    spi_write_reg(0xAD,0x06);  
    spi_write_reg(0xAE,0x06);  
    spi_write_reg(0xAF,0x03);  
    spi_write_reg(0xB0,0x08);  
    spi_write_reg(0xB1,0x26);  
    spi_write_reg(0xB2,0x28);  
    spi_write_reg(0xB3,0x28);  
    spi_write_reg(0xB4,0x03);  
    spi_write_reg(0xB5,0x08);  
    spi_write_reg(0xB6,0x26);  
    spi_write_reg(0xB7,0x08);  
    spi_write_reg(0xB8,0x26); 
    spi_write_reg(0xF0,0x00); 
    spi_write_reg(0xF6,0xC0);

    spi_write_reg(0xFF,0x30);
    spi_write_reg(0xFF,0x52);
    spi_write_reg(0xFF,0x02);
    spi_write_reg(0xB0,0x0B);
    spi_write_reg(0xB1,0x16);
    spi_write_reg(0xB2,0x17); 
    spi_write_reg(0xB3,0x2C); 
    spi_write_reg(0xB4,0x32);  
    spi_write_reg(0xB5,0x3B);  
    spi_write_reg(0xB6,0x29); 
    spi_write_reg(0xB7,0x40);   
    spi_write_reg(0xB8,0x0d);
    spi_write_reg(0xB9,0x05);
    spi_write_reg(0xBA,0x12);
    spi_write_reg(0xBB,0x10);
    spi_write_reg(0xBC,0x12);
    spi_write_reg(0xBD,0x15);
    spi_write_reg(0xBE,0x19);              
    spi_write_reg(0xBF,0x0E);
    spi_write_reg(0xC0,0x16);  
    spi_write_reg(0xC1,0x0A);
    spi_write_reg(0xD0,0x0C);
    spi_write_reg(0xD1,0x17);
    spi_write_reg(0xD2,0x14);
    spi_write_reg(0xD3,0x2E);   
    spi_write_reg(0xD4,0x32);   
    spi_write_reg(0xD5,0x3C);  
    spi_write_reg(0xD6,0x22);
    spi_write_reg(0xD7,0x3D);
    spi_write_reg(0xD8,0x0D);
    spi_write_reg(0xD9,0x07);
    spi_write_reg(0xDA,0x13);
    spi_write_reg(0xDB,0x13);
    spi_write_reg(0xDC,0x11);
    spi_write_reg(0xDD,0x15);
    spi_write_reg(0xDE,0x19);                   
    spi_write_reg(0xDF,0x10);
    spi_write_reg(0xE0,0x17);    
    spi_write_reg(0xE1,0x0A);
    spi_write_reg(0xFF,0x30);
    spi_write_reg(0xFF,0x52);
    spi_write_reg(0xFF,0x03);   
    spi_write_reg(0x00,0x2A);
    spi_write_reg(0x01,0x2A);
    spi_write_reg(0x02,0x2A);
    spi_write_reg(0x03,0x2A);
    spi_write_reg(0x04,0x61);  
    spi_write_reg(0x05,0x80);   
    spi_write_reg(0x06,0xc7);   
    spi_write_reg(0x07,0x01);  
    spi_write_reg(0x08,0x03); 
    spi_write_reg(0x09,0x04);
    spi_write_reg(0x70,0x22);
    spi_write_reg(0x71,0x80);
    spi_write_reg(0x30,0x2A);
    spi_write_reg(0x31,0x2A);
    spi_write_reg(0x32,0x2A);
    spi_write_reg(0x33,0x2A);
    spi_write_reg(0x34,0x61);
    spi_write_reg(0x35,0xc5);
    spi_write_reg(0x36,0x80);
    spi_write_reg(0x37,0x23);
    spi_write_reg(0x40,0x03); 
    spi_write_reg(0x41,0x04); 
    spi_write_reg(0x42,0x05); 
    spi_write_reg(0x43,0x06); 
    spi_write_reg(0x44,0x11); 
    spi_write_reg(0x45,0xe8); 
    spi_write_reg(0x46,0xe9); 
    spi_write_reg(0x47,0x11);
    spi_write_reg(0x48,0xea); 
    spi_write_reg(0x49,0xeb);
    spi_write_reg(0x50,0x07); 
    spi_write_reg(0x51,0x08); 
    spi_write_reg(0x52,0x09); 
    spi_write_reg(0x53,0x0a); 
    spi_write_reg(0x54,0x11); 
    spi_write_reg(0x55,0xec); 
    spi_write_reg(0x56,0xed); 
    spi_write_reg(0x57,0x11); 
    spi_write_reg(0x58,0xef); 
    spi_write_reg(0x59,0xf0); 
    spi_write_reg(0xB1,0x01); 
    spi_write_reg(0xB4,0x15); 
    spi_write_reg(0xB5,0x16); 
    spi_write_reg(0xB6,0x09); 
    spi_write_reg(0xB7,0x0f); 
    spi_write_reg(0xB8,0x0d); 
    spi_write_reg(0xB9,0x0b); 
    spi_write_reg(0xBA,0x00); 
    spi_write_reg(0xC7,0x02); 
    spi_write_reg(0xCA,0x17); 
    spi_write_reg(0xCB,0x18); 
    spi_write_reg(0xCC,0x0a); 
    spi_write_reg(0xCD,0x10); 
    spi_write_reg(0xCE,0x0e); 
    spi_write_reg(0xCF,0x0c); 
    spi_write_reg(0xD0,0x00); 
    spi_write_reg(0x81,0x00); 
    spi_write_reg(0x84,0x15); 
    spi_write_reg(0x85,0x16); 
    spi_write_reg(0x86,0x10); 
    spi_write_reg(0x87,0x0a); 
    spi_write_reg(0x88,0x0c); 
    spi_write_reg(0x89,0x0e);
    spi_write_reg(0x8A,0x02); 
    spi_write_reg(0x97,0x00); 
    spi_write_reg(0x9A,0x17); 
    spi_write_reg(0x9B,0x18);
    spi_write_reg(0x9C,0x0f);
    spi_write_reg(0x9D,0x09); 
    spi_write_reg(0x9E,0x0b); 
    spi_write_reg(0x9F,0x0d); 
    spi_write_reg(0xA0,0x01); 
    spi_write_reg(0xFF,0x30);
    spi_write_reg(0xFF,0x52);
    spi_write_reg(0xFF,0x02);  
    spi_write_reg(0x01,0x01);
    spi_write_reg(0x02,0xDA);
    spi_write_reg(0x03,0xBA);
    spi_write_reg(0x04,0xA8);
    spi_write_reg(0x05,0x9A);
    spi_write_reg(0x06,0x70);
    spi_write_reg(0x07,0xFF);
    spi_write_reg(0x08,0x91);
    spi_write_reg(0x09,0x90);
    spi_write_reg(0x0A,0xFF);
    spi_write_reg(0x0B,0x8F);
    spi_write_reg(0x0C,0x60);
    spi_write_reg(0x0D,0x58);
    spi_write_reg(0x0E,0x48);
    spi_write_reg(0x0F,0x38);
    spi_write_reg(0x10,0x2B);
    spi_write_reg(0xFF,0x30);
    spi_write_reg(0xFF,0x52);
    spi_write_reg(0xFF,0x00);   
    spi_write_reg(0x3A,0x55);//0x0177:24bit 0x0166:18bit 0x0155:16bit
    spi_write_reg(0x36,0x0A);//0a 是正扫，09是反扫

    spi_write_reg(0x11,0x00);	
    vTaskDelay(pdMS_TO_TICKS(200));
    spi_write_reg(0x29,0x00);
    vTaskDelay(pdMS_TO_TICKS(200));

}




