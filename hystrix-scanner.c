/*
  Copyright (C) 2021 Flavio Serreri (sviluppo@nuraxi.tech)
  All rights reserved.

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#include <stdio.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/types.h>

#include "hystrix-scanner.h"

/* gpio code 0-2 */
int  mux_code[] = {
    21,
    20,
    18
};

/* gpio sel_sens 0-2 */
int mux_sel_sens[] = {
    16,
    17,
    19
};

static const char *adc_device0 = "/dev/spidev1.0";
static const char *adc_device1 = "/dev/spidev1.1";

uint32_t mode;
uint8_t bits = 32;
uint32_t speed = 2000000;
uint16_t delay;

struct gpiod_chip *gpio_chip;

struct gpiod_line_bulk gpio_code = GPIOD_LINE_BULK_INITIALIZER;
struct gpiod_line_bulk gpio_sel_sens = GPIOD_LINE_BULK_INITIALIZER;
int main(int argc, char *argv[]){
    int sel_sens[3], code[3];
    int exitcode = EXIT_SUCCESS;
    /* setup: configure gpios */
    gpio_chip = gpiod_chip_open_by_number(GPIO_BANK);

    gpiod_line_bulk_init(&gpio_code);
    gpiod_line_bulk_init(&gpio_sel_sens);

	/* open the GPIO lines */
    uint8_t i = 0;
    for( i = 0; i < 3; i++){
        /* setup CODE gpio (input) */
        struct gpiod_line *gpio_line = gpiod_chip_get_line(gpio_chip, mux_code[i]);
        gpiod_line_bulk_add(&gpio_code, gpio_line);

        /* setup SEL_SENS gpio (output) */
        gpio_line = gpiod_chip_get_line(gpio_chip, mux_sel_sens[i]);

        gpiod_line_bulk_add(&gpio_sel_sens, gpio_line);

      
    }

    int ret = gpiod_line_request_bulk_output(&gpio_sel_sens,
                        "sel_sens", sel_sens);

    if( ret == -1 ){
        printf("unable to request sel_sens gpios\n\r");
        exitcode = EXIT_FAILURE;;
        goto bailout;
    }  

    ret = gpiod_line_request_bulk_input(&gpio_code,
                        "code");

    if( ret == -1 ){
        printf("unable to request code gpios\n\r");
        exitcode = EXIT_FAILURE;;
        goto bailout;
    }  
    /* iterate sensors on spi/i2c */

    JsonNode *sensor_readings = json_mkarray();

    uint8_t channel;
    uint8_t channel_code;

    for( channel = 0; channel < 8; channel++){
        channel_code = mux_channel(channel);

        if( channel_code == 0xff){
            printf("error in mux_channel %d, bailing out\n\r", channel);
            exitcode = EXIT_FAILURE;
            goto bailout;
        }

        uint8_t channels_type = channel & VOC_CHANNELS;
        JsonNode *json_result;

        switch( channels_type ){
            case VOC_CHANNELS:
                json_result = read_voc(channel, channel_code);                
                break;
            case EC_CHANNELS:
                json_result = read_electrochemical(channel, channel_code);
                break;                
        }

        json_append_element(sensor_readings, json_result);
    }

    printf(json_encode(sensor_readings));    
bailout:
    exit(exitcode);
}


/* electrochemical sensors on spi */
JsonNode* read_electrochemical(uint8_t channel, uint8_t channels_type){
    int ret;
    char* device;
    char *encoded;
    int adc_chan;
    uint8_t inbuf[4];
    
    JsonNode *result = json_mkobject();

    switch(channel){
        case 0:
        case 1:
            device = adc_device0;
            adc_chan = 0;
            break;
        case 2:
        case 3:
            device = adc_device1;
            adc_chan = 1;
            break;
    }

    int fd = open(device, O_RDWR);
    if (fd < 0)    
    {
        printf("can't open device\n\r");
        goto bailout2;
    }

    /*
	 * spi mode
	 */
	//ret = ioctl(fd, SPI_IOC_WR_MODE32, &mode);
	if (ret == -1)
    {
        printf("can't set spi mode\n\r");
        goto bailout1;
    }

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
    {
        printf("can't set bits per word\n\r");
        goto bailout1;
    }

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

	if (ret == -1)
    {
        printf("can't set max speed hz\n\r");
        goto bailout1;
    }

	struct spi_ioc_transfer tr = {
		.tx_buf = NULL,
		.rx_buf = (unsigned long long)inbuf,
		.len = 4,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

    if( ret < 0 )
    {
        printf("error during SPI read on %s\n", device);
        goto bailout1;
    }
    else
    {
        JsonNode *json_value = json_mknumber(inbuf[adc_chan]);
        JsonNode *json_code = json_mknumber(channels_type);
        json_append_member(result, "value", json_value);        
        json_append_member(result, "code", json_code);
    }

bailout1:
    close(fd);
bailout2:    
    return result;
}

/* VOC (Volatile Organic Compounds) sensors on i2c */

JsonNode* read_voc(uint8_t channel, uint8_t channels_type){
    JsonNode *result = json_mkobject();
    json_append_member(result, "random", json_mknumber(rand()));
    return result;
}

JsonNode* read_rs232(uint8_t channel, uint8_t channels_type){

}

uint8_t mux_channel(uint8_t channel){
    uint8_t result = 0;
    int sel_sens[3], code[3], i;

    for( i = 0; i < 3; i++){
        if( channel & (1 << i) != 0)
            sel_sens[i] = 1;
        else
            sel_sens[i] = 0;
    }

	int ret = gpiod_line_set_value_bulk(&gpio_sel_sens,
					      sel_sens);

    if( ret == -1 ){
        printf("unable to set sel_sens gpios\n\r");
        result = 0xff;
        goto bailout;
    }

    ret = gpiod_line_get_value_bulk(&gpio_code, code);

    if( ret == -1 ){
        printf("unable to get code gpios\n\r");
        result = 0xff;
        goto bailout;
    }

    for( i = 0; i < 3; i++){
        result |= code[i] << i;
    }

bailout:
    return result;
}