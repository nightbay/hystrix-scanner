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
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <linux/types.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <string.h>
#include "hystrix-ec.h"


static const char *adc_device0 = "/dev/spidev1.0";
static const char *adc_device1 = "/dev/spidev1.1";


uint32_t mode = 0;
uint8_t bits = 32;
uint32_t speed = 2000000;
uint16_t delay;

int main(int argc, char *argv[])
{
    struct sockaddr_un server;
    int sock;

    int exitcode = EXIT_SUCCESS;

    int chan_input = atoi(argv[2]);

    JsonNode *sensor_readings = NULL;

    uint8_t channel;
    uint8_t channel_code;

    sensor_readings = read_electrochemical(channel);
    if( sensor_readings == NULL ){
        exitcode = EXIT_FAILURE;
        goto bailout;
    }

    char *encoded = json_encode(sensor_readings);

    sock = socket(AF_UNIX, SOCK_STREAM, 0);

    if (sock < 0)
    {
        perror("opening stream socket");
        exitcode = EXIT_FAILURE;
        goto bailout;
    }

    server.sun_family = AF_UNIX;
    strcpy(server.sun_path, argv[1]);

    if (connect(sock, (struct sockaddr *)&server, sizeof(struct sockaddr_un)) < 0)
    {
        close(sock);
        perror("connecting stream socket");
        exitcode = EXIT_FAILURE;
        goto bailout;
    }
    if (write(sock, encoded, strlen(encoded)) < 0)
    {
        perror("writing on stream socket");
        exitcode = EXIT_FAILURE;
    }
    else{
        exitcode = EXIT_SUCCESS;
    }

    close(sock);

bailout:
    exit(exitcode);
}

#define my_adc_chan0 0xa0000000
#define my_adc_chan1 0xa1000000

/* Electrochemical sensors on i2c */
JsonNode *read_electrochemical(uint8_t channel)
{
    int ret;
    char *device;
    char *encoded;
    int adc_chan;
    adc_data_t inbuf;
    uint32_t buf[1];
    uint32_t txBuf[1];

    JsonNode *result = NULL;;

    switch (channel)
    {
    case 0:
        txBuf[0] = my_adc_chan0;
        device = adc_device0;
        break;
    case 1:
        txBuf[0] = my_adc_chan1;
        device = adc_device0;
        break;
    case 2:
        device = adc_device1;
        txBuf[0] = my_adc_chan0;
        break;
    case 3:
        device = adc_device1;
        txBuf[0] = my_adc_chan1;
        break;
    }

    adc_chan = channel & 1;
    int fd = open(device, O_RDWR);
    if (fd < 0)
    {
        printf("can't open device\n\r");
        ret = -1;
        goto bailout2;
    }

    /*
	 * spi mode
	 */
    ret = ioctl(fd, SPI_IOC_WR_MODE32, &mode);
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
        .tx_buf = (unsigned long)txBuf,
        .rx_buf = (unsigned long)buf,
        .len = 4,
        .delay_usecs = delay,
        .speed_hz = speed,
        .bits_per_word = bits,
    };



    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
bailout1:    
    close(fd);
bailout2:
    fd = 0; //solo per bailout;

    if (ret >= 0)
    {
        result = json_mkobject();
        inbuf.dbg[0] = buf[0];
        JsonNode *json_value = json_mknumber(inbuf.ch[adc_chan]);
        json_append_member(result, "chan", json_mknumber(channel));
        json_append_member(result, "val", json_value);
    }

    return result;
}


