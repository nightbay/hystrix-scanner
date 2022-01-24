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
#include <string.h>
#include <linux/types.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <string.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include "hystrix-scanner.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"
#include "sgp40_i2c.h"

/* gpio code 0-2 */
int mux_code[] = {
    21,
    20,
    18};

/* gpio sel_sens 0-2 */
int mux_sel_sens[] = {
    16,
    17,
    19};

static const char *adc_device0 = "/dev/spidev1.0";
static const char *adc_device1 = "/dev/spidev1.1";
static const char *voc_devices = "/dev/i2c-2";

static const char *sht21_umidity_entry = "/sys/class/hwmon/hwmon1/humidity1_input";
static const char *sht21_temperature_entry = "/sys/class/hwmon/hwmon1/temp1_input";

uint32_t mode = 0;
uint8_t bits = 32;
uint32_t speed = 2000000;
uint16_t delay;

struct gpiod_chip *gpio_chip;

struct gpiod_line_bulk gpio_code = GPIOD_LINE_BULK_INITIALIZER;
struct gpiod_line_bulk gpio_sel_sens = GPIOD_LINE_BULK_INITIALIZER;
int main(int argc, char *argv[])
{
    struct sockaddr_un server;
    int sock;
    int sel_sens[3], code[3];
    int exitcode = EXIT_SUCCESS;

    int chan_input = atoi(argv[2]);
    printf("chan_input %d\n\r", chan_input);

    /* setup: configure gpios */
    gpio_chip = gpiod_chip_open_by_number(GPIO_BANK);

    gpiod_line_bulk_init(&gpio_code);
    gpiod_line_bulk_init(&gpio_sel_sens);

    /* open the GPIO lines */
    uint8_t i = 0;
    for (i = 0; i < 3; i++)
    {
        /* setup CODE gpio (input) */
        struct gpiod_line *gpio_line = gpiod_chip_get_line(gpio_chip, mux_code[i]);
        gpiod_line_bulk_add(&gpio_code, gpio_line);

        /* setup SEL_SENS gpio (output) */
        gpio_line = gpiod_chip_get_line(gpio_chip, mux_sel_sens[i]);

        gpiod_line_bulk_add(&gpio_sel_sens, gpio_line);
    }

    int ret = gpiod_line_request_bulk_output(&gpio_sel_sens,
                                             "sel_sens", sel_sens);

    if (ret == -1)
    {
        printf("unable to request sel_sens gpios\n\r");
        exitcode = EXIT_FAILURE;
        ;
        goto bailout;
    }

    ret = gpiod_line_request_bulk_input(&gpio_code,
                                        "code");

    if (ret == -1)
    {
        printf("unable to request code gpios\n\r");
        exitcode = EXIT_FAILURE;
        ;
        goto bailout;
    }

    /* setup VOC hal i2c */
    sensirion_i2c_hal_init();

    /* iterate sensors on spi/i2c */

    JsonNode *sensor_readings = json_mkarray();

    uint8_t channel;
    uint8_t channel_code;

    for (channel = 0; channel < 8; channel++)
    {
        //if( channel != chan_input )
        //	continue;

        channel_code = mux_channel(channel);

        if (channel_code == 0xff)
        {
            printf("error in mux_channel %d, bailing out\n\r", channel);
            exitcode = EXIT_FAILURE;
            goto bailout;
        }

        uint8_t channels_type = channel & VOC_CHANNELS;
        JsonNode *json_result;

        switch (channels_type)
        {
        case VOC_CHANNELS:
            json_result = read_voc(channel, channel_code);
            break;
        case EC_CHANNELS:
            json_result = read_electrochemical(channel, channel_code);
            break;
        }

        json_append_element(sensor_readings, json_result);
    }

    JsonNode *ipc_message = json_mkobject();
    json_append_member(ipc_message, "type", json_mkstring("message"));
    json_append_member(ipc_message, "data", sensor_readings);

    char *encoded = json_encode(ipc_message);
    printf(encoded);

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

    close(sock);

bailout:
    exit(exitcode);
}

#define my_adc_chan0 0xa0000000
#define my_adc_chan1 0xa1000000

/* VOC (Volatile Organic Compounds)  sensors on spi */
JsonNode *read_voc(uint8_t channel, uint8_t channels_type)
{
    JsonNode *result = json_mkobject();
    int16_t error = 0;

    char *humidity = sht21_read_sysfs(sht21_umidity_entry);
    char *temperature = sht21_read_sysfs(sht21_temperature_entry);

    // Start Measurement

    // Parameters for deactivated humidity compensation:
    uint16_t default_rh = 0x8000;
    uint16_t default_t = 0x6666;

    uint16_t sraw_voc;

    sensirion_i2c_hal_sleep_usec(1000000);

    error = sgp40_measure_raw_signal(default_rh, default_t, &sraw_voc);
    if (error) {
        JsonNode *json_value = json_mknumber(0);
        JsonNode *json_code = json_mknumber(channels_type);
        json_append_member(result, "chan", json_mknumber(channel));
        json_append_member(result, "type", json_mkstring("VOC"));
        json_append_member(result, "code", json_code);
        json_append_member(result, "val", json_value);
        json_append_member(result, "temp", json_mkstring(temperature));
        json_append_member(result, "hum", json_mkstring(humidity));
        json_append_member(result, "status", json_mkbool(false));                
    } else {
        JsonNode *json_value = json_mknumber(sraw_voc);
        JsonNode *json_code = json_mknumber(channels_type);
        json_append_member(result, "chan", json_mknumber(channel));
        json_append_member(result, "type", json_mkstring("VOC"));
        json_append_member(result, "code", json_code);
        json_append_member(result, "val", json_value);
        json_append_member(result, "temp", json_mkstring(temperature));
        json_append_member(result, "hum", json_mkstring(humidity));
        json_append_member(result, "status", json_mkbool(true));         
    }

bailout:
    return result;
}

/* Electrochemical sensors on i2c */
JsonNode *read_electrochemical(uint8_t channel, uint8_t channels_type)
{
    int ret;
    char *device;
    char *encoded;
    int adc_chan;
    adc_data_t inbuf;
    uint32_t buf[1];
    uint32_t txBuf[1];

    JsonNode *result = json_mkobject();

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

    char *humidity = sht21_read_sysfs(sht21_umidity_entry);
    char *temperature = sht21_read_sysfs(sht21_temperature_entry);

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

    if (ret < 0)
    {
        JsonNode *json_value = json_mknumber(0);
        JsonNode *json_code = json_mknumber(channels_type);
        json_append_member(result, "chan", json_mknumber(channel));
        json_append_member(result, "type", json_mkstring("EC"));
        json_append_member(result, "code", json_code);
        json_append_member(result, "val", json_value);
        json_append_member(result, "temp", json_mkstring(temperature));
        json_append_member(result, "hum", json_mkstring(humidity));
        json_append_member(result, "status", json_mkbool(false));
    }
    else
    {
        inbuf.dbg[0] = buf[0];
        JsonNode *json_value = json_mknumber(inbuf.ch[adc_chan]);
        JsonNode *json_code = json_mknumber(channels_type);
        json_append_member(result, "chan", json_mknumber(channel));
        json_append_member(result, "type", json_mkstring("EC"));
        json_append_member(result, "code", json_code);
        json_append_member(result, "val", json_value);
        json_append_member(result, "temp", json_mkstring(temperature));
        json_append_member(result, "hum", json_mkstring(humidity));
        json_append_member(result, "status", json_mkbool(true));
    }

bailout1:
    close(fd);
bailout2:
    return result;
}

JsonNode *read_rs232(uint8_t channel, uint8_t channels_type)
{
}

char *sht21_read_sysfs(char *filename)
{
    static char buf[1024];
    ssize_t len;
    int fd;

    fd = open(filename, O_RDONLY);
    if (fd < 0)
    {
        sprintf(buf, "%d", 0);
    }

    len = read(fd, buf, sizeof(buf) - 1);
    if (len < 0)
    {
        sprintf(buf, "%d", 0);
    }

    buf[len] = 0;

    return buf;
}

uint8_t mux_channel(uint8_t channel)
{
    uint8_t result = 0;
    int sel_sens[3], code[3], i;

    for (i = 0; i < 3; i++)
    {
        sel_sens[i] = channel & (1U << i) ? 1 : 0;
    }

    printf("muxing %d to %x%x%x\n\r", channel, sel_sens[2], sel_sens[1], sel_sens[0]);

    int ret = gpiod_line_set_value_bulk(&gpio_sel_sens,
                                        sel_sens);

    if (ret == -1)
    {
        printf("unable to set sel_sens gpios\n\r");
        result = 0xff;
        goto bailout;
    }

    ret = gpiod_line_get_value_bulk(&gpio_code, code);

    if (ret == -1)
    {
        printf("unable to get code gpios\n\r");
        result = 0xff;
        goto bailout;
    }

    for (i = 0; i < 3; i++)
    {
        result |= code[i] << i;
    }

bailout:
    return result;
}

double SHT21_CalcRH(uint16_t rh)
{

    rh &= ~0x0003; // clean last two bits

    return (-6.0 + 125.0 / 65536 * (double)rh); // return relative humidity
}

double SHT21_CalcT(uint16_t t)
{

    t &= ~0x0003; // clean last two bits

    return (-46.85 + 175.72 / 65536 * (double)t);
}
