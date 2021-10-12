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
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <string.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include "hystrix-scanner.h"

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
static const char *voc_devices = "/dev/i2c-0";

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

    /* iterate sensors on spi/i2c */

    JsonNode *sensor_readings = json_mkarray();

    uint8_t channel;
    uint8_t channel_code;

    for (channel = 0; channel < 8; channel++)
    {
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

    char *encoded = json_encode(sensor_readings);
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

/* VOC (Volatile Organic Compounds)  sensors on spi */
JsonNode *read_voc(uint8_t channel, uint8_t channels_type)
{
    int ret;
    char *device;
    char *encoded;
    int adc_chan;
    adc_data_t inbuf;
    uint32_t buf[1];

    JsonNode *result = json_mkobject();

    switch (channel)
    {
    case 4:
    case 5:
        device = adc_device0;
        break;
    case 6:
    case 7:
        device = adc_device1;
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
        .tx_buf = NULL,
        .rx_buf = (unsigned long)buf,
        .len = 4,
        .delay_usecs = delay,
        .speed_hz = speed,
        .bits_per_word = bits,
    };

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

    if (ret < 0)
    {
        printf("error during SPI read on %s\n", device);
        goto bailout1;
    }
    else
    {
        inbuf.dbg[0] = buf[0];
        JsonNode *json_value = json_mknumber(inbuf.ch[adc_chan]);
        JsonNode *json_code = json_mknumber(channels_type);
        json_append_member(result, "value", json_value);
        json_append_member(result, "code", json_code);
        json_append_member(result, "debug", json_mknumber(inbuf.dbg[0]));
        json_append_member(result, "dbg_chan", json_mknumber(channel));

        printf("channel %d adc_chan %d code %d value %x\n\r", channel, adc_chan, channels_type, inbuf.dbg[0]);
    }

bailout1:
    close(fd);
bailout2:
    return result;
}

/* Electrochemical sensors on i2c */

JsonNode *read_electrochemical(uint8_t channel, uint8_t channels_type)
{
    JsonNode *result = json_mkobject();
    int dbg = 1;

    if( dbg ){
        json_append_member(result, "random", json_mknumber((uint16_t)rand()));
        json_append_member(result, "dbg_chan", json_mknumber(channel));
        goto bailout;
    }

    int i2c_dev_node = open(voc_devices, O_RDWR), ret_val;

    if (i2c_dev_node < 0)
    {
        const char *errmsg = "Unable to open voc device node\n\r";

        json_append_member(result, "error", json_mkstring(errmsg));
        json_append_member(result, "dbg_chan", json_mknumber(channel));
        goto bailout;
    }

    ret_val = ioctl(i2c_dev_node, I2C_SLAVE, I2C_ADD);
    if (ret_val < 0)
    {
        const char *errmsg = "Unable to set voc slave mode\n\r";

        json_append_member(result, "error", json_mkstring(errmsg));
        json_append_member(result, "dbg_chan", json_mknumber(channel));
        goto bailout;
    }

    double reading;
    reading = SHT21_getHumidity(i2c_dev_node);
    json_append_member(result, "humidity", json_mknumber(reading));

    reading = SHT21_getTemperature(i2c_dev_node);
    json_append_member(result, "temperature", json_mknumber(reading));

    close(i2c_dev_node);
bailout:
    return result;
}

JsonNode *read_rs232(uint8_t channel, uint8_t channels_type)
{
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

double SHT21_getHumidity(int sensor_fd)
{
    uint16_t result; // return variable

    result = SHT21_readSensor_hm(sensor_fd, TRIGGER_RH_MEASUREMENT_NHM);
    //result = SHT21_readSensor_hm(TRIGGER_RH_MEASUREMENT_HM);

    return SHT21_CalcRH(result);
}

double SHT21_getTemperature(int sensor_fd)
{
    uint16_t result; // return variable

    result = SHT21_readSensor_hm(sensor_fd,TRIGGER_T_MEASUREMENT_NHM);
    //result = SHT21_readSensor_hm(TRIGGER_T_MEASUREMENT_HM);

    return SHT21_CalcT(result);
}

void SHT21_reset(int sensor_fd)
{
    int ret_val;
    ret_val = i2c_smbus_write_byte_data(sensor_fd,
                                        I2C_ADD,
                                        SOFT_RESET);

    sleep_ms(15); // wait for SHT to reset
}

#if 0
uint8_t SHT21_getSerialNumber(uint8_t return_sn)
{

    uint8_t serialNumber[8];

    // read memory location 1
    Wire.beginTransmission(I2C_ADD);
    Wire.write(0xFA);
    Wire.write(0x0F);
    Wire.endTransmission();

    Wire.requestFrom(I2C_ADD, 8);
    while (Wire.available() < 8)
    {
    }

    serialNumber[5] = Wire.read(); // read SNB_3
    Wire.read();                   // CRC SNB_3 not used
    serialNumber[4] = Wire.read(); // read SNB_2
    Wire.read();                   // CRC SNB_2 not used
    serialNumber[3] = Wire.read(); // read SNB_1
    Wire.read();                   // CRC SNB_1 not used
    serialNumber[2] = Wire.read(); // read SNB_0
    Wire.read();                   // CRC SNB_0 not used

    // read memory location 2
    Wire.beginTransmission(I2C_ADD);
    Wire.write(0xFC);
    Wire.write(0xC9);
    Wire.endTransmission();

    Wire.requestFrom(I2C_ADD, 6);
    while (Wire.available() < 6)
    {
    }

    serialNumber[1] = Wire.read(); // read SNC_1
    serialNumber[0] = Wire.read(); // read SNC_0
    Wire.read();                   // CRC SNC_1/SNC_0 not used
    serialNumber[7] = Wire.read(); // read SNA_1
    serialNumber[6] = Wire.read(); // read SNA_0
    Wire.read();                   // CRC SNA_1/SNA_0 not used

    return serialNumber[return_sn];
}
#endif
//==============================================================================
// PRIVATE
//==============================================================================

uint16_t SHT21_readSensor_hm(int sensor_fd, uint8_t command)
{
    uint8_t checksum;
    uint8_t data[3];
    uint16_t result;
    uint8_t n = 0;
    uint8_t d;
    int ret_val, read_value;

    if (command == TRIGGER_RH_MEASUREMENT_HM || command == TRIGGER_RH_MEASUREMENT_NHM)
        d = 30;
    if (command == TRIGGER_T_MEASUREMENT_HM || command == TRIGGER_T_MEASUREMENT_NHM)
        d = 85;

    ret_val = i2c_smbus_write_byte_data(sensor_fd,
                                        I2C_ADD,
                                        command);

    sleep_ms(d);

    do
    {
        n++;
        read_value = i2c_smbus_read_byte(sensor_fd);
        data[n] = read_value;
    }while( n < 3);

#if 0
    Wire.requestFrom(I2C_ADD, 3);

    while (Wire.available() < 3)
    {
        sleep_ms(10);
        n++;
        if (n > 10)
            return 0;
    }
#endif

    //data[0] = Wire.read();  // read data (MSB)
    //data[1] = Wire.read();  // read data (LSB)
    //checksum = Wire.read(); // read checksum
    checksum = data[2]; // read checksum


    result = (data[0] << 8);
    result += data[1];

    if (SHT21_CRC_Checksum(data, 2, checksum))
    {
        SHT21_reset(sensor_fd);
        return 1;
    }

    else
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

uint8_t SHT21_CRC_Checksum(uint8_t data[], uint8_t no_of_bytes, uint8_t checksum)
{
    uint8_t crc = 0;
    uint8_t byteCtr;

    //calculates 8-Bit checksum with given polynomial
    for (byteCtr = 0; byteCtr < no_of_bytes; ++byteCtr)
    {
        crc ^= (data[byteCtr]);
        uint8_t bit;
        for (bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    if (crc != checksum)
        return 1;
    else
        return 0;
}

void sleep_ms(int milliseconds)
{ // cross-platform sleep function
#ifdef WIN32
    Sleep(milliseconds);
#elif _POSIX_C_SOURCE >= 199309L
    struct timespec ts;
    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = (milliseconds % 1000) * 1000000;
    nanosleep(&ts, NULL);
#else
    if (milliseconds >= 1000)
        sleep(milliseconds / 1000);
    usleep((milliseconds % 1000) * 1000);
#endif
}