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

#ifndef HYSTRIX_SCANNER_H
#define HYSTRIX_SCANNER_H
#include <stddef.h>
#include <stdint.h>
#include "ccan_json_json.h"

#ifndef SPI_IOC_RD_MODE32
#define SPI_IOC_RD_MODE32               _IOR(SPI_IOC_MAGIC, 5, __u32)
#endif

#ifndef SPI_IOC_WR_MODE32
#define SPI_IOC_WR_MODE32               _IOW(SPI_IOC_MAGIC, 5, __u32)
#endif

#define GPIO_BANK 1

#define EC_CHANNELS 0

#define VOC_CHANNELS    (1 << 2)

#define SEL_0           (1 << 0)
#define SEL_1           (1 << 1)
#define SEL_2           (1 << 2)

#define ADC_0            0
#define ADC_1           (1 << 1)

JsonNode* read_electrochemical(uint8_t channel, uint8_t channels_type);
JsonNode* read_voc(uint8_t channel, uint8_t channels_type);
JsonNode* read_rs232(uint8_t channel, uint8_t channels_type);
uint8_t mux_channel(uint8_t channel);

typedef union {
    __u64 raw;
    uint16_t ch[4];
} adc_data_t;
#endif