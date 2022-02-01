/*
 * I2C-Generator: 0.3.0
 * Yaml Version: 0.1.0
 * Template Version: 0.7.0-62-g3d691f9
 */
/*
 * Copyright (c) 2021, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>  // printf

#include <sys/ioctl.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <linux/types.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"
#include "sgp40_i2c.h"


int main(int argc, char *argv[])
{
    struct sockaddr_un server;
    int sock;
    int16_t error = 0;
    int exitcode = EXIT_SUCCESS;
    char buf[256];

    if( argc < 2){
        perror("not enough arguments");
        exitcode = EXIT_FAILURE;
        goto bailout;
    }

    int chan_input = atoi(argv[2]);

    sensirion_i2c_hal_init();

    uint16_t serial_number[3];
    uint8_t serial_number_size = 3;

    error = sgp40_get_serial_number(serial_number, serial_number_size);

    if( error ){

        sprintf(buf, "Error executing sgp40_get_serial_number(): %i\n", error);
        perror(buf);
        exitcode = EXIT_FAILURE;
        goto bailout;
    }

    // Start Measurement

    // Parameters for deactivated humidity compensation:
    uint16_t default_rh = 0x8000;
    uint16_t default_t = 0x6666;
    uint16_t sraw_voc;

    sensirion_i2c_hal_sleep_usec(1000000);
    error = sgp40_measure_raw_signal(default_rh, default_t, &sraw_voc);

    if (error) {
        sensirion_i2c_hal_sleep_usec(1000000);
        error = sgp40_measure_raw_signal(default_rh, default_t, &sraw_voc);
        if( error ){
            perror("cannot read sgp40");
            exitcode = EXIT_FAILURE;
            goto bailout;
        }
    } 

    //fallthrough after retry successful
    sprintf(buf, "{ \"chan\" : %d, \"val\" : %u }", chan_input, sraw_voc);    

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

    if (write(sock, buf, strlen(buf)) < 0)
    {
        perror("writing on stream socket");
        exitcode = EXIT_FAILURE;
    }

    close(sock);

bailout:
    return exitcode;
}
