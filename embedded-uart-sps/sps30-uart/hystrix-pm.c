/*
 * Copyright (c) 2018, Sensirion AG
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
#include <stdlib.h>
#include <string.h>
#include "sensirion_uart.h"
#include "sps30.h"

/**
 * TO USE CONSOLE OUTPUT (PRINTF) AND WAIT (SLEEP) PLEASE ADAPT THEM TO YOUR
 * PLATFORM
 */
//#define printf(...)
#define MAX_ATTEMPTS 3

int main(int argc, char *argv[]){
    struct sps30_measurement m;
    char serial[SPS30_MAX_SERIAL_LEN];
    const uint8_t AUTO_CLEAN_DAYS = 4;
    int16_t ret;
    int attempt = 0;

    struct sockaddr_un server;
    int sock;

    int exitcode = EXIT_FAILURE;
    char buf[1024];

    if( argc < 2){
        perror("not enough arguments");
        exitcode = EXIT_FAILURE;
        goto bailout;
    }

    int chan_input = atoi(argv[2]);

    while (sensirion_uart_open() != 0 ) {
        if( attempt < MAX_ATTEMPTS ){
            attempt++;
            sensirion_sleep_usec(1000000); /* sleep for 1s */
        }
        else{
            perror("UART init failed");
            exitcode = EXIT_FAILURE;
            goto bailout;
        }                
    }

    attempt = 0;
    /* Busy loop for initialization, because the main loop does not work without
     * a sensor.
     */
    while (sps30_probe() != 0) {
        if( attempt < MAX_ATTEMPTS){
            sensirion_sleep_usec(1000000); /* sleep for 1s */
            attempt++;
        }
        else{
            perror("SPS30 sensor probing failed\n");
            exitcode = EXIT_FAILURE;
            goto bailout;
        }
        
    }

    struct sps30_version_information version_information;
    ret = sps30_read_version(&version_information);
    if (ret) {
        perror("error reading version information");
    } else {
        // printf("FW: %u.%u HW: %u, SHDLC: %u.%u\n",
        //        version_information.firmware_major,
        //        version_information.firmware_minor,
        //        version_information.hardware_revision,
        //        version_information.shdlc_major,
        //        version_information.shdlc_minor);
        if (version_information.firmware_major >= 2) {
            ret = sps30_wake_up();
        }        
    }


    ret = sps30_set_fan_auto_cleaning_interval_days(AUTO_CLEAN_DAYS);


    while ( attempt < MAX_ATTEMPTS ) {
        ret = sps30_start_measurement();

        if (ret < 0) {
            attempt++;
            sensirion_sleep_usec(1000000); /* sleep for 1s */      
            continue;
        }        

        ret = sps30_read_measurement(&m);
        if (ret < 0) {
            attempt++;
            sensirion_sleep_usec(1000000); /* sleep for 1s */            
            continue;
        } else {
            sprintf(buf, "{\"chan\" : %d, \"val\": \""
                " %0.2f pm1.0"
                " %0.2f pm2.5"
                " %0.2f pm4.0"
                " %0.2f pm10.0"
                " %0.2f nc0.5"
                " %0.2f nc1.0"
                " %0.2f nc2.5"
                " %0.2f nc4.5"
                " %0.2f nc10.0"
                " %0.2f typical particle size"
                "\"}",
                chan_input,            
                m.mc_1p0, m.mc_2p5, m.mc_4p0, m.mc_10p0, m.nc_0p5,
                m.nc_1p0, m.nc_2p5, m.nc_4p0, m.nc_10p0,
                m.typical_particle_size);
            exitcode = EXIT_SUCCESS;
            attempt = MAX_ATTEMPTS;
            break;
        }
    }

    /* Stop measurement for 1min to preserve power. Also enter sleep mode
        * if the firmware version is >=2.0.
        */
    ret = sps30_stop_measurement();
    ret = sensirion_uart_close();

    if(exitcode == EXIT_SUCCESS)
    {
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
    }

bailout:
    return exitcode;
}
