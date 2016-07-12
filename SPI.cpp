/*
  SPI.cpp - LinuxDuino SPI library

  Copyright (c) 2016 Jorge Garza <jgarzagu@ucsd.edu>

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "SPI.h"

// All functions of unistd.h must be called like this: unistd::the_function()
namespace unistd {
    #include <unistd.h>
};


/////////////////////////////////////////////
//          SPILinux class (SPI)          //
////////////////////////////////////////////

char SPI_DRIVER_NAME[128] = ""; // "" means search for any SPI device

////  Private methods ////

// Transfers SPI data, recieved data is stored back in the data buffer
void SPILinux::spi_transfer_bytes(int file, uint8_t *data, size_t numBytes) 
{
    struct spi_ioc_transfer spi;

    memset (&spi, 0, sizeof(spi));

    spi.tx_buf        = (unsigned long)data;
    spi.rx_buf        = (unsigned long)data;
    spi.len           = numBytes;

    if (ioctl (file, SPI_IOC_MESSAGE(1), &spi) < 0) {
        fprintf(stderr, "%s(): spi transfer error: %s \n", __func__, strerror (errno));
    }
}

////  Public methods ////


SPILinux::SPILinux()
{
    fd = -1;
}

void SPILinux::begin()
{
    begin(SPI_DRIVER_NAME);
}

void SPILinux::begin(const char *spiDeviceName)
{
    FILE *fp;
    char path[1024];
    char *filename;
    char *spiDevice;
    char *tmpspiDevice = NULL;
    uint8_t bitsPerWord = 8;

    // If SPI device name is empty, then search for the first available spi in /dev/
    // else try to open the given device name
    if (strcmp(spiDeviceName, "") == 0) {

        // Process the command below to search for spidev device driver excistance
        fp = popen("/bin/ls /dev/ | /bin/grep spidev" , "r");
        if (fp == NULL) {
            fprintf(stderr, "%s(): failed to run command "
                "\"/bin/ls /dev/ | /bin/grep spidev\"\n",__func__);
            exit(1);
        }


        // If any spidevX.X exits, then set to open it.
        // If there are two or more (e.g spidev0.0 and spidev 0.1) 
        // then the one with the lowest number (spidev0.0) will be set
        while (fgets(path, sizeof(path)-1, fp) != NULL) {
            asprintf(&tmpspiDevice, "%s", path);
            tmpspiDevice[strcspn(tmpspiDevice, "\n")] = '\0'; // Remove \n if any
            break;
        }

        // If no SPI device driver is enabled or installed then exit. 
        if (tmpspiDevice == NULL) {
            fprintf(stderr, "%s(): filed to locate any \"spidevX.X\" device driver in /dev/. "
                "please install or enable a spi interface in your board \n",__func__);
            exit(1);
        }

        // Set spidevX.X device found to /dev/spidevX.X fromat
        asprintf(&spiDevice, "/dev/%s", tmpspiDevice);
    } else {
        // Set user given SPI device name
        asprintf(&spiDevice, "%s", spiDeviceName);
    }

    // Open /dev/spidevX.X device 
    asprintf(&filename, "%s", spiDevice);
    fd = open(filename, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "%s(): error openning SPI device %s: %s\n",
            __func__, filename, strerror (errno));
        exit(1);
    }

    // Set bits per word to 8 always
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bitsPerWord) < 0) {
        fprintf(stderr, "%s(): ioctl error: %s\n", __func__, strerror (errno));
        exit(1);
    }

}

void SPILinux::end()
{
    unistd::close(fd);
    fd = -1;
}

void SPILinux::beginTransaction(SPISettings settings)
{

    if (fd < 0) {
        fprintf(stderr, "%s(): initialize SPI first with SPI.begin() \n", __func__);
        exit(1);
    }

    // Set SPI mode (0,1,2,3)
    if (ioctl(fd, SPI_IOC_WR_MODE, &settings.spiDataMode) < 0) {
        fprintf(stderr, "%s(): ioctl error: %s\n", __func__, strerror (errno));
        exit(1);
    }

    // Set SPI bit order (LSB/MSB)
    if (ioctl(fd, SPI_IOC_WR_LSB_FIRST, &settings.spiBitOrder) < 0) {
        fprintf(stderr, "%s(): ioctl error: %s\n", __func__, strerror (errno));
        exit(1);
    }

    // Set max hz speed
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &settings.spiClock) < 0) {
        fprintf(stderr, "%s(): ioctl error: %s\n", __func__, strerror (errno));
        exit(1);
    }

}

void SPILinux::endTransaction()
{
    // Do Nothing
}

void SPILinux::setBitOrder(uint8_t bitOrder)
{
    if (fd < 0) {
        fprintf(stderr, "%s(): initialize SPI first with SPI.begin() \n", __func__);
        exit(1);
    }

    // Set SPI bit order (LSB/MSB)
    if (ioctl(fd, SPI_IOC_WR_LSB_FIRST, &bitOrder) < 0) {
        fprintf(stderr, "%s(): ioctl error: %s\n", __func__, strerror (errno));
        exit(1);
    }
}

void SPILinux::setClockDivider(uint32_t clockDiv)
{
    if (fd < 0) {
        fprintf(stderr, "%s(): initialize SPI first with SPI.begin() \n", __func__);
        exit(1);
    }

    // Set max hz speed
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &clockDiv) < 0) {
        fprintf(stderr, "%s(): ioctl error: %s\n",
            __func__, strerror (errno));
        exit(1);
    }
}

void SPILinux::setDataMode(uint8_t dataMode)
{
    if (fd < 0) {
        fprintf(stderr, "%s(): initialize SPI first with SPI.begin() \n", __func__);
        exit(1);
    }

    // Set SPI mode (0,1,2,3)
    if (ioctl(fd, SPI_IOC_WR_MODE, &dataMode) < 0) {
        fprintf(stderr, "%s(): ioctl error: %s\n", __func__, strerror (errno));
        exit(1);
    }
}

uint8_t SPILinux::transfer(uint8_t data)
{
    uint8_t transferData;
    transferData = data;
    spi_transfer_bytes(fd, &transferData, 1);
    return transferData;
}

/*
uint16_t SPILinux::transfer16(uint16_t data)
{
    // TODO
}
*/

void SPILinux::transfer(void *buf, size_t count)
{
   spi_transfer_bytes(fd, (uint8_t *)buf, count);
}


SPILinux SPI = SPILinux();