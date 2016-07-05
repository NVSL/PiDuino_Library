/*
  SPI.cpp - LinuxDuino I2C library

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
#include <termios.h> 
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include "Wire.h"


/////////////////////////////////////////////
//          WireLinux class (I2C)          //
////////////////////////////////////////////

char I2C_DRIVER_NAME[128] = ""; // "" means search for any I2C device


//// Private methods ///
uint8_t WireLinux::rxBuffer[BUFFER_LENGTH];
uint8_t WireLinux::rxBufferIndex = 0;
uint8_t WireLinux::rxBufferLength = 0;

uint8_t WireLinux::txBuffer[BUFFER_LENGTH];
uint8_t WireLinux::txBufferIndex = 0;
uint8_t WireLinux::txBufferLength = 0;

uint8_t WireLinux::transmitting = 0;


int WireLinux::i2c_write_bytes(int file, uint8_t *txBuff, size_t numBytes)
{
    int bytes_written = 0;

    if (numBytes == 0) {
        return bytes_written;
    } else {
        bytes_written = unistd::write(file, txBuff, numBytes);
        if ( bytes_written < 0) {
            // errno == 5 (Input/Output error) means I2C cables
            // may not be connected properly.
            // Make noise about everything else except errno == 5. 
            if (errno != 5 ) {
                fprintf(stderr, "%s(): i2c write error: %s \n",
                    __func__, strerror (errno));
            }
        }
    }

    return bytes_written;
}

int WireLinux::i2c_read_bytes(int file, uint8_t *rxBuff, size_t numBytes)
{
    int bytes_read = 0;

    if (numBytes == 0) {
        return bytes_read;
    } else {
        bytes_read = unistd::read(file, rxBuff, numBytes);
        if ( bytes_read < 0) {
            // errno == 5 (Input/Output error) means I2C cables 
            // may not be connected properly.
            // Make noise about everything else except errno == 5. 
            if (errno != 5 ) {
                fprintf(stderr, "%s(): i2c read error: %s \n",
                    __func__, strerror (errno));
            }
        }
    }

    return bytes_read;
}

////  Public methods ////

//Constructor
WireLinux::WireLinux()
{
    fd = -1;
}

void WireLinux::begin()
{
    begin(I2C_DRIVER_NAME);
}

// Initialize the Wire library
void WireLinux::begin(const char *i2cDeviceName)
{
    FILE *fn, *fp;
    char path[1024];
    char *filename;
    char *i2cDevice;
    char *tmpi2cDevice = NULL;

    // If I2C device name is empty, then search for the first available i2c in /dev/
    // else  try to open the given device name
    if (strcmp(i2cDeviceName, "") == 0) {

        // Process the command below to search for i2c-1 device driver excistance
        fn = popen("/bin/ls /dev/ | /bin/grep i2c-1" , "r");
        if (fn == NULL) {
            fprintf(stderr, "%s(): failed to run command "
                "\"/bin/ls /dev/ | /bin/grep i2c-1\"\n",__func__);
            exit(1);
        }
        
        // Process the command below to search for i2c-x devices drivers excistance, 
        // where x = 0, 1, etc
        fp = popen("/bin/ls /dev/ | /bin/grep i2c-" , "r");
        if (fp == NULL) {
            fprintf(stderr, "%s(): failed to run command "
                "\"/bin/ls /dev/ | /bin/grep i2c-\"\n",__func__);
            exit(1);
        }


        // If i2c-1 exists (RPI main i2c) then set it to open, 
        // else set any other existant i2c-x like i2c-0 for old RPI revisions 
        if (fgets(path, sizeof(path)-1, fn) != NULL) {
            // Set i2c-1 device
            asprintf(&tmpi2cDevice, "i2c-1");
        } else {
            // Set i2c-x device
            while (fgets(path, sizeof(path)-1, fp) != NULL) {
                asprintf(&tmpi2cDevice, "%s", path);
                break;
            }
        }

        // If no I2C device driver is enabled or installed then exit. 
        if (tmpi2cDevice == NULL) {
            fprintf(stderr, "%s(): filed to locate any \"i2c-x\" device driver in /dev/. "
                "please install or enable a i2c interface in your board \n",__func__);
            exit(1);
        }

        // Set i2c-x device found to /dev/i2c-x fromat
        asprintf(&i2cDevice, "/dev/%s", tmpi2cDevice);
    } else {
        // Set user given i2c device name
        asprintf(&i2cDevice, "%s", i2cDeviceName);
    }

    // Open i2c device name (e.g /dev/i2c-x)
    asprintf(&filename,"%s", i2cDevice);
    fd = open(filename, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "%s(): error openning I2C device %s: %s\n",
            __func__, filename, strerror (errno));
        exit(1);
    }

    rxBufferIndex = 0;
    rxBufferLength = 0;

    txBufferIndex = 0;
    txBufferLength = 0;

}

void WireLinux::end()
{
    unistd::close(fd);
    fd = -1;
}

// Initialize the Wire library with a slave address
/*
void WireLinux::begin(uint8_t address) 
{
    // TODO, Still reading documentation for linux new I2c slave support
}
*/

uint8_t WireLinux::requestFrom(uint8_t address, uint8_t quantity)
{

    if (fd < 0) {
        fprintf(stderr, 
            "%s(): initialize I2C first with Wire.begin() \n",
             __func__);
        exit(1);
    }

    if (ioctl(fd, I2C_SLAVE, address) < 0) {
        fprintf(stderr, "%s(): ioctl error: %s\n",
            __func__, strerror (errno));
        exit(1);
    }

    // clamp to buffer length
    if(quantity > BUFFER_LENGTH){
        quantity = BUFFER_LENGTH;
    }

    // perform blocking read into buffer
    uint8_t read = i2c_read_bytes(fd, rxBuffer, quantity);
    // set rx buffer iterator vars
    rxBufferIndex = 0;
    rxBufferLength = read;

    return read;
}


//Begin a transmission to the I2C slave device with the given address
void WireLinux::beginTransmission(uint8_t address)
{

    if (fd < 0) {
        fprintf(stderr, 
            "%s(): initialize I2C first with Wire.begin() \n", __func__);
        exit(1);
    }

    if (ioctl(fd, I2C_SLAVE, address) < 0) {
        fprintf(stderr, "%s(): ioctl error: %s\n", __func__, strerror (errno));
        exit(1);
    }

    // indicate that we are transmitting
    transmitting = 1;
    // reset tx buffer iterator vars
    txBufferIndex = 0;
    txBufferLength = 0;
}

// Writes data to the I2C, returns bytes written.
size_t WireLinux::write(uint8_t data)
{

    if (transmitting) {
        // in master transmitter mode
        // don't bother if buffer is full
        if (txBufferLength >= BUFFER_LENGTH) {
          return 0;
        }

        // put byte in tx buffer
        txBuffer[txBufferIndex] = data;
        ++txBufferIndex;
        // update amount in buffer   
        txBufferLength = txBufferIndex;
    } else {
        // in slave send mode
        // reply to master
        i2c_write_bytes(fd, &data, 1);
    }

    return 1;

}

// Writes data to the I2C in form of string, returns bytes written. 
size_t WireLinux::write(const char *str)
{
    size_t byteswritten = 0;

    for (size_t i = 0; i < strlen(str) ; i++) {
        // If transmitting data >= BUFFER_LENGTH, then break.
        if (write(str[i]) == 0) {
            break;
        }
        byteswritten++;
    }

    return byteswritten;
}


// Writes data to the I2C, returns bytes written. 
size_t WireLinux::write(uint8_t *data, size_t quantity)
{

    size_t byteswritten = 0;

    if (transmitting) {
        // in master transmitter mode
        for(size_t i = 0; i < quantity; ++i){
            write(data[i]);
        }
        byteswritten = quantity;
    } else {
        // in slave send mode
        // reply to master
        byteswritten = i2c_write_bytes(fd, data, quantity);
    }
    
    return byteswritten;
}


int WireLinux::available(void)
{
  return rxBufferLength - rxBufferIndex;
}


int WireLinux::read(void)
{
  int value = -1;
  
  // get each successive byte on each call
  if (rxBufferIndex < rxBufferLength) {
    value = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
  }

  return value;
}


uint8_t WireLinux::endTransmission()
{
    uint8_t ret;

    if (fd < 0) {
        fprintf(stderr, "%s(): initialize I2C first with Wire.begin() \n", __func__);
        exit(1);
    }

    // Transmit Data 
    ret = i2c_write_bytes(fd, txBuffer, txBufferLength);

    // reset tx buffer iterator vars
    txBufferIndex = 0;
    txBufferLength = 0;
    // indicate that we are done transmitting
    transmitting = 0;

    return ret;
}


WireLinux Wire = WireLinux();