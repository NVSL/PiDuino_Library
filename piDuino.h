/*
*   piDuino Library
*
*   Arduino like functions for (Raspberry Pi Zero)
*   piDuino is a fork of arduPi ->
*   (https://www.cooking-hacks.com/documentation/tutorials/raspberry-pi-to-arduino-shields-connection-bridge/)
*
*   version: 1.0.0
*   author: Jorge Garza (jgarzagu@gmail.com)
*   
*/

#ifndef piDuino_h
#define piDuino_h

#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <string.h>
#include <time.h>
#include <termios.h> 
#include <ctype.h>
#include <sys/ioctl.h>
#include <limits.h>
#include <algorithm>
#include <limits.h>
#include <pthread.h>
#include <poll.h>
#include <stdarg.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/spi/spidev.h>


// Remove some PROGMEM space macros if posible
#define PROGMEM
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#define pgm_read_float(addr) (*(const float *)(addr))
#define pgm_read_ptr(addr) (*(const void *)(addr))


namespace unistd {
	//All functions of unistd.h must be called like this: unistd::the_function()
    #include <unistd.h>
};

enum Representation{
	BIN,
	OCT,
	DEC,
	HEX,
	BYTE
};

typedef enum{
	INPUT,
	OUTPUT
}Pinmode;

typedef enum{
	LOW = 0,
	HIGH = 1,
	RISING = 2,
	FALLING = 3,
	BOTH = 4
}Digivalue;

typedef bool boolean;
typedef unsigned char byte;


/////////////////////////////////////////////
//          SerialPi class (UART)         //
////////////////////////////////////////////

// Defines for setting data, parity, and stop bits
// e.g SERIAL_ABC
// A= data (5 bits, 6 bits, 7 bits, 8 bits)
// B= parity (None, Even, Odd)
// C= stop bits (1 bit, 2 bits) 
#define SERIAL_5N1 0x00
#define SERIAL_6N1 0x02
#define SERIAL_7N1 0x04
#define SERIAL_8N1 0x06 // default
#define SERIAL_5N2 0x08
#define SERIAL_6N2 0x0A
#define SERIAL_7N2 0x0C
#define SERIAL_8N2 0x0E
#define SERIAL_5E1 0x20
#define SERIAL_6E1 0x22
#define SERIAL_7E1 0x24
#define SERIAL_8E1 0x26
#define SERIAL_5E2 0x28
#define SERIAL_6E2 0x2A
#define SERIAL_7E2 0x2C
#define SERIAL_8E2 0x2E
#define SERIAL_5O1 0x30
#define SERIAL_6O1 0x32
#define SERIAL_7O1 0x34
#define SERIAL_8O1 0x36
#define SERIAL_5O2 0x38
#define SERIAL_6O2 0x3A
#define SERIAL_7O2 0x3C
#define SERIAL_8O2 0x3E


class SerialPi {

private:
	int sd;
	unsigned char c;
	const char *serialPort;
	long timeOut;
	timespec time1, time2;
	timespec timeDiff(timespec start, timespec end);
	int timeDiffmillis(timespec start, timespec end);
	char * int2bin(int i);

public:

	SerialPi();
	void begin(int baud);
	void begin(int baud, unsigned char config);
	int available();
	int availableForWrite();
	bool find(const char *target);
	bool findUntil(const char *target, const char *terminator);
	void flush();

	long parseInt();
	float parseFloat();

	int read();
	int readBytes(char message[], int size);
	int readBytesUntil(char character,char buffer[],int length);
	char peek();
	void print(const char *message);
	void print(char message);
	void print(unsigned char i,Representation rep);
	void print(float f, int precission);
	void println(const char *message);
	void println(char message);
	void println(int i, Representation rep);
	void println(float f, int precission);
	int write(unsigned char message);
	int write(const char *message);
	int write (char *message, int size);
	void setTimeout(long millis);
	void end();
};




/////////////////////////////////////////////
//          WirePi class (I2C)             //
////////////////////////////////////////////



#define I2C_SLAVE       0x0703  // Use this slave address
#define BUFFER_LENGTH 32


class WirePi {
	private:
		int fd;
        static uint8_t rxBuffer[];
        static uint8_t rxBufferIndex;
        static uint8_t rxBufferLength;

        static uint8_t txBuffer[];
        static uint8_t txBufferIndex;
        static uint8_t txBufferLength;

        static uint8_t transmitting;

        int i2c_write_bytes(int file, uint8_t *txBuff, size_t numBytes);
        int i2c_read_bytes(int file, uint8_t *rxBuff, size_t numBytes);

	public:
		WirePi();
		void begin();
		void end();
		uint8_t  requestFrom(uint8_t address, uint8_t quantity);
		void beginTransmission(uint8_t address);
		uint8_t endTransmission();		
		size_t write(uint8_t data);
		size_t write(const char *data);
		size_t write(uint8_t *data, size_t quantity);
		int available(void);
		int read(void);
		
};




/////////////////////////////////////////////
//          SPIPi class (SPI)             //
////////////////////////////////////////////

#ifndef LSBFIRST
#define LSBFIRST 1
#endif
#ifndef MSBFIRST
#define MSBFIRST 0
#endif

// SPI Modes 
#define SPI_MODE0 SPI_MODE_0 
#define SPI_MODE1 SPI_MODE_1
#define SPI_MODE2 SPI_MODE_2
#define SPI_MODE3 SPI_MODE_3

// SPI Speed taking a 16 Mhz arduino clock speed
#define SPI_CLOCK_DIV2 8000000
#define SPI_CLOCK_DIV4 4000000
#define SPI_CLOCK_DIV8 2000000
#define SPI_CLOCK_DIV16 1000000
#define SPI_CLOCK_DIV32 500000
#define SPI_CLOCK_DIV64 250000
#define SPI_CLOCK_DIV128 125000

class SPISettings {
	private:
		uint32_t spiClock;
		uint8_t spiBitOrder;
		uint8_t spiDataMode;
	public:
		SPISettings() {
			spiClock = 4000000;
			spiBitOrder = MSBFIRST;
			spiDataMode = SPI_MODE0;
		}

		SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {
			spiClock = clock;
			spiBitOrder = bitOrder;
			spiDataMode = dataMode;
		}

	friend class SPIPi;
};

class SPIPi {
	private:
		int fd;
		void spi_transfer_bytes(int file, uint8_t *data, size_t numBytes);
	public:
		SPIPi();
  		void begin();
    	void end();
    	void beginTransaction(SPISettings settings);
    	void endTransaction();
    	void setBitOrder(uint8_t bitOrder);
 		void setClockDivider(uint32_t clockDiv);
		void setDataMode(uint8_t dataMode);
 		uint8_t transfer(uint8_t data);
 		uint16_t transfer16(uint16_t data);
 		void transfer(void *buf, size_t count);
};

// Some useful arduino functions
void pinMode(int pin, Pinmode mode);
void analogWrite(int pin, int value);
void digitalWrite(int pin, int value);
void digitalWriteSoft(int pin, int value);
void delay(long millis);
void delayMicroseconds(long micros);
int digitalRead(int pin);
int analogRead (int pin);


/*
uint8_t shiftIn  (uint8_t dPin, uint8_t cPin, bcm2835SPIBitOrder order);
void shiftOut (uint8_t dPin, uint8_t cPin, bcm2835SPIBitOrder order, uint8_t val);
void attachInterrupt(int p,void (*f)(), Digivalue m);
void detachInterrupt(int p);
void setup();
void loop();
long millis();

// Helper functions
int getBoardRev();
uint32_t *mapmem(const char *msg, size_t size, int fd, off_t off);
void setBoardRev(int rev);
int raspberryPinNumber(int arduinoPin);
pthread_t *getThreadIdFromPin(int pin);
void * threadFunction(void *args);

*/

extern SerialPi Serial;
extern WirePi Wire;
extern SPIPi SPI;

#endif
