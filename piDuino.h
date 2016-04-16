/*
*   piDuino Library
*
*   Arduino like functions for (Raspberry Pi Zero)
*   piDuino is a fork of arduPi ->
*   (https://www.cooking-hacks.com/documentation/tutorials/raspberry-pi-to-arduino-shields-connection-bridge/)
*
*   version: 1.0.0
*   author: Anonymous
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


class SerialPi {

private:
	int sd,status;
	const char *serialPort;
	unsigned char c;
	struct termios options;
	int speed;
	long timeOut;
	timespec time1, time2;
	timespec timeDiff(timespec start, timespec end);
	char * int2bin(int i);
	char * int2hex(int i);
	char * int2oct(int i);

public:

	SerialPi();
	void begin(int serialSpeed);
	int available();
	char read();
	int readBytes(char message[], int size);
	int readBytesUntil(char character,char buffer[],int length);
	bool find(const char *target);
	bool findUntil(const char *target, const char *terminal);
	long parseInt();
	float parseFloat();
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
	void flush();
	void setTimeout(long millis);
	void end();
};




/////////////////////////////////////////////
//          WirePi class (I2C)             //
////////////////////////////////////////////



#define I2C_SLAVE       0x0703  // Use this slave address
#define BUFFER_LENGTH 32


class WirePi{
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


class SPIPi{
	public:
		SPIPi();
  		void begin();
    	void end();
    	void setBitOrder(uint8_t order);
 		void setClockDivider(uint16_t divider);
		void setDataMode(uint8_t mode);
 		void chipSelect(uint8_t cs);
		void setChipSelectPolarity(uint8_t cs, uint8_t active);
 		uint8_t transfer(uint8_t value);
 		void transfernb(char* tbuf, char* rbuf, uint32_t len);
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
