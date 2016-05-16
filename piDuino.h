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
	// All functions of unistd.h must be called like this: unistd::the_function()
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
	LOW = 0,
	HIGH = 1,
	RISING = 2,
	FALLING = 3,
	BOTH = 4
}Digivalue;

// Arduino extra types
typedef bool boolean;
typedef uint8_t byte;
typedef unsigned int word;




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

// A char not found in a valid ASCII numeric field
#define NO_IGNORE_CHAR  '\x01' 

class SerialPi {

private:
	int sd;
	FILE * sd_file;
	const char *serialPort;
	long timeOut;
	timespec timeDiff(timespec start, timespec end);
	int timedPeek();
	int peekNextDigit(bool detectDecimal);
	long timeDiffmillis(timespec start, timespec end);
	char * int2bin(int n);

public:
	SerialPi();
	void begin(int baud);
	void begin(int baud, unsigned char config);
	void end();
	int available();
	int availableForWrite();
	bool find(const char *target);
	bool findUntil(const char *target, const char *terminator);
	void flush();
	long parseInt() { return parseInt(NO_IGNORE_CHAR); };
	long parseInt(char ignore);
	float parseFloat();
	int peek();
	size_t print(const char str[]);
	size_t print(char c);
	size_t print(unsigned char b, int base);
	size_t print(int n, int base);
	size_t print(unsigned int n, int base);
	size_t println(void);
	size_t println(const char c[]);
	size_t println(char c);
	size_t println(unsigned char b, int base);
	size_t println(int num, int base);
	size_t println(unsigned int num, int base);
	int read();
	size_t readBytes(char buffer[], size_t length);
	size_t readBytesUntil(char terminator, char buffer[], size_t length);
	void setTimeout(long millis);
	size_t write(uint8_t c);
	size_t write(const char *str);
	size_t write(char *buffer, size_t size);
	operator bool() { return (sd == -1) ? false : true; }
	
};




/////////////////////////////////////////////
//          WirePi class (I2C)             //
////////////////////////////////////////////



#define I2C_SLAVE 0x0703
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


/////////////////////////////////////////////
//          Digital I/O           		  //
////////////////////////////////////////////

// Pin modes
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define INPUT_PULLDOWN 0x3

// GPIO Driver name (user can change it)
extern char GPIO_DRIVER_NAME[];

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
int digitalRead(uint8_t pin);

/////////////////////////////////////////////
//          Analog I/O           		  //
////////////////////////////////////////////

int analogRead (int pin);
void analogWrite(int pin, int value);

/////////////////////////////////////////////
//          Advanced I/O           		  //
////////////////////////////////////////////
/////////////////////////////////////////////
//          Time      		     		  //
////////////////////////////////////////////

class TimeElapsed {											
    public:
        struct timespec timestamp;
        TimeElapsed(); 
};

unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long millis);
void delayMicroseconds(unsigned int us);

/////////////////////////////////////////////
//          Math           				  //
////////////////////////////////////////////

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352

#ifdef abs
#undef abs
#endif

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

long map(long x, long in_min, long in_max, long out_min, long out_max);
// pow() already in <math.h>...

/////////////////////////////////////////////
//          Trigonometry          		  //
////////////////////////////////////////////

// sin(rad) already in <math.h>...
// cos(rad) already in <math.h>...
// tan(rad) already in <math.h>...

/////////////////////////////////////////////
//          Characters           		  //
////////////////////////////////////////////

inline boolean isAlphaNumeric(int c) __attribute__((always_inline));
inline boolean isAlpha(int c) __attribute__((always_inline));
inline boolean isAscii(int c) __attribute__((always_inline));
inline boolean isWhitespace(int c) __attribute__((always_inline));
inline boolean isControl(int c) __attribute__((always_inline));
inline boolean isDigit(int c) __attribute__((always_inline));
inline boolean isGraph(int c) __attribute__((always_inline));
inline boolean isLowerCase(int c) __attribute__((always_inline));
inline boolean isPrintable(int c) __attribute__((always_inline));
inline boolean isPunct(int c) __attribute__((always_inline));
inline boolean isSpace(int c) __attribute__((always_inline));
inline boolean isUpperCase(int c) __attribute__((always_inline));
inline boolean isHexadecimalDigit(int c) __attribute__((always_inline));
inline int toAscii(int c) __attribute__((always_inline));
inline int toLowerCase(int c) __attribute__((always_inline));
inline int toUpperCase(int c)__attribute__((always_inline));

/////////////////////////////////////////////
//          Random Functions       		  //
////////////////////////////////////////////

void randomSeed(unsigned long);
long random(long);
long random(long, long);

/////////////////////////////////////////////
//          Bits and Bytes         		  //
////////////////////////////////////////////

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bit(b) (1UL << (b))

/////////////////////////////////////////////
//          External Interrupts    		  //
////////////////////////////////////////////
#define NOT_AN_INTERRUPT -1
#define digitalPinToInterrupt(p) ((p) >= 0 && (p) <= 26 ? (p) : NOT_AN_INTERRUPT)))

void attachInterrupt(uint8_t, void (*)(void), int mode);
void detachInterrupt(uint8_t);

/////////////////////////////////////////////
//          Interrupts           		  //
////////////////////////////////////////////







/*
uint8_t shiftIn  (uint8_t dPin, uint8_t cPin, bcm2835SPIBitOrder order);
void shiftOut (uint8_t dPin, uint8_t cPin, bcm2835SPIBitOrder order, uint8_t val);
void attachInterrupt(int p,void (*f)(), Digivalue m);
void detachInterrupt(int p);
void setup();
void loop();

// Helper functions
int getBoardRev();
uint32_t *mapmem(const char *msg, size_t size, int fd, off_t off);
void setBoardRev(int rev);
int raspberryPinNumber(int arduinoPin);
pthread_t *getThreadIdFromPin(int pin);
void * threadFunction(void *args);

*/
extern TimeElapsed ProgramStart;
extern SerialPi Serial;
extern WirePi Wire;
extern SPIPi SPI;

#endif
