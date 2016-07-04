#ifndef SPILinux_h
#define SPILinux_h

#include <stdint.h>
#include <stdlib.h>
#include <linux/spi/spidev.h>

// All functions of unistd.h must be called like this: unistd::the_function()
namespace unistd {
    #include <unistd.h>
};

/////////////////////////////////////////////
//          SPILinux class (SPI)          //
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

extern char SPI_DRIVER_NAME[]; 

class SPISettings {
	public:
		uint32_t spiClock;
		uint8_t spiBitOrder;
		uint8_t spiDataMode;

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

class SPILinux {
	private:
		int fd;
		void spi_transfer_bytes(int file, uint8_t *data, size_t numBytes);
	public:
		SPILinux();
  		void begin();
  		void begin(const char *spiDeviceName);
    	void end();
    	void beginTransaction(SPISettings settings);
    	void endTransaction();
    	void setBitOrder(uint8_t bitOrder);
 		void setClockDivider(uint32_t clockDiv);
		void setDataMode(uint8_t dataMode);
 		uint8_t transfer(uint8_t data);
 		//uint16_t transfer16(uint16_t data);
 		void transfer(void *buf, size_t count);
};


extern SPILinux SPI;

#endif