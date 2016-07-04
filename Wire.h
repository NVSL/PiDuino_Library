#ifndef WireLinux_h
#define WireLinux_h

#include <stdint.h>
#include <stdlib.h>

// All functions of unistd.h must be called like this: unistd::the_function()
namespace unistd {
    #include <unistd.h>
};

/////////////////////////////////////////////
//          WireLinux class (I2C)         //
////////////////////////////////////////////

#define I2C_SLAVE 0x0703
#define BUFFER_LENGTH 32

extern char I2C_DRIVER_NAME[]; 

class WireLinux {
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
		WireLinux();
		void begin();
		void begin(const char *i2cDeviceName);
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


extern WireLinux Wire;


#endif