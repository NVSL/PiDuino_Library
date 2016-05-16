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


#include "piDuino.h"


/////////////////////////////////////////////
//          SerialPi class (UART)         //
////////////////////////////////////////////


////  Public methods ////

//Constructor
SerialPi::SerialPi()
{
    // Default serial driver and timeout
    serialPort="/dev/ttyAMA0";
    timeOut = 1000;
    sd = -1;
    sd_file = NULL;
}

void SerialPi::begin(int baud)
{
    begin(baud, SERIAL_8N1);
}

// Sets the data rate in bits per second (baud) for serial data transmission
void SerialPi::begin(int baud, unsigned char config)
{

    int speed;
    int DataSize, ParityEN, Parity, StopBits;
    struct termios options;
    int flags;

    // Open Serial port 
	if ((sd = open(serialPort, O_RDWR | O_NOCTTY | O_NONBLOCK)) == -1) {
		fprintf(stderr,"Unable to open the serial port %s - \n", serialPort);
		exit(-1);
	}

    // We obtain a pointer to FILE structure (sd_file) from the file descriptor sd
    // and set it to be non-blocking
    sd_file = fdopen(sd,"r+");
    flags = fcntl( fileno(sd_file), F_GETFL );
    fcntl(fileno(sd_file), F_SETFL, flags | O_NONBLOCK);
    

    // Set Serial options: baudRate/speed, data size and parity.

    switch (baud) {
        case      50:   speed =      B50 ; break ;
        case      75:   speed =      B75 ; break ;
        case     110:   speed =     B110 ; break ;
        case     134:   speed =     B134 ; break ;
        case     150:   speed =     B150 ; break ;
        case     200:   speed =     B200 ; break ;
        case     300:   speed =     B300 ; break ;
        case     600:   speed =     B600 ; break ;
        case    1200:   speed =    B1200 ; break ;
        case    1800:   speed =    B1800 ; break ;
        case    2400:   speed =    B2400 ; break ;
        case    9600:   speed =    B9600 ; break ;
        case   19200:   speed =   B19200 ; break ;
        case   38400:   speed =   B38400 ; break ;
        case   57600:   speed =   B57600 ; break ;
        case  115200:   speed =  B115200 ; break ;
        case  230400:   speed =  B230400 ; break ;
        case  460800:   speed =  B460800 ; break ;
        case  500000:   speed =  B500000 ; break ;
        case  576000:   speed =  B576000 ; break ;
        case  921600:   speed =  B921600 ; break ;
        case 1000000:   speed = B1000000 ; break ;
        case 1152000:   speed = B1152000 ; break ;
        case 1500000:   speed = B1500000 ; break ;
        case 2000000:   speed = B2000000 ; break ;
        case 2500000:   speed = B2500000 ; break ;
        case 3000000:   speed = B3000000 ; break ;
        case 3500000:   speed = B3500000 ; break ;
        case 4000000:   speed = B4000000 ; break ;
        default:        speed =   B9600 ; break ;
    }

	tcgetattr(sd, &options);
	cfmakeraw(&options);
	cfsetispeed (&options, speed);
	cfsetospeed (&options, speed);

    switch (config) {
        case SERIAL_5N1: DataSize = CS5; ParityEN = 0; Parity = 0; StopBits = 0; break;
        case SERIAL_6N1: DataSize = CS6; ParityEN = 0; Parity = 0; StopBits = 0; break;
        case SERIAL_7N1: DataSize = CS7; ParityEN = 0; Parity = 0; StopBits = 0; break;
        case SERIAL_8N1: DataSize = CS8; ParityEN = 0; Parity = 0; StopBits = 0; break;
        case SERIAL_5N2: DataSize = CS5; ParityEN = 0; Parity = 0; StopBits = 1; break;
        case SERIAL_6N2: DataSize = CS6; ParityEN = 0; Parity = 0; StopBits = 1; break;
        case SERIAL_7N2: DataSize = CS7; ParityEN = 0; Parity = 0; StopBits = 1; break;
        case SERIAL_8N2: DataSize = CS8; ParityEN = 0; Parity = 0; StopBits = 1; break;
        case SERIAL_5E1: DataSize = CS5; ParityEN = 1; Parity = 0; StopBits = 0; break;
        case SERIAL_6E1: DataSize = CS6; ParityEN = 1; Parity = 0; StopBits = 0; break;
        case SERIAL_7E1: DataSize = CS7; ParityEN = 1; Parity = 0; StopBits = 0; break;
        case SERIAL_8E1: DataSize = CS8; ParityEN = 1; Parity = 0; StopBits = 0; break;
        case SERIAL_5E2: DataSize = CS5; ParityEN = 1; Parity = 0; StopBits = 1; break;
        case SERIAL_6E2: DataSize = CS6; ParityEN = 1; Parity = 0; StopBits = 1; break;
        case SERIAL_7E2: DataSize = CS7; ParityEN = 1; Parity = 0; StopBits = 1; break;
        case SERIAL_8E2: DataSize = CS8; ParityEN = 1; Parity = 0; StopBits = 1; break;
        case SERIAL_5O1: DataSize = CS5; ParityEN = 1; Parity = 1; StopBits = 0; break;
        case SERIAL_6O1: DataSize = CS6; ParityEN = 1; Parity = 1; StopBits = 0; break;
        case SERIAL_7O1: DataSize = CS7; ParityEN = 1; Parity = 1; StopBits = 0; break;
        case SERIAL_8O1: DataSize = CS8; ParityEN = 1; Parity = 1; StopBits = 0; break;
        case SERIAL_5O2: DataSize = CS5; ParityEN = 1; Parity = 1; StopBits = 1; break;
        case SERIAL_6O2: DataSize = CS6; ParityEN = 1; Parity = 1; StopBits = 1; break;
        case SERIAL_7O2: DataSize = CS7; ParityEN = 1; Parity = 1; StopBits = 1; break;
        case SERIAL_8O2: DataSize = CS8; ParityEN = 1; Parity = 1; StopBits = 1; break;
        default: DataSize = CS8; ParityEN = 0; Parity = 0; StopBits = 0; break; // SERIAL_8N1
    }

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &=  ~CSIZE;                                             // Enable set data size
    options.c_cflag |= DataSize;                                            // Data size
    (ParityEN) ? options.c_cflag |= PARENB : options.c_cflag &= ~PARENB;    // Parity enable ? YES : NO
    (Parity) ? options.c_cflag |= PARODD : options.c_cflag &= ~PARODD;      // Parity ? Odd : Even
    (StopBits) ? options.c_cflag |= CSTOPB : options.c_cflag &= ~CSTOPB;    // Stop bits ? 2 bits: 1 bit
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_oflag &= ~OPOST;

	tcsetattr (sd, TCSANOW, &options);
    
}

// Disables serial communication
void SerialPi::end() 
{
    unistd::close(sd);
    sd = -1;
}

// Get the numberof bytes (characters) available for reading from 
// the serial port.
// Return: number of bytes avalable to read 
int SerialPi::available()
{
    int nbytes = 0;
    if (ioctl(sd, FIONREAD, &nbytes) < 0)  {
        fprintf(stderr, "%s(): serial get available bytes error: %s \n",__func__, strerror (errno));
        exit(-1);
    }
    return nbytes;
}

// Arduino uses buffers to send/recieve data and in arduino this
// function returns the available bytes in the tx serial buffer to write to.
// For Piduino we don't use buffers so this function is not necessary. 
// 63 is what you normaly get in an Arduino Uno with an empty tx Serial buffer. 
int SerialPi::availableForWrite ()
{
    return 63;
}

bool SerialPi::find(const char *target)
{
    findUntil(target,NULL);
}

// Reads data from the serial buffer until a target string of given length,
// terminator string is found or times out.
// Returns: true if target string is found, false if times out or terminator is found.
bool SerialPi::findUntil(const char *target, const char *terminator)
{
    timespec time1, time2;
    int index = 0;
    int termIndex = 0;
    int targetLen;
    int termLen;
    char readed;

    if (target == NULL || *target == '\0') {
        return true;   // return true if target is a null string
    } 
    targetLen = strlen(target);

    if (terminator == NULL) {
        termLen = 0;
    } else {
        termLen =  strlen(terminator);
    }

    clock_gettime(CLOCK_REALTIME, &time1);

    do {
        if (available()) {
            unistd::read(sd,&readed,1);
            if (readed != target[index])
            index = 0; // reset index if any char does not match

            if (readed == target[index]) {
                // return true if all chars in the target match
                if (++index >= targetLen) { 
                    return true;
                }
            }

            if (termLen > 0 && readed == terminator[termIndex]) {
                // return false if terminate string found before target string
                if(++termIndex >= termLen) return false; 
            } else { 
                termIndex = 0;
            }
        }

        clock_gettime(CLOCK_REALTIME, &time2);

    } while(timeDiffmillis(time1, time2) < timeOut);

    return false;
}

// Remove any data remaining on the serial buffer
void SerialPi::flush()
{
    tcflush(sd,TCIOFLUSH);
}

// returns the first valid floating point number from the serial buffer.
// initial characters that are not digits (or the minus sign) are skipped
// function is terminated by the first character that is not a digit.
float SerialPi::parseFloat()
{
    bool isNegative = false;
    bool isFraction = false;
    long value = 0;
    int c;
    float fraction = 1.0;

    //Skip characters until a number or - sign found
    c = peekNextDigit(true);
    // ignore non numeric leading characters
    if(c < 0)
        return 0; // zero returned if timeout

    do {
        if(c == '-')
            isNegative = true;
        else if (c == '.')
            isFraction = true;
        else if(c >= '0' && c <= '9') {     // is c a digit?
            value = value * 10 + c - '0';   // get digit number
            if(isFraction)
                fraction *= 0.1;
        }

        getc(sd_file);  // consume the character we got with peek
        c = timedPeek();
    } while( (c >= '0' && c <= '9')  || (c == '.' && !isFraction));

    if (isNegative)
        value = -value;
    if (isFraction)
        return value * fraction;
    else
        return value;
}

// returns the first valid (long) integer value from the current position.
// initial characters that are not digits (or the minus sign) are skipped
// function is terminated by the first character that is not a digit.
long SerialPi::parseInt(char ignore)
{
    bool isNegative = false;
    long value = 0;
    int c;
    char b;

    c = peekNextDigit(false);
    // ignore non numeric leading characters
    if(c < 0)
        return 0; // zero returned if timeout

    do {
        if(c == ignore)
            ; // ignore this character
        else if(c == '-')
            isNegative = true;
        else if(c >= '0' && c <= '9')       // is c a digit?
            value = value * 10 + c - '0';   // get digit number
        
        getc(sd_file);  // consume the character we got with peek
        c = timedPeek();
    } while( (c >= '0' && c <= '9') || c == ignore );

    if(isNegative)
        value = -value;
    return value;
}

// Returns the next byte (character) of incoming serial data
// without removing it from the internal serial buffer.
int SerialPi::peek()
{
    int8_t c;

    // Rewind the file to get the latest data. 
    rewind(sd_file);
    // With a pointer to FILE we can do getc and ungetc
    c = getc(sd_file);
    ungetc(c, sd_file);

    if (c == 0) 
        return -1;
    else 
        return c;
}

//------- PRINTS --------//

// Prints data to the serial port as human-readable ASCII text.
size_t SerialPi::print(const char str[])
{
    return unistd::write(sd,str,strlen(str));
}

// Prints one character to the serial port as human-readable ASCII text.
size_t SerialPi::print(char c)
{
	return unistd::write(sd,&c,1);
}

size_t SerialPi::print(unsigned char b, int base)
{
  return print((unsigned int) b, base);
}

// Prints data to the serial port as human-readable ASCII text.
//  It can print the message in many format representations such as:
// Binary, Octal, Decimal, Hexadecimal and as a BYTE.
size_t SerialPi::print(unsigned int n, int base)
{
    char * message;
    switch(base) {
        case BIN:
            message = int2bin(n);
            break;
        case OCT:
            asprintf(&message,"%o",n);
            break;
        case DEC:
            asprintf(&message,"%d",n);
            break;
        case HEX:
            asprintf(&message,"%X",n);
            break;
        default:
            asprintf(&message,"%d",n);
            break;
    }

    return unistd::write(sd,message,strlen(message));
}

// Prints data to the serial port as human-readable ASCII text.
//  It can print the message in many format representations such as:
// Binary, Octal, Decimal, Hexadecimal and as a BYTE.
size_t SerialPi::print(int n, int base)
{
    char * message;
    switch(base) {
        case BIN:
            message = int2bin(n);
            break;
        case OCT:
            asprintf(&message,"%o",n);
            break;
        case DEC:
            asprintf(&message,"%d",n);
            break;
        case HEX:
            asprintf(&message,"%X",n);
            break;
        default:
            asprintf(&message,"%d",n);
            break;
    }

    return unistd::write(sd,message,strlen(message));
}

// Prints a new line
size_t SerialPi::println(void)
{
    char * msg;
    asprintf(&msg,"\r\n");
    return unistd::write(sd,msg,strlen(msg));
}

// Prints data to the serial port as human-readable ASCII text
// Followed by a new line
size_t SerialPi::println(const char c[])
{
  size_t n = print(c);
  n += println();
  return n;
}

// Prints one character to the serial port as human-readable ASCII text.
// Followed by a new line
size_t SerialPi::println(char c)
{
  size_t n = print(c);
  n += println();
  return n;
}

size_t SerialPi::println(unsigned char b, int base)
{
  size_t n = print(b, base);
  n += println();
  return n;
}

size_t SerialPi::println(int num, int base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t SerialPi::println(unsigned int num, int base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

/*
// Prints data to the serial port as human-readable ASCII text.
// precission is used to limit the number of decimals.
//
 //TODO change precision
void SerialPi::print(float f, int precission)
{
    char * message;
    asprintf(&message, "%.1f", f );
    unistd::write(sd,message,strlen(message));
}

// Prints data to the serial port as human-readable ASCII text followed
// by a carriage retrun character '\r' and a newline character '\n' 
void SerialPi::println(float f, int precission)
{
    const char *str1="%.";
    char * str2;
    char * str3;
    char * message;
    sprintf(str2,"%df",precission);
    asprintf(&str3,"%s%s",str1,str2);
    sprintf(message,str3,f);

    char * msg = NULL;
    asprintf(&msg,"%s\r\n",message);
    unistd::write(sd,msg,strlen(msg));
}
*/

//------- END PRINTS --------//


// Reads 1 byte of incoming serial data
// Returns: first byte of incoming serial data available
int SerialPi::read() 
{
    int8_t c;
    unistd::read(sd,&c,1);
    return c;
}

// Reads characters from th serial port into a buffer. The function 
// terminates if the determined length has been read, or it times out
// Returns: number of bytes readed
size_t SerialPi::readBytes(char buffer[], size_t length)
{
    timespec time1, time2;
    clock_gettime(CLOCK_REALTIME, &time1);
    int count = 0;
    while (count < length) {
        if (available()) {
            unistd::read(sd,&buffer[count],1);
            count ++;
        }
        clock_gettime(CLOCK_REALTIME, &time2);
        if (timeDiffmillis(time1,time2) > timeOut) break;
    }
    return count;
}

// Reads characters from the serial buffer into an array. 
// The function terminates if the terminator character is detected,
// the determined length has been read, or it times out.
// Returns: number of characters read into the buffer.
size_t  SerialPi::readBytesUntil(char terminator, char buffer[], size_t length)
{
    timespec time1, time2;
    clock_gettime(CLOCK_REALTIME, &time1);
    int count = 0;
    char c;
    while (count < length) {
        if (available()) {
            unistd::read(sd,&c,1);
            if (c == terminator) break;
            buffer[count] = c;
            count ++;
        }
        clock_gettime(CLOCK_REALTIME, &time2);
        if (timeDiffmillis(time1,time2) > timeOut) break;
    }
    return count;
}

/* TODO: Implement String first
// Read a string until timeout or null char detected
String Stream::readString()
{
    timespec time1, time2;
    clock_gettime(CLOCK_REALTIME, &time1);
    int count = 0;
    while (count < length) {
        if (available()) {
            unistd::read(sd,&buffer[count],1);
            count ++;
        }
        clock_gettime(CLOCK_REALTIME, &time2);
        if (timeDiffmillis(time1,time2) > timeOut) break;
    }
    return count;
}


// Read a string until timeout, null char detected or terminator detected
String Stream::readStringUntil(char terminator)
{
    timespec time1, time2;
    clock_gettime(CLOCK_REALTIME, &time1);
    int count = 0;
    char c;
    while (count < length) {
        if (available()) {
            unistd::read(sd,&c,1);
            if (c == terminator) break;
            buffer[count] = c;
            count ++;
        }
        clock_gettime(CLOCK_REALTIME, &time2);
        if (timeDiffmillis(time1,time2) > timeOut) break;
    }
    return count;
}
*/

// Sets the maximum milliseconds to wait for serial data when using 
// readBytes(), readBytesUntil(), parseInt(), parseFloat(), findUnitl(), ...
// The default value is set to 1000 
void SerialPi::setTimeout(long millis)
{
    timeOut = millis;
}

// Writes binary data to the serial port. This data is sent as a byte 
// Returns: number of bytes written
size_t SerialPi::write(uint8_t c)
{
	unistd::write(sd,&c,1);
	return 1;
}

// Writes binary data to the serial port. This data is sent as a series
// of bytes
// Returns: number of bytes written
size_t SerialPi::write(const char *str)
{
    if (str == NULL) return 0;
	return unistd::write(sd,str,strlen(str));
}

// Writes binary data to the serial port. This data is sent as a series
// of bytes placed in an buffer. It needs the length of the buffer
// Returns: number of bytes written 
size_t SerialPi::write(char *buffer, size_t size)
{
	return unistd::write(sd,buffer,size);
}


////  Private methods ////

// private method to peek stream with timeout
int SerialPi::timedPeek()
{
    timespec time1, time2;
    int c;
    clock_gettime(CLOCK_REALTIME, &time1);
    do {
        c = peek();
        if (c >= 0) return c;
        clock_gettime(CLOCK_REALTIME, &time2);
    } while(timeDiffmillis(time1, time2) < timeOut);
    return -1;     // -1 indicates timeout
}

// returns peek of the next digit in the stream or -1 if timeout
// discards non-numeric characters
int SerialPi::peekNextDigit(bool detectDecimal)
{
    int c;

    while (1) {
        c = timedPeek();

        if( c < 0 ||
            c == '-' ||
            (c >= '0' && c <= '9') ||
            (detectDecimal && c == '.')) return c;

        getc(sd_file);  // discard non-numeric
    }
}

// Returns the difference of two times in miiliseconds
long SerialPi::timeDiffmillis(timespec start, timespec end)
{
    return (long) ((end.tv_sec - start.tv_sec) * 1e3 + (end.tv_nsec - start.tv_nsec) * 1e-6);
}

// Returns a binary representation of the integer passed as argument
char * SerialPi::int2bin(int n)
{
    size_t bits = sizeof(int) * 8;
    char * str = (char *)malloc(bits + 1);
    unsigned int mask = 1 << (bits-1); //Same as 0x80000000
    int i = 0;

    if (!str) return NULL;
    
    // Convert from integer to binary
    for (i = 0; i < bits; mask >>= 1, i++) {
        str[i]  = n & mask ? '1' : '0';
    }
    str[i] = 0;

    // Remove leading zeros
    i = strspn (str,"0");
    strcpy(str, &str[i]);

    return str;
}




/////////////////////////////////////////////
//          WirePi class (I2C)             //
////////////////////////////////////////////

//// Private methods ///
uint8_t WirePi::rxBuffer[BUFFER_LENGTH];
uint8_t WirePi::rxBufferIndex = 0;
uint8_t WirePi::rxBufferLength = 0;

uint8_t WirePi::txBuffer[BUFFER_LENGTH];
uint8_t WirePi::txBufferIndex = 0;
uint8_t WirePi::txBufferLength = 0;

uint8_t WirePi::transmitting = 0;


int WirePi::i2c_write_bytes(int file, uint8_t *txBuff, size_t numBytes)
{
    int bytes_written = 0;

    if (numBytes == 0) {
        return bytes_written;
    } else {
        bytes_written = unistd::write(file, txBuff, numBytes);
        if ( bytes_written < 0) {
            // errno == 5 (Input/Output error) means I2C cables may not be connected properly.
            // Make noise about everything else except errno == 5. 
            if (errno != 5 ) {
                fprintf(stderr, "%s(): i2c write error: %s \n",__func__, strerror (errno));
            }
        }
    }

    return bytes_written;
}

int WirePi::i2c_read_bytes(int file, uint8_t *rxBuff, size_t numBytes)
{
    int bytes_read = 0;

    if (numBytes == 0) {
        return bytes_read;
    } else {
        bytes_read = unistd::read(file, rxBuff, numBytes);
        if ( bytes_read < 0) {
            // errno == 5 (Input/Output error) means I2C cables may not be connected properly.
            // Make noise about everything else except errno == 5. 
            if (errno != 5 ) {
                fprintf(stderr, "%s(): i2c read error: %s \n",__func__, strerror (errno));
            }
        }
    }

    return bytes_read;
}

////  Public methods ////

//Constructor
WirePi::WirePi()
{
    fd = -1;
}

// Initialize the Wire library
void WirePi::begin()
{

    FILE *fn, *fp;
    char filename[20];
    char path[1024];
    char i2cDevice[32] = "";

    // Process the command below to search for i2c-1 device driver excistance
    fn = popen("/bin/ls /dev/ | /bin/grep i2c-1" , "r");
    if (fn == NULL) {
        fprintf(stderr, "%s(): Failed to run command \"/bin/ls /dev/ | /bin/grep i2c-1\"\n",__func__);
        exit(1);
    }
    
    // Process the command below to search for i2c-x devices drivers excistance, 
    // where x = 0, 1, etc
    fp = popen("/bin/ls /dev/ | /bin/grep i2c-" , "r");
    if (fp == NULL) {
        fprintf(stderr, "%s(): Failed to run command \"/bin/ls /dev/ | /bin/grep i2c-\"\n",__func__);
        exit(1);
    }


    // If i2c-1 exists (RPI main i2c) then set it to open, 
    // else set any other existant i2c-x like i2c-0 for old RPI revisions 
    if (fgets(path, sizeof(path)-1, fn) != NULL) {
        // Set i2c-1 device
        snprintf(i2cDevice, 32, "i2c-1", path);
    } else {
        // Set i2c-x device
        while (fgets(path, sizeof(path)-1, fp) != NULL) {
            snprintf(i2cDevice, 32, "%s", path);
            break;
        }
    }

    // If no I2C device driver is enabled or installed then exit. 
    if(strcmp(i2cDevice,"") == 0) {
        fprintf(stderr, "%s(): Filed to locate any \"i2c-x\" device driver in /dev/. \
            please install or enable a i2c interface in your board \n",__func__);
        exit(1);
    }

    // Open /dev/i2c-x device 
    snprintf(filename, 11, "/dev/%s", i2cDevice);
    fd = open(filename, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "%s(): Error openning I2C channel %s: %s\n",__func__, filename, strerror (errno));
        exit(1);
    }

    rxBufferIndex = 0;
    rxBufferLength = 0;

    txBufferIndex = 0;
    txBufferLength = 0;

}

void WirePi::end()
{
    unistd::close(fd);
    fd = -1;
}

// Initialize the Wire library with a slave address
/*
void WirePi::begin(uint8_t address) 
{
    // TODO, Still reading documentation for linux new I2c slave support
}
*/

uint8_t WirePi::requestFrom(uint8_t address, uint8_t quantity)
{

    if (fd < 0) {
        fprintf(stderr, "%s(): Initialize I2C first with Wire.begin() \n", __func__);
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
void WirePi::beginTransmission(uint8_t address)
{

    if (fd < 0) {
        fprintf(stderr, "%s(): Initialize I2C first with Wire.begin() \n", __func__);
        exit(1);
    }

    if (ioctl(fd, I2C_SLAVE, address) < 0) {
        fprintf(stderr, "%s(): ioctl error: %s\n",
            __func__, strerror (errno));
        exit(1);
    }

    // indicate that we are transmitting
    transmitting = 1;
    // reset tx buffer iterator vars
    txBufferIndex = 0;
    txBufferLength = 0;

}

// Writes data to the I2C, returns bytes written.
size_t WirePi::write(uint8_t data)
{

    if(transmitting) {
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
size_t WirePi::write(const char *str)
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
size_t WirePi::write(uint8_t *data, size_t quantity)
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


int WirePi::available(void)
{
  return rxBufferLength - rxBufferIndex;
}


int WirePi::read(void)
{
  int value = -1;
  
  // get each successive byte on each call
  if (rxBufferIndex < rxBufferLength) {
    value = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
  }

  return value;
}


uint8_t WirePi::endTransmission()
{
    if (fd < 0) {
        fprintf(stderr, "%s(): Initialize I2C first with Wire.begin() \n", __func__);
        exit(1);
    }

    // Transmit Data 
    uint8_t ret = i2c_write_bytes(fd, txBuffer, txBufferLength);

    // reset tx buffer iterator vars
    txBufferIndex = 0;
    txBufferLength = 0;
    // indicate that we are done transmitting
    transmitting = 0;

    return ret;
}




/////////////////////////////////////////////
//          SPIPi class (SPI)             //
////////////////////////////////////////////

SPISettings SPISET;

////  Private methods ////

// Transfers SPI data, recieved data is stored back in the data buffer
void SPIPi::spi_transfer_bytes(int file, uint8_t *data, size_t numBytes) 
{
    struct spi_ioc_transfer spi;
    int ret;

    memset (&spi, 0, sizeof(spi));

    spi.tx_buf        = (unsigned long)data;
    spi.rx_buf        = (unsigned long)data;
    spi.len           = numBytes;

    if (ioctl (file, SPI_IOC_MESSAGE(1), &spi) < 0) {
        fprintf(stderr, "%s(): spi transfer error: %s \n",__func__, strerror (errno));
    }
}

////  Public methods ////


SPIPi::SPIPi()
{
    fd = -1;
}

void SPIPi::begin()
{
    FILE *fp;
    char filename[20];
    char path[1024];
    char spiDevice[32] = "";
    uint8_t bitsPerWord = 8;

    // Process the command below to search for spidev device driver excistance
    fp = popen("/bin/ls /dev/ | /bin/grep spidev" , "r");
    if (fp == NULL) {
        fprintf(stderr, "%s(): Failed to run command \"/bin/ls /dev/ | /bin/grep spidev\"\n",__func__);
        exit(1);
    }


    // If any spidevX.X exits, then set to open it.
    // If there are two or more (e.g spidev0.0 and spidev 0.1) 
    // then the one with the lowest number (spidev0.0) will be set
    while (fgets(path, sizeof(path)-1, fp) != NULL) {
        snprintf(spiDevice, 32, "%s", path);
        break;
    }

    // If no SPI device driver is enabled or installed then exit. 
    if(strcmp(spiDevice,"") == 0) {
        fprintf(stderr, "%s(): Filed to locate any \"spidevX.X\" device driver in /dev/. \
            please install or enable a spi interface in your board \n",__func__);
        exit(1);
    }

    // Open /dev/spidevX.X device 
    snprintf(filename, 15, "/dev/%s", spiDevice);
    fd = open(filename, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "%s(): Error openning SPI channel %s: %s\n",__func__, filename, strerror (errno));
        exit(1);
    }

    // Set bits per word to 8 always
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bitsPerWord) < 0) {
        fprintf(stderr, "%s(): ioctl error: %s\n",
            __func__, strerror (errno));
        exit(1);
    }

}

void SPIPi::end()
{
    unistd::close(fd);
    fd = -1;
}

void SPIPi::beginTransaction(SPISettings settings)
{

    if (fd < 0) {
        fprintf(stderr, "%s(): Initialize SPI first with SPI.begin() \n", __func__);
        exit(1);
    }

    // Set SPI mode (0,1,2,3)
    if (ioctl(fd, SPI_IOC_WR_MODE, &SPISET.spiDataMode) < 0) {
        fprintf(stderr, "%s(): ioctl error: %s\n",
            __func__, strerror (errno));
        exit(1);
    }

    // Set SPI bit order (LSB/MSB)
    if (ioctl(fd, SPI_IOC_WR_LSB_FIRST, &SPISET.spiBitOrder) < 0) {
        fprintf(stderr, "%s(): ioctl error: %s\n",
            __func__, strerror (errno));
        exit(1);
    }

    // Set max hz speed
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &SPISET.spiClock) < 0) {
        fprintf(stderr, "%s(): ioctl error: %s\n",
            __func__, strerror (errno));
        exit(1);
    }

}

void SPIPi::endTransaction()
{
    // Do Nothing
}

void SPIPi::setBitOrder(uint8_t bitOrder)
{
    if (fd < 0) {
        fprintf(stderr, "%s(): Initialize SPI first with SPI.begin() \n", __func__);
        exit(1);
    }

    // Set SPI bit order (LSB/MSB)
    if (ioctl(fd, SPI_IOC_WR_LSB_FIRST, &bitOrder) < 0) {
        fprintf(stderr, "%s(): ioctl error: %s\n",
            __func__, strerror (errno));
        exit(1);
    }
}

void SPIPi::setClockDivider(uint32_t clockDiv)
{
    if (fd < 0) {
        fprintf(stderr, "%s(): Initialize SPI first with SPI.begin() \n", __func__);
        exit(1);
    }

    // Set max hz speed
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &clockDiv) < 0) {
        fprintf(stderr, "%s(): ioctl error: %s\n",
            __func__, strerror (errno));
        exit(1);
    }
}

void SPIPi::setDataMode(uint8_t dataMode)
{
    if (fd < 0) {
        fprintf(stderr, "%s(): Initialize SPI first with SPI.begin() \n", __func__);
        exit(1);
    }

    // Set SPI mode (0,1,2,3)
    if (ioctl(fd, SPI_IOC_WR_MODE, &dataMode) < 0) {
        fprintf(stderr, "%s(): ioctl error: %s\n", __func__, strerror (errno));
        exit(1);
    }
}

uint8_t SPIPi::transfer(uint8_t data)
{
    uint8_t transferData;
    transferData = data;
    spi_transfer_bytes(fd, &transferData, 1);
    return transferData;
}

uint16_t SPIPi::transfer16(uint16_t data)
{
    // TODO
}

void SPIPi::transfer(void *buf, size_t count)
{
   spi_transfer_bytes(fd, (uint8_t *)buf, count);
}




/////////////////////////////////////////////
//          Digital I/O                   //
////////////////////////////////////////////

// BCM2708 Registers for GPIO (Do not put them in .h)
#define BCM2708_PERI_BASE   0x20000000
#define GPIO_BASE           (BCM2708_PERI_BASE + 0x200000)
#define OFFSET_FSEL         0   // 0x0000
#define OFFSET_SET          7   // 0x001c / 4
#define OFFSET_CLR          10  // 0x0028 / 4
#define OFFSET_PINLEVEL     13  // 0x0034 / 4
#define OFFSET_PULLUPDN     37  // 0x0094 / 4
#define OFFSET_PULLUPDNCLK  38  // 0x0098 / 4
#define PAGE_SIZE  (4*1024)
#define BLOCK_SIZE (4*1024)
static volatile uint32_t *gpio_map;
static bool open_gpiomem_flag = false;
char GPIO_DRIVER_NAME[] = "/dev/gpiomem";

// Sets pin (gpio) mode as INPUT/INTPUT_PULLUP/INTPUT_PULLDOWN/OUTPUT
void pinMode(uint8_t pin, uint8_t mode)
{
    int mem_fd;
    uint8_t *gpio_mem;
    int clk_offset = OFFSET_PULLUPDNCLK + (pin/32);
    int shift_offset = (pin%32);
    int offset = OFFSET_FSEL + (pin/10);
    int shift = (pin%10)*3;

    printf("Driver = %s \n", GPIO_DRIVER_NAME);

    // Initialize gpiomem only once
    if (open_gpiomem_flag == false) {
        if ((mem_fd = open(GPIO_DRIVER_NAME, O_RDWR|O_SYNC) ) < 0) {
            fprintf(stderr, "%s(): gpio driver %s: %s\n",__func__, 
                GPIO_DRIVER_NAME, strerror (errno));
            exit(1);
        }

        if ((gpio_mem = (uint8_t *) malloc(BLOCK_SIZE + (PAGE_SIZE-1))) == NULL) {
            fprintf(stderr, "%s(): gpio error: %s\n",__func__, strerror (errno));
            exit(1);
        }

        if ((uint32_t)gpio_mem % PAGE_SIZE) {
            gpio_mem += PAGE_SIZE - ((uint32_t)gpio_mem % PAGE_SIZE);
        }

        gpio_map = (uint32_t *)mmap( (void *)gpio_mem, BLOCK_SIZE, 
            PROT_READ|PROT_WRITE, MAP_SHARED|MAP_FIXED, mem_fd, GPIO_BASE);

        if ((uint32_t)gpio_map < 0) {
            fprintf(stderr, "%s(): gpio error: %s\n",__func__, strerror (errno));
            exit(1);
        }

        // gpiomem initialized correctly
        open_gpiomem_flag = true;
    }


    // Set resistor mode PULLUP, PULLDOWN or PULLOFF resitor (OUTPUT always PULLOFF)
    if (mode == INPUT_PULLDOWN) {
       *(gpio_map+OFFSET_PULLUPDN) = (*(gpio_map+OFFSET_PULLUPDN) & ~3) | 0x01;
    } else if (mode == INPUT_PULLUP) {
       *(gpio_map+OFFSET_PULLUPDN) = (*(gpio_map+OFFSET_PULLUPDN) & ~3) | 0x02;
    } else { // mode == PULLOFF
       *(gpio_map+OFFSET_PULLUPDN) &= ~3;
    }
    unistd::usleep(1);
    *(gpio_map+clk_offset) = 1 << shift_offset;
    unistd::usleep(1);
    *(gpio_map+OFFSET_PULLUPDN) &= ~3;
    *(gpio_map+clk_offset) = 0;

    // Set pin mode INPUT/OUTPUT
    if (mode == OUTPUT) {
        *(gpio_map+offset) = (*(gpio_map+offset) & ~(7<<shift)) | (1<<shift);
    } else { // mode == INPUT or INPUT_PULLUP or INPUT_PULLDOWN
        *(gpio_map+offset) = (*(gpio_map+offset) & ~(7<<shift));
    }

}

// Sets a pin (gpio) output to 1 or 0
void digitalWrite(uint8_t pin, uint8_t val)
{
    int offset;
    if (val) { // value == HIGH
        offset = OFFSET_SET + (pin / 32);
    } else {    // value == LOW
        offset = OFFSET_CLR + (pin / 32);
    }
    *(gpio_map+offset) = 1 << pin % 32;
}

// Returns the value of a pin (gpio) input (1 or 0)
int digitalRead(uint8_t pin)
{
   int offset, value, mask;
   offset = OFFSET_PINLEVEL + (pin/32);
   mask = (1 << pin%32);
   value = *(gpio_map+offset) & mask;
   return (value) ? HIGH : LOW;
}

/////////////////////////////////////////////
//          Analog I/O                    //
////////////////////////////////////////////
/////////////////////////////////////////////
//          Advanced I/O                  //
////////////////////////////////////////////
/*
uint8_t shiftIn(uint8_t dPin, uint8_t cPin, bcm2835SPIBitOrder order){
    uint8_t value = 0 ;
    int8_t  i ;

    if (order == BCM2835_SPI_BIT_ORDER_MSBFIRST )
        for (i = 7 ; i >= 0 ; --i){
            digitalWrite (cPin, HIGH);
            value |= digitalRead (dPin) << i;
            digitalWrite (cPin, LOW);
        }
    else
        for (i = 0 ; i < 8 ; ++i){
          digitalWrite (cPin, HIGH);
          value |= digitalRead (dPin) << i;
          digitalWrite (cPin, LOW);
        }

    return value;
}

void shiftOut(uint8_t dPin, uint8_t cPin, bcm2835SPIBitOrder order, uint8_t val){
    int8_t i;

    if (order == BCM2835_SPI_BIT_ORDER_MSBFIRST )
        for (i = 7 ; i >= 0 ; --i){ 
            digitalWrite (dPin, val & (1 << i)) ;
            digitalWrite (cPin, HIGH) ;
            digitalWrite (cPin, LOW) ;
        }
    else
        for (i = 0 ; i < 8 ; ++i){
            digitalWrite (dPin, val & (1 << i)) ;
            digitalWrite (cPin, HIGH) ;
            digitalWrite (cPin, LOW) ;
        }
}
*/
/////////////////////////////////////////////
//          Time                          //
////////////////////////////////////////////

// Gets a timestamp when the program starts
TimeElapsed::TimeElapsed() {
    clock_gettime(CLOCK_REALTIME, &timestamp);
}

// Returns the time in millseconds since the program started.
unsigned long millis(void) 
{
    struct timespec timenow, start, end;
    clock_gettime(CLOCK_REALTIME, &timenow);
    start = ProgramStart.timestamp;
    end = timenow;
    return ((end.tv_sec - start.tv_sec) * 1e3 + (end.tv_nsec - start.tv_nsec) * 1e-6);
}

// Returns the time in microseconds since the program started.
unsigned long micros(void)
{
    struct timespec timenow, start, end;
    clock_gettime(CLOCK_REALTIME, &timenow);
    start = ProgramStart.timestamp;
    end = timenow;
    return ((end.tv_sec - start.tv_sec) * 1e6 + (end.tv_nsec - start.tv_nsec) * 1e-3);
}

// Sleep the specified milliseconds
void delay(unsigned long millis)
{
    unistd::usleep(millis*1000);
}

// Sleep the specified microseconds
void delayMicroseconds(unsigned int us)
{
    unistd::usleep(us);
}

/////////////////////////////////////////////
//          Math                          //
////////////////////////////////////////////

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/////////////////////////////////////////////
//          Characters                    //
////////////////////////////////////////////

// Checks for an alphanumeric character. 
// It is equivalent to (isalpha(c) || isdigit(c)).
inline boolean isAlphaNumeric(int c) 
{
  return ( isalnum(c) == 0 ? false : true);
}

// Checks for an alphabetic character. 
// It is equivalent to (isupper(c) || islower(c)).
inline boolean isAlpha(int c)
{
  return ( isalpha(c) == 0 ? false : true);
}

// Checks whether c is a 7-bit unsigned char value 
// that fits into the ASCII character set.
inline boolean isAscii(int c)
{
  return ( isascii (c) == 0 ? false : true);
}

// Checks for a blank character, that is, a space or a tab.
inline boolean isWhitespace(int c)
{
  return ( isblank (c) == 0 ? false : true);
}

// Checks for a control character.
inline boolean isControl(int c)
{
  return ( iscntrl (c) == 0 ? false : true);
}

// Checks for a digit (0 through 9).
inline boolean isDigit(int c)
{
  return ( isdigit (c) == 0 ? false : true);
}

// Checks for any printable character except space.
inline boolean isGraph(int c)
{
  return ( isgraph (c) == 0 ? false : true);
}

// Checks for a lower-case character.
inline boolean isLowerCase(int c)
{
  return (islower (c) == 0 ? false : true);
}

// Checks for any printable character including space.
inline boolean isPrintable(int c)
{
  return ( isprint (c) == 0 ? false : true);
}

// Checks for any printable character which is not a space 
// or an alphanumeric character.
inline boolean isPunct(int c)
{
  return ( ispunct (c) == 0 ? false : true);
}

// Checks for white-space characters. For the avr-libc library, 
// these are: space, formfeed ('\f'), newline ('\n'), carriage 
// return ('\r'), horizontal tab ('\t'), and vertical tab ('\v').
inline boolean isSpace(int c)
{
  return ( isspace (c) == 0 ? false : true);
}

// Checks for an uppercase letter.
inline boolean isUpperCase(int c)
{
  return ( isupper (c) == 0 ? false : true);
}

// Checks for a hexadecimal digits, i.e. one of 0 1 2 3 4 5 6 7 
// 8 9 a b c d e f A B C D E F.
inline boolean isHexadecimalDigit(int c)
{
  return ( isxdigit (c) == 0 ? false : true);
}

// Converts c to a 7-bit unsigned char value that fits into the 
// ASCII character set, by clearing the high-order bits.
inline int toAscii(int c)
{
  return toascii (c);
}

// Converts the letter c to lower case, if possible.
inline int toLowerCase(int c)
{
  return tolower (c);
}

// Converts the letter c to upper case, if possible.
inline int toUpperCase(int c)
{
  return toupper (c);
}

/////////////////////////////////////////////
//          Random Functions              //
////////////////////////////////////////////

void randomSeed(unsigned long seed)
{
  if (seed != 0) {
    srandom(seed);
  }
}

long random(long howbig)
{
  if (howbig == 0) {
    return 0;
  }
  return random() % howbig;
}

long random(long howsmall, long howbig)
{
  if (howsmall >= howbig) {
    return howsmall;
  }
  long diff = howbig - howsmall;
  return random(diff) + howsmall;
}

/////////////////////////////////////////////
//          External Interrupts           //
////////////////////////////////////////////
/*
void attachInterrupt(int p,void (*f)(), Digivalue m){
    int GPIOPin = raspberryPinNumber(p);
    pthread_t *threadId = getThreadIdFromPin(p);
    struct ThreadArg *threadArgs = (ThreadArg *)malloc(sizeof(ThreadArg));
    threadArgs->func = f;
    threadArgs->pin = GPIOPin;
    
    //Export pin for interrupt
    FILE *fp = fopen("/sys/class/gpio/export","w");
    if (fp == NULL){
        fprintf(stderr,"Unable to export pin %d for interrupt\n",p);
        exit(1);
    }else{
        fprintf(fp,"%d",GPIOPin); 
    }
    fclose(fp);
    
    //The system to create the file /sys/class/gpio/gpio<GPIO number>
    //So we wait a bit
    delay(1L);
    
    char * interruptFile = NULL;
    asprintf(&interruptFile, "/sys/class/gpio/gpio%d/edge",GPIOPin);
    
    //Set detection condition
    fp = fopen(interruptFile,"w");
    if (fp == NULL){
        fprintf(stderr,"Unable to set detection type on pin %d\n",p);
        exit(1);
    }else{
        switch(m){
            case RISING: fprintf(fp,"rising");break;
            case FALLING: fprintf(fp,"falling");break;
            default: fprintf(fp,"both");break;
        }
        
    }
    fclose(fp);
    
    if(*threadId == 0){
        //Create a thread passing the pin and function
        pthread_create (threadId, NULL, threadFunction, (void *)threadArgs);
    }else{
        //First cancel the existing thread for that pin
        pthread_cancel(*threadId);
        //Create a thread passing the pin, function and mode
        pthread_create (threadId, NULL, threadFunction, (void *)threadArgs);
    }
    
}

void detachInterrupt(int p){
    int GPIOPin = raspberryPinNumber(p);
    
    FILE *fp = fopen("/sys/class/gpio/unexport","w");
    if (fp == NULL){
        fprintf(stderr,"Unable to unexport pin %d for interrupt\n",p);
        exit(1);
    }else{
        fprintf(fp,"%d",GPIOPin); 
    }
    fclose(fp);
    
    pthread_t *threadId = getThreadIdFromPin(p);
    pthread_cancel(*threadId);
}
*/

/////////////////////////////////////////////
//          Interrupts                    //
////////////////////////////////////////////



/*
void analogWrite(int pin, int value) {
    auto digitalVal = value > 0 ? HIGH : LOW;
    digitalWrite(pin, digitalVal);
}


int analogRead (int pin){

	int value = 0;
	return value;
}




//// Some helper functions ////

int getBoardRev(){
	
	FILE *cpu_info;
	char line [120];
	char *c,finalChar;
	static int rev = 0;
	
	if (REV != 0) return REV;
	
	if ((cpu_info = fopen("/proc/cpuinfo","r"))==NULL){
		fprintf(stderr,"Unable to open /proc/cpuinfo. Cannot determine board reivision.\n");
		exit(1);
	}
	
	while (fgets (line,120,cpu_info) != NULL){
		if(strncmp(line,"Revision",8) == 0) break;
	}
	
	fclose(cpu_info);
	
	if (line == NULL){
		fprintf (stderr, "Unable to determine board revision from /proc/cpuinfo.\n") ;
		exit(1);
	}
	
	for (c = line ; *c ; ++c)
    if (isdigit (*c))
      break ;

	if (!isdigit (*c)){
		fprintf (stderr, "Unable to determine board revision from /proc/cpuinfo\n") ;
		fprintf (stderr, "  (Info not found in: %s\n", line) ;
		exit(1);
	}
	
	finalChar = c [strlen (c) - 2] ;
	
	if ((finalChar == '2') || (finalChar == '3')){
		bsc0 = bsc_rev1;
		return 1;
	}else{
		bsc0 = bsc_rev2;
		return 2;
	}
}

uint32_t* mapmem(const char *msg, size_t size, int fd, off_t off)
{
    uint32_t *map = (uint32_t *)mmap(NULL, size, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, off);
    if (MAP_FAILED == map)
	fprintf(stderr, "bcm2835_init: %s mmap failed: %s\n", msg, strerror(errno));
    return map;
}

int raspberryPinNumber(int arduinoPin){
     return arduinoPin;
}

pthread_t *getThreadIdFromPin(int pin){
	switch(pin){
		case 2: return &idThread2; break;
		case 3: return &idThread3; break;
		case 4: return &idThread4; break;
		case 5: return &idThread5; break;
		case 6: return &idThread6; break;
		case 7: return &idThread7; break;
		case 8: return &idThread8; break;
		case 9: return &idThread9; break;
		case 10: return &idThread10; break;
		case 11: return &idThread11; break;
		case 12: return &idThread12; break;
		case 13: return &idThread13; break;
	}
}

// This is the function that will be running in a thread if
// attachInterrupt() is called 
void * threadFunction(void *args){
	ThreadArg *arguments = (ThreadArg *)args;
	int pin = arguments->pin;
	
	int GPIO_FN_MAXLEN = 32;
	int RDBUF_LEN = 5;
	
	char fn[GPIO_FN_MAXLEN];
	int fd,ret;
	struct pollfd pfd;
	char rdbuf [RDBUF_LEN];
	
	memset(rdbuf, 0x00, RDBUF_LEN);
	memset(fn,0x00,GPIO_FN_MAXLEN);
	
	snprintf(fn, GPIO_FN_MAXLEN-1, "/sys/class/gpio/gpio%d/value",pin);
	fd=open(fn, O_RDONLY);
	if(fd<0){
		perror(fn);
		exit(1);
	}
	pfd.fd=fd;
	pfd.events=POLLPRI;
	
	ret=unistd::read(fd,rdbuf,RDBUF_LEN-1);
	if(ret<0){
		perror("Error reading interrupt file\n");
		exit(1);
	}
	
	while(1){
		memset(rdbuf, 0x00, RDBUF_LEN);
		unistd::lseek(fd, 0, SEEK_SET);
		ret=poll(&pfd, 1, -1);
		if(ret<0){
			perror("Error waiting for interrupt\n");
			unistd::close(fd);
			exit(1);
		}
		if(ret==0){
			printf("Timeout\n");
			continue;
		}
		ret=unistd::read(fd,rdbuf,RDBUF_LEN-1);
		if(ret<0){
			perror("Error reading interrupt file\n");
			exit(1);
		}
		//Interrupt. We call user function.
		arguments->func();
	}
}

*/

TimeElapsed ProgramStart = TimeElapsed();
SerialPi Serial = SerialPi();
WirePi Wire = WirePi();
SPIPi SPI = SPIPi();
