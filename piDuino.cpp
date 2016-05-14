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
int SerialPi::timeDiffmillis(timespec start, timespec end)
{
    return (int) ((end.tv_sec - start.tv_sec) * 1e3 + (end.tv_nsec - start.tv_nsec) * 1e-6);
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
        fprintf(stderr, "%s(): ioctl error: %s\n",
            __func__, strerror (errno));
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


/********** FUNCTIONS OUTSIDE CLASSES **********/

// Sleep the specified milliseconds
void delay(long millis)
{
	unistd::usleep(millis*1000);
}

void delayMicroseconds(long micros)
{
    if (micros > 100){
        struct timespec tim, tim2;
        tim.tv_sec = 0;
        tim.tv_nsec = micros * 1000;
        
        if(nanosleep(&tim , &tim2) < 0 )   {
            fprintf(stderr,"Nano sleep system call failed \n");
            exit(1);
        }
    }else{
        struct timeval tNow, tLong, tEnd ;
        
        gettimeofday (&tNow, NULL) ;
        tLong.tv_sec  = micros / 1000000 ;
        tLong.tv_usec = micros % 1000000 ;
        timeradd (&tNow, &tLong, &tEnd) ;
        
        while (timercmp (&tNow, &tEnd, <))
            gettimeofday (&tNow, NULL) ;
    }
}

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

// Configures the specified pin to behave either as an input or an output
void pinMode(int pin, Pinmode mode){
	pin = raspberryPinNumber(pin);
	if(mode == OUTPUT){
		switch(pin){
			case 4:  GPFSEL0 &= ~(7 << 12); GPFSEL0 |= (1 << 12); break;
			case 8:  GPFSEL0 &= ~(7 << 24); GPFSEL0 |= (1 << 24); break;
			case 9:  GPFSEL0 &= ~(7 << 27); GPFSEL0 |= (1 << 27); break;
			case 10: GPFSEL1 &= ~(7 << 0); 	GPFSEL1 |= (1 << 0);  break;
			case 11: GPFSEL1 &= ~(7 << 3);  GPFSEL1 |= (1 << 3);  break;
			case 14: GPFSEL1 &= ~(7 << 12); GPFSEL1 |= (1 << 12); break;
			case 17: GPFSEL1 &= ~(7 << 21); GPFSEL1 |= (1 << 21); break;
			case 18: GPFSEL1 &= ~(7 << 24); GPFSEL1 |= (1 << 24); break;
			case 21: GPFSEL2 &= ~(7 << 3);  GPFSEL2 |= (1 << 3);  break;
			case 27: GPFSEL2 &= ~(7 << 21); GPFSEL2 |= (1 << 21); break;
			case 22: GPFSEL2 &= ~(7 << 6);  GPFSEL2 |= (1 << 6);  break;
			case 23: GPFSEL2 &= ~(7 << 9);  GPFSEL2 |= (1 << 9);  break;
			case 24: GPFSEL2 &= ~(7 << 12); GPFSEL2 |= (1 << 12); break;
			case 25: GPFSEL2 &= ~(7 << 15); GPFSEL2 |= (1 << 15); break;
		}

	}else if (mode == INPUT){
		switch(pin){
			case 4:  GPFSEL0 &= ~(7 << 12); break;
			case 8:  GPFSEL0 &= ~(7 << 24); break;
			case 9:  GPFSEL0 &= ~(7 << 27); break;
			case 10: GPFSEL1 &= ~(7 << 0);  break;
			case 11: GPFSEL1 &= ~(7 << 3);  break;	
			case 14: GPFSEL1 &= ~(7 << 12);  break;	
            case 17: GPFSEL1 &= ~(7 << 21); break;
			case 18: GPFSEL1 &= ~(7 << 24); break;
			case 21: GPFSEL2 &= ~(7 << 3);  break;
			case 27: GPFSEL2 &= ~(7 << 3);  break;
			case 22: GPFSEL2 &= ~(7 << 6);  break;
			case 23: GPFSEL2 &= ~(7 << 9);  break;
			case 24: GPFSEL2 &= ~(7 << 12); break;
			case 25: GPFSEL2 &= ~(7 << 15); break;
		}
	}
}

void analogWrite(int pin, int value) {
    auto digitalVal = value > 0 ? HIGH : LOW;
    digitalWrite(pin, digitalVal);
}

// Write a HIGH or a LOW value to a digital pin
void digitalWrite(int pin, int value){
	pin = raspberryPinNumber(pin);
	if (value == HIGH){
		switch(pin){
			case  4:GPSET0 =  BIT_4;break;
			case  8:GPSET0 =  BIT_8;break;
			case  9:GPSET0 =  BIT_9;break;
			case 10:GPSET0 = BIT_10;break;
			case 11:GPSET0 = BIT_11;break;
			case 14:GPSET0 = BIT_14;break;
			case 17:GPSET0 = BIT_17;break;
			case 18:GPSET0 = BIT_18;break;
			case 21:GPSET0 = BIT_21;break;
			case 27:GPSET0 = BIT_27;break;
			case 22:GPSET0 = BIT_22;break;
			case 23:GPSET0 = BIT_23;break;
			case 24:GPSET0 = BIT_24;break;
			case 25:GPSET0 = BIT_25;break;
		}
	}else if(value == LOW){
		switch(pin){
			case  4:GPCLR0 =  BIT_4;break;
			case  8:GPCLR0 =  BIT_8;break;
			case  9:GPCLR0 =  BIT_9;break;
			case 10:GPCLR0 = BIT_10;break;
			case 11:GPCLR0 = BIT_11;break;
			case 14:GPCLR0 = BIT_14;break;
			case 17:GPCLR0 = BIT_17;break;
			case 18:GPCLR0 = BIT_18;break;
			case 21:GPCLR0 = BIT_21;break;
			case 27:GPCLR0 = BIT_27;break;
			case 22:GPCLR0 = BIT_22;break;
			case 23:GPCLR0 = BIT_23;break;
			case 24:GPCLR0 = BIT_24;break;
			case 25:GPCLR0 = BIT_25;break;
		}
	}
    
    delayMicroseconds(1);
    // Delay to allow any change in state to be reflected in the LEVn, register bit.
}

//Soft digitalWrite to avoid spureous Reset in Socket Power ON
void digitalWriteSoft(int pin, int value)
{
  uint frame[32];

  for(int z=0;z<7;z++)
  {
    uint V = 0xFFFFFFFF;

    for(int v=0;v<32;v++)
    {
      uint T = 0xFFFFFFFF;

      V = V << 1;
      for (int t=0; t<32;t++)
      {
        if (value==HIGH)
        {  
          if (T > V) frame[t]=HIGH;
          else frame [t]=LOW;
        }
        else
        {  
          if (T > V) frame[t]=LOW;
          else frame [t]=HIGH;
        }
        T = T << 1;
      }

      for (int i=0; i<32; i++)
      {
        digitalWrite(pin,frame[i]);
      }
    }
  }

  if (value==HIGH) digitalWrite(pin,HIGH);
  else digitalWrite(pin,LOW);
}

// Reads the value from a specified digital pin, either HIGH or LOW.
int digitalRead(int pin){
	Digivalue value;
	pin = raspberryPinNumber(pin);
	switch(pin){
		case 4: if(GPLEV0 & BIT_4){value = HIGH;} else{value = LOW;};break;
		case 8: if(GPLEV0 & BIT_8){value = HIGH;} else{value = LOW;};break;
		case 9: if(GPLEV0 & BIT_9){value = HIGH;} else{value = LOW;};break;
		case 10:if(GPLEV0 & BIT_10){value = HIGH;} else{value = LOW;};break;
		case 11:if(GPLEV0 & BIT_11){value = HIGH;} else{value = LOW;};break;
		case 17:if(GPLEV0 & BIT_17){value = HIGH;}else{value = LOW;};break;
		case 18:if(GPLEV0 & BIT_18){value = HIGH;}else{value = LOW;};break;
		case 21:if(GPLEV0 & BIT_21){value = HIGH;}else{value = LOW;};break;
		case 27:if(GPLEV0 & BIT_27){value = HIGH;}else{value = LOW;};break;
		case 22:if(GPLEV0 & BIT_22){value = HIGH;}else{value = LOW;};break;
		case 23:if(GPLEV0 & BIT_23){value = HIGH;}else{value = LOW;};break;
		case 24:if(GPLEV0 & BIT_24){value = HIGH;}else{value = LOW;};break;
		case 25:if(GPLEV0 & BIT_25){value = HIGH;}else{value = LOW;};break;
	}
	return value;
}

int analogRead (int pin){

	int value = 0;
	return value;
}

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

long millis(){
	long elapsedTime;
	// stop timer
    gettimeofday(&end_point, NULL);

    // compute and print the elapsed time in millisec
    elapsedTime = (end_point.tv_sec - start_program.tv_sec) * 1000.0;      // sec to ms
    elapsedTime += (end_point.tv_usec - start_program.tv_usec) / 1000.0;   // us to ms
    return elapsedTime;
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

SerialPi Serial = SerialPi();
WirePi Wire = WirePi();
SPIPi SPI = SPIPi();
