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

    ### BCM283x Notes for GPIO and PWM
    
    - BCM283x  has a maximum of 54 GPIOS
    - The BCM2835 contains 2 independent PWM channels (0 and 1) that uses
    the same PWM clock as the base frequency. The following are the
    available PWM pins for this chip:
    GPIO PIN   RPi2 pin   PWM Channel  ALT FUN
    12         YES        0            0
    13         YES        1            0
    18         YES        0            5
    19         YES        1            5
    40                    0            0
    41                    1            0
    45                    1            0
    52                    0            1
    53                    1            1
*/


#include "piDuino.h"


/////////////////////////////////////////////
//          SerialPi class (UART)         //
////////////////////////////////////////////

char SERIAL_DRIVER_NAME[] = "/dev/ttyAMA0";

////  Public methods ////

//Constructor
SerialPi::SerialPi()
{
    // Default serial driver and timeout
    timeOut = 1000;
    fd = -1;
    fd_file = NULL;
}

void SerialPi::begin(int baud, unsigned char config)
{
     begin((const char *)SERIAL_DRIVER_NAME, baud, config);
}

// Sets the data rate in bits per second (baud) for serial data transmission
void SerialPi::begin(const char *serialPort, int baud, unsigned char config)
{

    int speed;
    int DataSize, ParityEN, Parity, StopBits;
    struct termios options;
    int flags;

    // Open Serial port 
    if ((fd = open(serialPort, O_RDWR | O_NOCTTY | O_NONBLOCK)) == -1) {
        fprintf(stderr,"%s(): Unable to open the serial port %s: %s\n",
            __func__, serialPort, strerror (errno));
        exit(1);
    }

    // We obtain a pointer to FILE structure (fd_file) from the file descriptor fd
    // and set it to be non-blocking
    fd_file = fdopen(fd,"r+");
    flags = fcntl( fileno(fd_file), F_GETFL );
    fcntl(fileno(fd_file), F_SETFL, flags | O_NONBLOCK);
    

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

    tcgetattr(fd, &options);
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

    tcsetattr (fd, TCSANOW, &options);
    
}

// Disables serial communication
void SerialPi::end() 
{
    unistd::close(fd);
    fd = -1;
}

// Get the numberof bytes (characters) available for reading from 
// the serial port.
// Return: number of bytes avalable to read 
int SerialPi::available()
{
    int nbytes = 0;
    if (ioctl(fd, FIONREAD, &nbytes) < 0)  {
        fprintf(stderr, "%s(): serial get available bytes error: %s \n",
            __func__, strerror (errno));
        exit(1);
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
            unistd::read(fd,&readed,1);
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
    tcflush(fd,TCIOFLUSH);
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

        getc(fd_file);  // consume the character we got with peek
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
        
        getc(fd_file);  // consume the character we got with peek
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
    rewind(fd_file);
    // With a pointer to FILE we can do getc and ungetc
    c = getc(fd_file);
    ungetc(c, fd_file);

    if (c == 0) 
        return -1;
    else 
        return c;
}

//------- PRINTS --------//

// Prints data to the serial port as human-readable ASCII text.
size_t SerialPi::print(const String &s)
{
    unistd::write(fd,s.c_str(), s.length());
}

// Prints data to the serial port as human-readable ASCII text.
size_t SerialPi::print(const char str[])
{
    return unistd::write(fd,str,strlen(str));
}

// Prints one character to the serial port as human-readable ASCII text.
size_t SerialPi::print(char c)
{
    return unistd::write(fd,&c,1);
}

size_t SerialPi::print(unsigned char b, int base)
{
  return print((unsigned int) b, base);
}

// Prints data to the serial port as human-readable ASCII text.
// It can print the message in many format representations such as:
// Binary, Octal, Decimal and Hexadecimal.
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

    return unistd::write(fd,message,strlen(message));
}

// Prints data to the serial port as human-readable ASCII text.
// It can print the message in many format representations such as:
// Binary, Octal, Decimal and Hexadecimal.
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

    return unistd::write(fd,message,strlen(message));
}

// Prints a new line
size_t SerialPi::println(void)
{
    char * msg;
    asprintf(&msg,"\r\n");
    return unistd::write(fd,msg,strlen(msg));
}

// Prints data to the serial port as human-readable ASCII text
// Followed by a new line
size_t SerialPi::println(const String &s)
{
    size_t n = print(s);
    n += println();
    return n;
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

// Prints like a normal C language printf() but to the Serial port
size_t SerialPi::printf(const char *fmt, ... ) {
    char *buf = NULL; 
    va_list args;

    // Copy arguments to buf
    va_start (args, fmt);
    vasprintf(&buf, (const char *)fmt, args);
    va_end (args);

    return unistd::write(fd,buf,strlen(buf));
}


//------- END PRINTS --------//


// Reads 1 byte of incoming serial data
// Returns: first byte of incoming serial data available
int SerialPi::read() 
{
    int8_t c;
    unistd::read(fd,&c,1);
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
            unistd::read(fd,&buffer[count],1);
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
size_t SerialPi::readBytesUntil(char terminator, char buffer[], size_t length)
{
    timespec time1, time2;
    clock_gettime(CLOCK_REALTIME, &time1);
    int count = 0;
    char c;
    while (count < length) {
        if (available()) {
            unistd::read(fd,&c,1);
            if (c == terminator) break;
            buffer[count] = c;
            count ++;
        }
        clock_gettime(CLOCK_REALTIME, &time2);
        if (timeDiffmillis(time1,time2) > timeOut) break;
    }
    return count;
}

// Read a string until timeout
String SerialPi::readString()
{
    String ret = "";
    timespec time1, time2;
    char c = 0;
    clock_gettime(CLOCK_REALTIME, &time1);
    do {
        if (available()) {
            unistd::read(fd,&c,1);
            ret += (char)c;
        }
        clock_gettime(CLOCK_REALTIME, &time2);
        if (timeDiffmillis(time1,time2) > timeOut) break;
    } while (c >= 0);
    return ret;
}


// Read a string until timeout or terminator is detected
String SerialPi::readStringUntil(char terminator)
{
    String ret = "";
    timespec time1, time2;
    char c = 0;
    clock_gettime(CLOCK_REALTIME, &time1);
    do {
        if (available()) {
            unistd::read(fd,&c,1);
            ret += (char)c;
        }
        clock_gettime(CLOCK_REALTIME, &time2);
        if (timeDiffmillis(time1,time2) > timeOut) break;
    } while (c >= 0 && c != terminator);
    return ret;
}

// Reads a string unitl a termintor is given, this function blocks until the terminator is found.
// Terminator character is not added to the char array and last character of the array
// is always terminated with a null ('\0') character
size_t SerialPi::readStringCommand(char terminator, char buffer[], size_t length)
{
    int count = 0;
    char c;
    if (length <= 0) return 0;
    while (count < length) {
        if (available()) {
            unistd::read(fd,&c,1);
            if (c == terminator) break;
            buffer[count] = c;
            count ++;
        }
    }
    buffer[length-1] = '\0';
    return count;
}

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
    unistd::write(fd,&c,1);
    return 1;
}

// Writes binary data to the serial port. This data is sent as a series
// of bytes
// Returns: number of bytes written
size_t SerialPi::write(const char *str)
{
    if (str == NULL) return 0;
    return unistd::write(fd,str,strlen(str));
}

// Writes binary data to the serial port. This data is sent as a series
// of bytes placed in an buffer. It needs the length of the buffer
// Returns: number of bytes written 
size_t SerialPi::write(char *buffer, size_t size)
{
    return unistd::write(fd,buffer,size);
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

        getc(fd_file);  // discard non-numeric
    }
}

// Returns the difference of two times in miiliseconds
long SerialPi::timeDiffmillis(timespec start, timespec end)
{
    return ((end.tv_sec - start.tv_sec) * 1e3 + (end.tv_nsec - start.tv_nsec) * 1e-6);
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

char I2C_DRIVER_NAME[] = ""; // "" means search for any I2C device


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

int WirePi::i2c_read_bytes(int file, uint8_t *rxBuff, size_t numBytes)
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
WirePi::WirePi()
{
    fd = -1;
}

void WirePi::begin()
{
    begin(I2C_DRIVER_NAME);
}

// Initialize the Wire library
void WirePi::begin(const char *i2cDeviceName)
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
            asprintf(&tmpi2cDevice, "i2c-1", path);
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
void WirePi::beginTransmission(uint8_t address)
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
size_t WirePi::write(uint8_t data)
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
        fprintf(stderr, "%s(): initialize I2C first with Wire.begin() \n", __func__);
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

char SPI_DRIVER_NAME[] = ""; // "" means search for any SPI device
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
        fprintf(stderr, "%s(): spi transfer error: %s \n", __func__, strerror (errno));
    }
}

////  Public methods ////


SPIPi::SPIPi()
{
    fd = -1;
}

void SPIPi::begin()
{
    begin(SPI_DRIVER_NAME);
}

void SPIPi::begin(const char *spiDeviceName)
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

void SPIPi::end()
{
    unistd::close(fd);
    fd = -1;
}

void SPIPi::beginTransaction(SPISettings settings)
{

    if (fd < 0) {
        fprintf(stderr, "%s(): initialize SPI first with SPI.begin() \n", __func__);
        exit(1);
    }

    // Set SPI mode (0,1,2,3)
    if (ioctl(fd, SPI_IOC_WR_MODE, &SPISET.spiDataMode) < 0) {
        fprintf(stderr, "%s(): ioctl error: %s\n", __func__, strerror (errno));
        exit(1);
    }

    // Set SPI bit order (LSB/MSB)
    if (ioctl(fd, SPI_IOC_WR_LSB_FIRST, &SPISET.spiBitOrder) < 0) {
        fprintf(stderr, "%s(): ioctl error: %s\n", __func__, strerror (errno));
        exit(1);
    }

    // Set max hz speed
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &SPISET.spiClock) < 0) {
        fprintf(stderr, "%s(): ioctl error: %s\n", __func__, strerror (errno));
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
        fprintf(stderr, "%s(): initialize SPI first with SPI.begin() \n", __func__);
        exit(1);
    }

    // Set SPI bit order (LSB/MSB)
    if (ioctl(fd, SPI_IOC_WR_LSB_FIRST, &bitOrder) < 0) {
        fprintf(stderr, "%s(): ioctl error: %s\n", __func__, strerror (errno));
        exit(1);
    }
}

void SPIPi::setClockDivider(uint32_t clockDiv)
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

void SPIPi::setDataMode(uint8_t dataMode)
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

// -- Digital I/O --
// BCM2708 Registers for GPIO (Do not put them in .h)
#define BCM2708_PERI_BASE   0x20000000
#define GPIO_BASE           (BCM2708_PERI_BASE + 0x200000)
#define OFFSET_FSEL         0   // 0x0000
#define OFFSET_SET          7   // 0x001c / 4
#define OFFSET_CLR          10  // 0x0028 / 4
#define OFFSET_PINLEVEL     13  // 0x0034 / 4
#define OFFSET_PULLUPDN     37  // 0x0094 / 4
#define OFFSET_PULLUPDNCLK  38  // 0x0098 / 4
#define GPIO_FSEL_INPUT     0   // Pin Input mode
#define GPIO_FSEL_OUTPUT    1   // Pin Output mode
#define PAGE_SIZE  (4*1024)
#define BLOCK_SIZE (4*1024)
static volatile uint32_t *gpio_map = NULL;
static bool g_open_gpiomem_flag = false;
int g_gpio_pin_set[SOC_GPIO_PINS];           // Used to know which gpio pins are set (HIGH) or not set (LOW) 
char GPIO_DRIVER_NAME[] = "/dev/gpiomem";

// -- Analog I/O --
#define BCM2708_PERI_BASE   0x20000000
#define PWM_BASE           (BCM2708_PERI_BASE + 0x20C000)
#define CLOCK_BASE         (BCM2708_PERI_BASE + 0x101000)
#define PWMCLK_CNTL         40
#define PWMCLK_DIV          41
#define PWM_CONTROL         0
#define PWM0_RANGE          4
#define PWM0_DATA           5
#define PWM1_RANGE          8
#define PWM1_DATA           9
#define GPIO_FSEL_ALT0      4  
#define GPIO_FSEL_ALT1      5  
#define GPIO_FSEL_ALT5      2  
static volatile uint32_t *pwm_map = NULL;
static volatile uint32_t *clk_map = NULL;
static bool g_open_pwmmem_flag = false;
int g_pwm_pin_set[SOC_GPIO_PINS];               // Used to know which pwm pins are set (HIGH) or not set (LOW)
int g_pwm_dutycycle_value[SOC_GPIO_PINS];       // Pwm duty cycle value of pwm pins
int PWM_DUTYCYCLE_RESOLUTION = 256;             // Set pwm duty cycle resolution to 256 buts
int PWM_DEFAULT_FREQUENCY = 490;                // Set default pwm frequency to 490 Hz (Arduino default pwm freq)
char PWM_DRIVER_NAME[] = "/dev/mem";

// Sets pin (gpio) mode as INPUT,INTPUT_PULLUP,INTPUT_PULLDOWN,OUTPUT,PWM_OUTPUT
void pinMode(uint8_t pin, uint8_t mode)
{
    int mem_fd, pwm_mem_fd;
    uint8_t *gpio_mem;
    int clk_offset = OFFSET_PULLUPDNCLK + (pin/32);
    int shift_offset = (pin%32);
    int offset = OFFSET_FSEL + (pin/10);
    int shift = (pin%10)*3;
    int gpio_fsel_alt = 0;
    int pwm_channel = 0;
    int divisor;
    double period;
    double countDuration;


    // Check if the pin number is valid
    if (pin >= SOC_GPIO_PINS) {
        fprintf(stderr, "%s(): pin number should be less than "
            "%d, yours is %d \n", __func__, SOC_GPIO_PINS, pin);
        exit(1);
    }

    // Initialize gpiomem only once
    if (g_open_gpiomem_flag == false) {
        
        if ((mem_fd = open(GPIO_DRIVER_NAME, O_RDWR|O_SYNC) ) < 0) {
            fprintf(stderr, "%s(): gpio driver %s: %s\n",__func__, 
                GPIO_DRIVER_NAME, strerror (errno));
            exit(1);
        }

        gpio_map = (uint32_t *)mmap( NULL, BLOCK_SIZE, 
            PROT_READ|PROT_WRITE|PROT_EXEC, MAP_SHARED|MAP_LOCKED, mem_fd, GPIO_BASE);

        if ((uint32_t)gpio_map < 0) {
            fprintf(stderr, "%s(): gpio error: %s\n",__func__, strerror (errno));
            exit(1);
        }

        // gpio memory mapping initialized correctly
        g_open_gpiomem_flag = true;
    }

    // Initialize different pin mode options
    if (mode == INPUT || mode == INPUT_PULLUP 
        || mode == INPUT_PULLDOWN || mode == OUTPUT) {

        // Save gpio pin number so at program close we put it to default state
        // Also clear pwm pin to prevent any errro if difenrent pin mode is set multiple times
        g_gpio_pin_set[pin] = HIGH;
        g_pwm_pin_set[pin] = LOW;

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
            *(gpio_map+offset) = (*(gpio_map+offset) & ~(7<<shift)) | (GPIO_FSEL_OUTPUT<<shift);
        } else { // mode == INPUT or INPUT_PULLUP or INPUT_PULLDOWN
            *(gpio_map+offset) = (*(gpio_map+offset) & ~(7<<shift)) | (GPIO_FSEL_INPUT<<shift);
        }

    } else if(mode == PWM_OUTPUT) {

        // Check if the pin is compatible for PWM and assign its channel
        switch (pin) {
            case 12: pwm_channel = 0; gpio_fsel_alt = GPIO_FSEL_ALT0; break;
            case 13: pwm_channel = 1; gpio_fsel_alt = GPIO_FSEL_ALT0; break;
            case 18: pwm_channel = 0; gpio_fsel_alt = GPIO_FSEL_ALT5; break;
            case 19: pwm_channel = 1; gpio_fsel_alt = GPIO_FSEL_ALT5; break;
            case 40: pwm_channel = 0; gpio_fsel_alt = GPIO_FSEL_ALT0; break;
            case 41: pwm_channel = 1; gpio_fsel_alt = GPIO_FSEL_ALT0; break;
            case 45: pwm_channel = 1; gpio_fsel_alt = GPIO_FSEL_ALT0; break;
            case 52: pwm_channel = 0; gpio_fsel_alt = GPIO_FSEL_ALT1; break;
            case 53: pwm_channel = 1; gpio_fsel_alt = GPIO_FSEL_ALT1; break;
            default:
                fprintf(stderr, "%s(): pin %d can not be set as PWM_OUTPUT\n", __func__, pin);
                exit(1);
                break;
        }

        // Initialize mem only once (this requires sudo)
        if (g_open_pwmmem_flag == false) {

            if ((pwm_mem_fd = open(PWM_DRIVER_NAME, O_RDWR|O_SYNC) ) < 0) {
                fprintf(stderr, "%s(): pwm driver %s: %s\n",__func__, 
                    PWM_DRIVER_NAME, strerror (errno));
                exit(1);
            }

            pwm_map = (uint32_t *)mmap(NULL, BLOCK_SIZE, 
                PROT_READ|PROT_WRITE|PROT_EXEC, MAP_SHARED|MAP_LOCKED, pwm_mem_fd, PWM_BASE);

            if ((uint32_t)pwm_map < 0) {
                fprintf(stderr, "%s(): pwm error: %s\n", __func__, strerror (errno));
                exit(1);
            }

            clk_map = (uint32_t *)mmap(NULL, BLOCK_SIZE, 
                PROT_READ|PROT_WRITE|PROT_EXEC, MAP_SHARED|MAP_LOCKED, pwm_mem_fd, CLOCK_BASE);
            
            if ((uint32_t)clk_map < 0) {
                fprintf(stderr, "%s(): pwm error: %s\n", __func__, strerror (errno));
                exit(1);
            }

            // pwm memory mapping initialized correctly
            g_open_pwmmem_flag = true;
        }

        // Save pwm pin number so at program close we put it to default state
        // Also clear gpio pin to prevent any errro if diferent pin mode is set multiple times
        g_pwm_pin_set[pin] = HIGH;
        g_gpio_pin_set[pin] = LOW;

        // Set pin to its correcponding ALT mode or (PWM MODE)
        *(gpio_map+offset) = 
            (*(gpio_map+offset) & ~(7 << shift)) | ((gpio_fsel_alt << shift) & (7 << shift));

        // Set frequency to defualt Arduino frequency (490Hz) and duty cycle value to zero
        setPwmFrequency(pin, PWM_DEFAULT_FREQUENCY, 0);

        // Ser PWM range to default of 256 bits of resolution
        if (pwm_channel == 1) {
            *(pwm_map + PWM1_RANGE) = PWM_DUTYCYCLE_RESOLUTION;
        } else {
            *(pwm_map + PWM0_RANGE) = PWM_DUTYCYCLE_RESOLUTION;
        }

        // Set PWM in MARKSPACE MODE and Enable PWM 
        if (pwm_channel == 1) {
            *(pwm_map + PWM_CONTROL) |= ( 0x8000 | 0x0100 );  // (PWM1_MS_MODE | PWM1_ENABLE )
        } else {
            *(pwm_map + PWM_CONTROL) |= ( 0x0080 | 0x0001 );  // (PWM0_MS_MODE | PWM0_ENABLE )
        }

        } else {
            fprintf(stderr, "%s(): pin mode %d is not an available mode \n", __func__, mode);
            exit(1);
        }

}

// Sets a pin (gpio) output to 1 or 0
inline void digitalWrite(uint8_t pin, uint8_t val)
{
    int offset;

    // Check if the pin number is valid
    if (pin >= SOC_GPIO_PINS) {
        fprintf(stderr, "%s(): pin number should be less than "
            "%d, yours is %d \n", __func__, SOC_GPIO_PINS, pin);
        exit(1);
    }

    // Check if pin has been initialized 
    if (g_gpio_pin_set[pin] != HIGH) {
        fprintf(stderr, "%s(): please initalize pin %d first "
            "using pinMode() function \n",__func__, pin);
        exit(1);
    }

    if (val) { // value == HIGH
        offset = OFFSET_SET + (pin / 32);
    } else {    // value == LOW
        offset = OFFSET_CLR + (pin / 32);
    }
    *(gpio_map+offset) = 1 << pin % 32;
}

// Returns the value of a pin (gpio) input (1 or 0)
inline int digitalRead(uint8_t pin)
{
    int offset, value, mask;

    // Check if the pin number is valid
    if (pin >= SOC_GPIO_PINS) {
        fprintf(stderr, "%s(): pin number should be less than "
            "%d, yours is %d \n", __func__, SOC_GPIO_PINS, pin);
        exit(1);
    }

    // Check if pin has been initialized 
    if (g_gpio_pin_set[pin] != HIGH) {
        fprintf(stderr, "%s(): please initalize pin %d first "
            "using pinMode() function \n",__func__, pin);
        exit(1);
    }


    offset = OFFSET_PINLEVEL + (pin/32);
    mask = (1 << pin%32);
    value = *(gpio_map+offset) & mask;
    return (value) ? HIGH : LOW;
}

/////////////////////////////////////////////
//          Analog I/O                    //
////////////////////////////////////////////


// Changes the duty Cycle of the PWM
void analogWrite(uint8_t pin, uint32_t value) 
{
    int pwm_channel = 0;

    // Check if pin has been initialized
    if (g_pwm_pin_set[pin] != HIGH) {
        fprintf(stderr, "%s(): please initalize pin %d first "
            "using pinMode() function \n",__func__, pin);
        exit(1);
    } else {
        // Check if the pin is valid for PWM and assign its channel
        switch (pin) {
            case 12: pwm_channel = 0; break;
            case 13: pwm_channel = 1; break;
            case 18: pwm_channel = 0; break;
            case 19: pwm_channel = 1; break;
            case 40: pwm_channel = 0; break;
            case 41: pwm_channel = 1; break;
            case 45: pwm_channel = 1; break;
            case 52: pwm_channel = 0; break;
            case 53: pwm_channel = 1; break;
            default:
                fprintf(stderr, "%s(): pin %d can not be assigned for "
                 "analogWrite() with PWM_OUTPUT mode\n",__func__, pin);
                exit(1);
                break;
        }
    }

    // Check if duty cycle resolution match
    if (value >= PWM_DUTYCYCLE_RESOLUTION) {
        fprintf(stderr, "%s(): dutycycle %d should be less than the "
            "max pwm resolution = %d \n",
            __func__, value, PWM_DUTYCYCLE_RESOLUTION);
            exit(1);
    }


    // Set PWM0 Duty Cycle Value
    g_pwm_dutycycle_value[pin] = value;
    if (pwm_channel == 1) {
        *(pwm_map + PWM1_DATA) = g_pwm_dutycycle_value[pin];
    } else {
        *(pwm_map + PWM0_DATA) = g_pwm_dutycycle_value[pin];
    }

}

// Does the same as anaogWrite but the function name makes more sense.
void setPwmDutyCycle (uint8_t pin, uint32_t dutycycle)
{
    analogWrite(pin, dutycycle);
}

void setPwmPeriod (uint8_t pin, uint32_t microseconds) 
{
    setPwmFrequency(pin, (1000000 / microseconds), g_pwm_dutycycle_value[pin]);
}

void setPwmFrequency (uint8_t pin, uint32_t frequency) 
{
    setPwmFrequency(pin, frequency, g_pwm_dutycycle_value[pin]);
}

// Sets PWM frequency (in Hertz) and pwm duty cycle
void setPwmFrequency (uint8_t pin, uint32_t frequency, uint32_t dutycycle) 
{
    int pwm_channel = 0;
    int divisor;
    double period;
    double countDuration;

    if (g_pwm_pin_set[pin] != HIGH) {
        fprintf(stderr, "%s(): please initalize pin %d first "
            "using pinMode() function \n",__func__, pin);
        exit(1);
    }

    // Check if the pin is valid for PWM and assign its channel
    switch (pin) {
        case 12: pwm_channel = 0; break;
        case 13: pwm_channel = 1; break;
        case 18: pwm_channel = 0; break;
        case 19: pwm_channel = 1; break;
        case 40: pwm_channel = 0; break;
        case 41: pwm_channel = 1; break;
        case 45: pwm_channel = 1; break;
        case 52: pwm_channel = 0; break;
        case 53: pwm_channel = 1; break;
        default:
            fprintf(stderr, "%s(): pin %d can not be assigned for "
                "this function with PWM_OUTPUT mode \n",__func__, pin);
            exit(1);
            break;
    }

    // Check if duty cycle resolution match
    if (dutycycle >= PWM_DUTYCYCLE_RESOLUTION) {
        fprintf(stderr, "%s(): duty cycle %d should be less than the "
            "max pwm resolution = %d \n",
            __func__, dutycycle, PWM_DUTYCYCLE_RESOLUTION);
            exit(1);
    }

    // -- Set frequency and duty cycle

    // stop clock and waiting for busy flag doesn't work, so kill clock
    *(clk_map + PWMCLK_CNTL) = 0x5A000000 | 0x01;
    unistd::usleep(10);

    // wait until busy flag is set 
    while ( (*(clk_map + PWMCLK_CNTL)) & 0x80);

    //calculate divisor value for PWM1 clock...base frequency is 19.2MHz
    period = 1.0/frequency; 
    countDuration = period/(PWM_DUTYCYCLE_RESOLUTION*1.0f);
    divisor = (int)(19200000.0f / (1.0/countDuration));

    if( divisor < 0 || divisor > 4095 ) {
        fprintf(stderr, "%s(): pwm frequency %d with pwm duty cycle "
            "resolution/range of %d bits not supported \n",__func__,
             frequency, PWM_DUTYCYCLE_RESOLUTION);
        exit(-1);
    }

    // Set divisor
    *(clk_map + PWMCLK_DIV) = 0x5A000000 | (divisor << 12);

    // source=osc and enable clock
    *(clk_map + PWMCLK_CNTL) = 0x5A000011;

    // Set PWM0 Duty Cycle pin Value to zero
    g_pwm_dutycycle_value[pin] = dutycycle;
    if (pwm_channel == 1) {
        *(pwm_map + PWM1_DATA) = g_pwm_dutycycle_value[pin];
    } else {
        *(pwm_map + PWM0_DATA) = g_pwm_dutycycle_value[pin];
    }

}


/////////////////////////////////////////////
//          Advanced I/O                  //
////////////////////////////////////////////

// Arguments for Tone threads
struct ThreadToneArg {
    int pin;
    unsigned long duration;
};

pthread_t idToneThread[SOC_GPIO_PINS];

// This is function will be running in a thread if
// non-blocking tone() is called.
void * toneThreadFunction(void *args)
{
    ThreadToneArg *arguments = (ThreadToneArg *)args;
    int pin = arguments->pin;
    unsigned long duration = arguments->duration;

    unistd::usleep(duration*1000);
    noTone(pin);
}


// Set tone frequency (in hertz) and duration (in milliseconds)
void tone(uint8_t pin, uint32_t frequency, unsigned long duration, uint32_t block)
{
    pthread_t *threadId;
    struct ThreadToneArg *threadArgs;

    // Set frequency at 50% duty cycle
    setPwmFrequency(pin, frequency, PWM_DUTYCYCLE_RESOLUTION / 2);

    // Tone duration: If duration == 0, don't stop the tone, 
    // else perform duration either blocking or non-blocking
    if (duration == 0) {
        return;
    } else {

        threadId = &idToneThread[SOC_GPIO_PINS];
        threadArgs = (ThreadToneArg *)malloc(sizeof(ThreadToneArg));
        threadArgs->pin = pin;
        threadArgs->duration = duration;

        // Cancel any existant threads for the pwm pin
        if (*threadId != 0) {
            pthread_cancel(*threadId);
        }

        // If block == true  stop the tone after a sleep delay
        // If block == false then start a thread that will stop the tone
        // after certain duration and parallely continue with the rest of the func. 
        if  (block) {
            unistd::usleep(duration*1000);
            noTone(pin);
        } else {
            pthread_create (threadId, NULL, toneThreadFunction, (void *)threadArgs);
        }
    }
}

void noTone(uint8_t pin) {
    analogWrite(pin, 0);
}

uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) 
{
    uint8_t value = 0;
    uint8_t i;

    for (i = 0; i < 8; ++i) {
        digitalWrite(clockPin, HIGH);
        if (bitOrder == LSBFIRST)
            value |= digitalRead(dataPin) << i;
        else
            value |= digitalRead(dataPin) << (7 - i);
        digitalWrite(clockPin, LOW);
    }
    return value;
}


void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val)
{
    uint8_t i;

    for (i = 0; i < 8; i++)  {
        if (bitOrder == LSBFIRST)
            digitalWrite(dataPin, !!(val & (1 << i)));
        else    
            digitalWrite(dataPin, !!(val & (1 << (7 - i))));
            
        digitalWrite(clockPin, HIGH);
        digitalWrite(clockPin, LOW);        
    }
}

// Measures the length (in microseconds) of a pulse on the pin; state is HIGH
// or LOW, the type of pulse to measure. timeout is 1 second by default.
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout)
{
    struct timespec start, end;
    clock_gettime(CLOCK_REALTIME, &start);

    // wait for any previous pulse to end
    while (digitalRead(pin) == state) {
        clock_gettime(CLOCK_REALTIME, &end);
        // timeDiffmicros > timeout?
        if (((end.tv_sec - start.tv_sec) * 1e6  + 
            (end.tv_nsec - start.tv_nsec) * 1e-3) > timeout)
            return 0;
    }

    // wait for the pulse to start
    while (digitalRead(pin) != state) {
        clock_gettime(CLOCK_REALTIME, &end);
         // timeDiffmicros > timeout?
        if (((end.tv_sec - start.tv_sec) * 1e6  + 
            (end.tv_nsec - start.tv_nsec) * 1e-3) > timeout)
            return 0;
    }

    clock_gettime(CLOCK_REALTIME, &start);
    // wait for the pulse to stop
    while (digitalRead(pin) == state) {
        clock_gettime(CLOCK_REALTIME, &end);
         // timeDiffmicros > timeout?
        if (((end.tv_sec - start.tv_sec) * 1e6  + 
            (end.tv_nsec - start.tv_nsec) * 1e-3) > timeout)
            return 0;
    }

    // return microsecond ellpsed
    return ((end.tv_sec - start.tv_sec) * 1e6  + 
            (end.tv_nsec - start.tv_nsec) * 1e-3);
}


/////////////////////////////////////////////
//          Time                          //
////////////////////////////////////////////

// Returns the time in millseconds since the program started.
unsigned long millis(void) 
{
    struct timespec timenow, start, end;
    clock_gettime(CLOCK_REALTIME, &timenow);
    start = Arduino.timestamp;
    end = timenow;
    // timeDiffmillis:
    return ((end.tv_sec - start.tv_sec) * 1e3 + (end.tv_nsec - start.tv_nsec) * 1e-6);
}

// Returns the time in microseconds since the program started.
unsigned long micros(void)
{
    struct timespec timenow, start, end;
    clock_gettime(CLOCK_REALTIME, &timenow);
    start = Arduino.timestamp;
    end = timenow;
    // timeDiffmicros
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

// Arguments for Externatl Interrupt threads
struct ThreadExtArg {
    void (*func)();
    int pin;
};

pthread_t idExtThread[SOC_GPIO_PINS];

// This is the function that will be running in a thread if
// attachInterrupt() is called 
void * threadFunction(void *args)
{
    ThreadExtArg *arguments = (ThreadExtArg *)args;
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
    if (fd<0) {
        fprintf(stderr, "%s(): gpio error: %s\n", __func__, strerror (errno));
        exit(1);
    }
    pfd.fd=fd;
    pfd.events=POLLPRI;
    
    ret=unistd::read(fd,rdbuf,RDBUF_LEN-1);
    if (ret<0) {
        fprintf(stderr, "%s(): gpio error: %s\n", __func__, strerror (errno));
        exit(1);
    }
    
    while(1) {
        memset(rdbuf, 0x00, RDBUF_LEN);
        unistd::lseek(fd, 0, SEEK_SET);
        ret=poll(&pfd, 1, -1);
        if (ret<0) {
            fprintf(stderr, "%s(): gpio error: %s\n", __func__, strerror (errno));
            unistd::close(fd);
            exit(1);
        }
        if (ret==0) {
            // Timeout
            continue;
        }
        ret=unistd::read(fd,rdbuf,RDBUF_LEN-1);
        if (ret<0) {
            fprintf(stderr, "%s(): gpio error: %s\n", __func__, strerror (errno));
            exit(1);
        }
        //Interrupt. We call user function.
        arguments->func();
    }
}

void attachInterrupt(uint8_t pin, void (*f)(void), int mode)
{
    pthread_t *threadId = &idExtThread[pin];
    struct ThreadExtArg *threadArgs = (ThreadExtArg *)malloc(sizeof(ThreadExtArg));
    threadArgs->func = f;
    threadArgs->pin = pin;

    // Return if the interrupt pin number is out of range
    // NOT_AN_INTERRUPT is set when digitalPinToInterrupt(p) is used for an invalid pin
    if (pin == (uint8_t) NOT_AN_INTERRUPT) {
        fprintf(stderr, "%s(): interrupr pin number out of range\n",__func__);
        return;
    }

    // Check if the pin number is valid
    if (pin >= SOC_GPIO_PINS) {
        fprintf(stderr, "%s(): pin number should be less than "
            "%d, yours is %d \n", __func__, SOC_GPIO_PINS, pin);
        exit(1);
    }
    
    // Export pin for interrupt
    FILE *fp = fopen("/sys/class/gpio/export","w");
    if (fp == NULL) {
        fprintf(stderr, "%s(): export gpio error: %s\n",__func__, strerror (errno));
        exit(1);
    } else {
        fprintf(fp,"%d",pin); 
    }
    fclose(fp);
    
    // Tell the system to create the file /sys/class/gpio/gpio<GPIO number>
    char * interruptFile = NULL;
    asprintf(&interruptFile, "/sys/class/gpio/gpio%d/edge",pin);
    
    //Set detection edge condition
    fp = fopen(interruptFile,"w");
    if (fp == NULL) {
        // First time may fail because the file may not be ready.
        // if that the case then we wait two seconds and try again.
        unistd::sleep(2);
        fp = fopen(interruptFile,"w");
        if (fp == NULL) {
            fprintf(stderr, "%s(): set gpio edge interrupt of (%s) error: %s\n",
                __func__, interruptFile, strerror (errno));
            exit(1);
        }
    }
    switch(mode) {
        case RISING: fprintf(fp,"rising");break;
        case FALLING: fprintf(fp,"falling");break;
        default: fprintf(fp,"both");break;  // Change
    }
    fclose(fp);
    
    // Cancel any existant threads for the interrupt pin
    if (*threadId != 0) {
        pthread_cancel(*threadId);
    }

    // Create a thread passing the pin, function and mode
    pthread_create (threadId, NULL, threadFunction, (void *)threadArgs);
    
}

void detachInterrupt(uint8_t pin)
{
    pthread_t *threadId = &idExtThread[pin];

    // Return if the interrupt pin number is out of range
    // NOT_AN_INTERRUPT is set when digitalPinToInterrupt(p) is used for an invalid pin
    if (pin == (uint8_t) NOT_AN_INTERRUPT) {
        fprintf(stderr, "%s(): interrupr pin number out of range\n",__func__);
        return;
    }

    // Check if the pin number is valid
    if (pin >= SOC_GPIO_PINS) {
        fprintf(stderr, "%s(): pin number should be less than "
            "%d, yours is %d \n", __func__, SOC_GPIO_PINS, pin);
        exit(1);
    }

    // Cancel Thread
    pthread_cancel(*threadId);

    // Unexport gpio pin
    FILE *fp = fopen("/sys/class/gpio/unexport","w");
    if (fp == NULL) {
        fprintf(stderr, "%s(): unexrpot gpio error: %s\n",__func__, strerror (errno));
        exit(1);
    } else {
        fprintf(fp,"%d",pin); 
    }
    fclose(fp);
    
}

/////////////////////////////////////////////
//          Interrupts                    //
////////////////////////////////////////////


/////////////////////////////////////////////
//    Extra Arduino Functions for Linux   //
////////////////////////////////////////////
void (*ARDUINO_EXIT_FUNC)(void) = NULL;

// Every time an arduino program is ran it executes the following functions.
ArduinoPi::ArduinoPi()
{
    // Gets a timestamp when the program starts
    clock_gettime(CLOCK_REALTIME, &timestamp);

    // Set a callback function to detect when program is closed.
    // This is important so later we can turn off any gpio and pwm running. 
    if (signal(SIGINT, ArduinoPi::onArduinoExit) == SIG_ERR)  // Ctrl^C
        fprintf(stderr, "%s(): can't catch signal SIGINT: %s\n",__func__, strerror (errno));
    if (signal(SIGTERM, ArduinoPi::onArduinoExit) == SIG_ERR) // Kill command
        fprintf(stderr, "%s(): can't catch signal SIGKILL: %s\n",__func__, strerror (errno));
    if (signal(SIGHUP, ArduinoPi::onArduinoExit) == SIG_ERR)  // Terminal closes
        fprintf(stderr, "%s(): can't catch signal SIGHUP: %s\n",__func__, strerror (errno));

}


// Catch Ctrl^C (SIGINT) and kill (SIGKILL) signals to set gpio and pwm to default state
void ArduinoPi::onArduinoExit(int signumber)
{
    int i;

    // Shut down 
    if (signumber == SIGINT || signumber == SIGTERM ||  signumber == SIGHUP) {

        // If user wants to call a function at the end, here he can call it. 
        // He can exit so the rest of the code don't take place.
        if (ARDUINO_EXIT_FUNC != NULL) {
            // Call User exit func
            (*ARDUINO_EXIT_FUNC)();
        } else {

            // Set PWM and GPIO used pins to default state = input with no pull-up resistor
            for (i=0; i<SOC_GPIO_PINS; i++) {
                if (g_gpio_pin_set[i] == HIGH || g_pwm_pin_set[i] == HIGH) {
                    pinMode(i, INPUT);
                }
            }

            exit(0);

        }

    }
}


ArduinoPi Arduino = ArduinoPi();
SerialPi Serial = SerialPi();
WirePi Wire = WirePi();
SPIPi SPI = SPIPi();
