/*
  SPI.cpp - LinuxDuino Serial (UART) library

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
#include <time.h> 
#include <termios.h> 
#include <ctype.h>
#include <sys/ioctl.h>
#include <stdarg.h>
#include <linux/types.h>
#include "Serial.h"
#include "WString.h"

// All functions of unistd.h must be called like this: unistd::the_function()
namespace unistd {
    #include <unistd.h>
};

/////////////////////////////////////////////
//          SerialLinux class (UART)       //
////////////////////////////////////////////

char SERIAL_DRIVER_NAME[128] = "/dev/ttyAMA0";

////  Public methods ////

//Constructor
SerialLinux::SerialLinux()
{
    // Default serial driver and timeout
    timeOut = 1000;
    fd = -1;
    fd_file = NULL;
}

void SerialLinux::begin(int baud, unsigned char config)
{
     begin((const char *)SERIAL_DRIVER_NAME, baud, config);
}

// Sets the data rate in bits per second (baud) for serial data transmission
void SerialLinux::begin(const char *serialPort, int baud, unsigned char config)
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
void SerialLinux::end() 
{
    unistd::close(fd);
    fd = -1;
}

// Get the numberof bytes (characters) available for reading from 
// the serial port.
// Return: number of bytes avalable to read 
int SerialLinux::available()
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
int SerialLinux::availableForWrite ()
{
    return 63;
}

bool SerialLinux::find(const char *target)
{
    return findUntil(target,NULL);
}

// Reads data from the serial buffer until a target string of given length,
// terminator string is found or times out.
// Returns: true if target string is found, false if times out or terminator is found.
bool SerialLinux::findUntil(const char *target, const char *terminator)
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
void SerialLinux::flush()
{
    tcflush(fd,TCIOFLUSH);
}

// returns the first valid floating point number from the serial buffer.
// initial characters that are not digits (or the minus sign) are skipped
// function is terminated by the first character that is not a digit.
float SerialLinux::parseFloat()
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
long SerialLinux::parseInt(char ignore)
{
    bool isNegative = false;
    long value = 0;
    int c;

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
int SerialLinux::peek()
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
size_t SerialLinux::print(const String &s)
{
    return unistd::write(fd,s.c_str(), s.length());
}

// Prints data to the serial port as human-readable ASCII text.
size_t SerialLinux::print(const char str[])
{
    return unistd::write(fd,str,strlen(str));
}

// Prints one character to the serial port as human-readable ASCII text.
size_t SerialLinux::print(char c)
{
    return unistd::write(fd,&c,1);
}

size_t SerialLinux::print(unsigned char b, int base)
{
  return print((unsigned int) b, base);
}

// Prints data to the serial port as human-readable ASCII text.
// It can print the message in many format representations such as:
// Binary, Octal, Decimal and Hexadecimal.
size_t SerialLinux::print(unsigned int n, int base)
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
size_t SerialLinux::print(int n, int base)
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
size_t SerialLinux::println(void)
{
    char * msg;
    asprintf(&msg,"\r\n");
    return unistd::write(fd,msg,strlen(msg));
}

// Prints data to the serial port as human-readable ASCII text
// Followed by a new line
size_t SerialLinux::println(const String &s)
{
    size_t n = print(s);
    n += println();
    return n;
}

// Prints data to the serial port as human-readable ASCII text
// Followed by a new line
size_t SerialLinux::println(const char c[])
{
  size_t n = print(c);
  n += println();
  return n;
}

// Prints one character to the serial port as human-readable ASCII text.
// Followed by a new line
size_t SerialLinux::println(char c)
{
  size_t n = print(c);
  n += println();
  return n;
}

size_t SerialLinux::println(unsigned char b, int base)
{
  size_t n = print(b, base);
  n += println();
  return n;
}

size_t SerialLinux::println(int num, int base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t SerialLinux::println(unsigned int num, int base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

// Prints like a normal C language printf() but to the Serial port
size_t SerialLinux::printf(const char *fmt, ... ) {
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
int SerialLinux::read() 
{
    int8_t c;
    unistd::read(fd,&c,1);
    return c;
}

// Reads characters from th serial port into a buffer. The function 
// terminates if the determined length has been read, or it times out
// Returns: number of bytes readed
size_t SerialLinux::readBytes(char buffer[], size_t length)
{
    timespec time1, time2;
    clock_gettime(CLOCK_REALTIME, &time1);
    size_t count = 0;
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
size_t SerialLinux::readBytesUntil(char terminator, char buffer[], size_t length)
{
    timespec time1, time2;
    clock_gettime(CLOCK_REALTIME, &time1);
    size_t count = 0;
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
String SerialLinux::readString()
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
String SerialLinux::readStringUntil(char terminator)
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
size_t SerialLinux::readStringCommand(char terminator, char buffer[], size_t length)
{
    size_t count = 0;
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
void SerialLinux::setTimeout(long millis)
{
    timeOut = millis;
}

// Writes binary data to the serial port. This data is sent as a byte 
// Returns: number of bytes written
size_t SerialLinux::write(uint8_t c)
{
    unistd::write(fd,&c,1);
    return 1;
}

// Writes binary data to the serial port. This data is sent as a series
// of bytes
// Returns: number of bytes written
size_t SerialLinux::write(const char *str)
{
    if (str == NULL) return 0;
    return unistd::write(fd,str,strlen(str));
}

// Writes binary data to the serial port. This data is sent as a series
// of bytes placed in an buffer. It needs the length of the buffer
// Returns: number of bytes written 
size_t SerialLinux::write(char *buffer, size_t size)
{
    return unistd::write(fd,buffer,size);
}


////  Private methods ////

// private method to peek stream with timeout
int SerialLinux::timedPeek()
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
int SerialLinux::peekNextDigit(bool detectDecimal)
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
long SerialLinux::timeDiffmillis(timespec start, timespec end)
{
    return ((end.tv_sec - start.tv_sec) * 1e3 + (end.tv_nsec - start.tv_nsec) * 1e-6);
}

// Returns a binary representation of the integer passed as argument
char * SerialLinux::int2bin(int n)
{
    size_t bits = sizeof(int) * 8;
    char * str = (char *)malloc(bits + 1);
    unsigned int mask = 1 << (bits-1); //Same as 0x80000000
    size_t i = 0;

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

SerialLinux Serial = SerialLinux();