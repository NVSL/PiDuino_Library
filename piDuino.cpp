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


#include "piDuino.h"


/////////////////////////////////////////////
//          SerialPi class (UART)         //
////////////////////////////////////////////


////  Public methods ////

//Constructor
SerialPi::SerialPi(){
    serialPort="/dev/ttyAMA0";
    timeOut = 1000;
}

//Sets the data rate in bits per second (baud) for serial data transmission
void SerialPi::begin(int serialSpeed)
{

	switch(serialSpeed){
		case     50:	speed =     B50 ; break ;
		case     75:	speed =     B75 ; break ;
		case    110:	speed =    B110 ; break ;
		case    134:	speed =    B134 ; break ;
		case    150:	speed =    B150 ; break ;
		case    200:	speed =    B200 ; break ;
		case    300:	speed =    B300 ; break ;
		case    600:	speed =    B600 ; break ;
		case   1200:	speed =   B1200 ; break ;
		case   1800:	speed =   B1800 ; break ;
		case   2400:	speed =   B2400 ; break ;
		case   9600:	speed =   B9600 ; break ;
		case  19200:	speed =  B19200 ; break ;
		case  38400:	speed =  B38400 ; break ;
		case  57600:	speed =  B57600 ; break ;
		case 115200:	speed = B115200 ; break ;
		default:	speed = B230400 ; break ;
			
	}


	if ((sd = open(serialPort, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1){
		fprintf(stderr,"Unable to open the serial port %s - \n", serialPort);
		exit(-1);
	}
    
	fcntl (sd, F_SETFL, O_RDWR) ;
    
	tcgetattr(sd, &options);
	cfmakeraw(&options);
	cfsetispeed (&options, speed);
	cfsetospeed (&options, speed);

	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_oflag &= ~OPOST;

	tcsetattr (sd, TCSANOW, &options);

	ioctl (sd, TIOCMGET, &status);

	status |= TIOCM_DTR;
	status |= TIOCM_RTS;

	ioctl (sd, TIOCMSET, &status);
	
	unistd::usleep (10000);
    
}

//Prints data to the serial port as human-readable ASCII text.
void SerialPi::print(const char *message)
{
    unistd::write(sd,message,strlen(message));
}

//Prints data to the serial port as human-readable ASCII text.
void SerialPi::print (char message)
{
	unistd::write(sd,&message,1);
}

/*Prints data to the serial port as human-readable ASCII text.
 * It can print the message in many format representations such as:
 * Binary, Octal, Decimal, Hexadecimal and as a BYTE. */
void SerialPi::print(unsigned char i,Representation rep)
{
    char * message;
    switch(rep){

        case BIN:
            message = int2bin(i);
            unistd::write(sd,message,strlen(message));
            break;
        case OCT:
            message = int2oct(i);
            unistd::write(sd,message,strlen(message));
            break;
        case DEC:
            sprintf(message,"%d",i);
            unistd::write(sd,message,strlen(message));
            break;
        case HEX:
            message = int2hex(i);
            unistd::write(sd,message,strlen(message));
            break;
        case BYTE:
            unistd::write(sd,&i,1);
            break;

    }
}

/* Prints data to the serial port as human-readable ASCII text.
 * precission is used to limit the number of decimals.
 */
void SerialPi::print(float f, int precission)
{
	/*
    const char *str1="%.";
    char * str2;
    char * str3;
    char * message;
    sprintf(str2,"%df",precission);
    asprintf(&str3,"%s%s",str1,str2);
    sprintf(message,str3,f);
	*/
	char message[10];
	sprintf(message, "%.1f", f );
    unistd::write(sd,message,strlen(message));
}

/* Prints data to the serial port as human-readable ASCII text followed
 * by a carriage retrun character '\r' and a newline character '\n' */
void SerialPi::println(const char *message)
{
	const char *newline="\r\n";
	char * msg = NULL;
	asprintf(&msg,"%s%s",message,newline);
    unistd::write(sd,msg,strlen(msg));
}

/* Prints data to the serial port as human-readable ASCII text followed
 * by a carriage retrun character '\r' and a newline character '\n' */
void SerialPi::println(char message)
{
	const char *newline="\r\n";
	char * msg = NULL;
	asprintf(&msg,"%s%s",&message,newline);
    unistd::write(sd,msg,strlen(msg));
}

/* Prints data to the serial port as human-readable ASCII text followed
 * by a carriage retrun character '\r' and a newline character '\n' */
void SerialPi::println(int i, Representation rep)
{
    char * message;
    switch(rep){

        case BIN:
            message = int2bin(i);
            break;
        case OCT:
            message = int2oct(i);
            break;
        case DEC:
            sprintf(message,"%d",i);
            break;
        case HEX:
            message = int2hex(i);
            break;

    }

    const char *newline="\r\n";
    char * msg = NULL;
    asprintf(&msg,"%s%s",message,newline);
    unistd::write(sd,msg,strlen(msg));
}

/* Prints data to the serial port as human-readable ASCII text followed
 * by a carriage retrun character '\r' and a newline character '\n' */
void SerialPi::println(float f, int precission)
{
    const char *str1="%.";
    char * str2;
    char * str3;
    char * message;
    sprintf(str2,"%df",precission);
    asprintf(&str3,"%s%s",str1,str2);
    sprintf(message,str3,f);

    const char *newline="\r\n";
    char * msg = NULL;
    asprintf(&msg,"%s%s",message,newline);
    unistd::write(sd,msg,strlen(msg));
}

/* Writes binary data to the serial port. This data is sent as a byte 
 * Returns: number of bytes written */
int SerialPi::write(unsigned char message)
{
	unistd::write(sd,&message,1);
	return 1;
}

/* Writes binary data to the serial port. This data is sent as a series
 * of bytes
 * Returns: number of bytes written */
int SerialPi::write(const char *message)
{
	int len = strlen(message);
	unistd::write(sd,&message,len);
	return len;
}

/* Writes binary data to the serial port. This data is sent as a series
 * of bytes placed in an buffer. It needs the length of the buffer
 * Returns: number of bytes written */
int SerialPi::write(char *message, int size)
{
	unistd::write(sd,message,size);
	return size;
}

/* Get the numberof bytes (characters) available for reading from 
 * the serial port.
 * Return: number of bytes avalable to read */
int SerialPi::available()
{
    int nbytes = 0;
    if (ioctl(sd, FIONREAD, &nbytes) < 0)  {
		fprintf(stderr, "Failed to get byte count on serial.\n");
        exit(-1);
    }
    return nbytes;
}

/* Reads 1 byte of incoming serial data
 * Returns: first byte of incoming serial data available */
char SerialPi::read() 
{
	unistd::read(sd,&c,1);
    return c;
}

/* Reads characters from th serial port into a buffer. The function 
 * terminates if the determined length has been read, or it times out
 * Returns: number of bytes readed */
int SerialPi::readBytes(char message[], int size)
{
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
    int count;
    for (count=0;count<size;count++) {
    	if (available()) unistd::read(sd,&message[count],1);
    	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
    	timespec t = timeDiff(time1,time2);
    	if ((t.tv_nsec/1000)>timeOut) break;
    }
    return count;
}

/* Reads characters from the serial buffer into an array. 
 * The function terminates if the terminator character is detected,
 * the determined length has been read, or it times out.
 * Returns: number of characters read into the buffer. */
int SerialPi::readBytesUntil(char character,char buffer[],int length)
{
    char lastReaded = character +1; //Just to make lastReaded != character
    int count=0;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
    while (count != length && lastReaded != character) {
        if (available()) unistd::read(sd,&buffer[count],1);
        lastReaded = buffer[count];
        count ++;
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
        timespec t = timeDiff(time1,time2);
        if ((t.tv_nsec/1000)>timeOut) break;
    }

    return count;
}


bool SerialPi::find(const char *target)
{
    findUntil(target,NULL);
}

/* Reads data from the serial buffer until a target string of given length
 * or terminator string is found.
 * Returns: true if the target string is found, false if it times out */
bool SerialPi::findUntil(const char *target, const char *terminal)
{
    int index = 0;
    int termIndex = 0;
    int targetLen = strlen(target);
    int termLen = strlen(terminal);
    char readed;
    timespec t;

    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);

    if( *target == 0)
        return true;   // return true if target is a null string

    do{
        if(available()){
            unistd::read(sd,&readed,1);
            if(readed != target[index])
            index = 0; // reset index if any char does not match

            if( readed == target[index]){
                if(++index >= targetLen){ // return true if all chars in the target match
                    return true;
                }
            }

            if(termLen > 0 && c == terminal[termIndex]){
                if(++termIndex >= termLen) return false; // return false if terminate string found before target string
            }else{ 
                termIndex = 0;
            }
        }

        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
        t = timeDiff(time1,time2);

    } while((t.tv_nsec/1000)<=timeOut);

    return false;
}

/* returns the first valid (long) integer value from the current position.
 * initial characters that are not digits (or the minus sign) are skipped
 * function is terminated by the first character that is not a digit. */
long SerialPi::parseInt()
{
    bool isNegative = false;
    long value = 0;
    char c;

    //Skip characters until a number or - sign found
    do{
        c = peek();
        if (c == '-') break;
        if (c >= '0' && c <= '9') break;
        unistd::read(sd,&c,1);  // discard non-numeric
    }while(1);

    do{
        if(c == '-')
            isNegative = true;
        else if(c >= '0' && c <= '9')// is c a digit?
            value = value * 10 + c - '0';
        unistd::read(sd,&c,1);  // consume the character we got with peek
        c = peek();

    }while(c >= '0' && c <= '9');

    if(isNegative)
        value = -value;
    return value;
}

float SerialPi::parseFloat()
{
    boolean isNegative = false;
    boolean isFraction = false;
    long value = 0;
    char c;
    float fraction = 1.0;

    //Skip characters until a number or - sign found
    do {
        c = peek();
        if (c == '-') break;
        if (c >= '0' && c <= '9') break;
        unistd::read(sd,&c,1);  // discard non-numeric
    } while(1);

    do {
        if(c == '-')
            isNegative = true;
        else if (c == '.')
            isFraction = true;
        else if(c >= '0' && c <= '9') {      // is c a digit?
            value = value * 10 + c - '0';
            if(isFraction)
                fraction *= 0.1;
        }
        unistd::read(sd,&c,1);  // consume the character we got with peek
        c = peek();
    }while( (c >= '0' && c <= '9')  || (c == '.' && isFraction==false));

    if(isNegative)
        value = -value;
    if(isFraction)
        return value * fraction;
    else
        return value;


}

// Returns the next byte (character) of incoming serial data without removing it from the internal serial buffer.
char SerialPi::peek()
{
    //We obtain a pointer to FILE structure from the file descriptor sd
    FILE * f = fdopen(sd,"r+");
    //With a pointer to FILE we can do getc and ungetc
    c = getc(f);
    ungetc(c, f);
    return c;
}

// Remove any data remaining on the serial buffer
void SerialPi::flush()
{
    while (available()) {
        unistd::read(sd,&c,1);
    }
}

/* Sets the maximum milliseconds to wait for serial data when using SerialPi::readBytes()
 * The default value is set to 1000 */
void SerialPi::setTimeout(long millis){
	timeOut = millis;
}

//Disables serial communication
void SerialPi::end(){
	unistd::close(sd);
}


////  Private methods ////


//Returns a timespec struct with the time elapsed between start and end timespecs
timespec SerialPi::timeDiff(timespec start, timespec end){
	timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}

//Returns a binary representation of the integer passed as argument
char * SerialPi::int2bin(int i){
	size_t bits = sizeof(int) * CHAR_BIT;
    char * str = (char *)malloc(bits + 1);
    int firstCeros = 0;
    int size = bits;
    if (!str) return NULL;
    str[bits] = 0;

    // type punning because signed shift is implementation-defined
    unsigned u = *(unsigned *)&i;
    for (; bits--; u >>= 1)
        str[bits] = u & 1 ? '1' : '0';

    //Delete first 0's
    for (int i=0; i<bits; i++){
        if (str[i] == '0') {
            firstCeros++;
        } else {
            break;
        }
    }
    char * str_noceros = (char *)malloc(size-firstCeros+1);
    for (int i=0; i<(size-firstCeros);i++){
        str_noceros[i]=str[firstCeros+i];
    }

    return str_noceros;
}

//Returns an hexadecimal representation of the integer passed as argument
char * SerialPi::int2hex(int i){
	char buffer[32];
    sprintf(buffer,"%x",i);
    char * hex = (char *)malloc(strlen(buffer)+1);
    strcpy(hex,buffer);
    return hex;
}

//Returns an octal representation of the integer passed as argument
char * SerialPi::int2oct(int i){
    char buffer[32];
    sprintf(buffer,"%o",i);
    char * oct = (char *)malloc(strlen(buffer)+1);
    strcpy(oct,buffer);
    return oct;	
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


int WirePi::i2c_write_bytes(int file, uint8_t *values, size_t length)
{
    int bytes_written = 0;

    if (length == 0) {
        return bytes_written;
    } else {
        bytes_written = unistd::write(file, values, length);
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

////  Public methods ////

//Constructor
WirePi::WirePi()
{
    fd = -1;
}

//Initiate the Wire library
void WirePi::begin()
{

    FILE *fp;
    char filename[20];
    char path[1024];
    char i2cDevice[1024] = "";
    
    // Open the command for reading and search for i2c-x devices, 
    // where x = 0 or 1 depending on the raspberry Pi version.
    fp = popen("/bin/ls /dev/ | /bin/grep i2c-" , "r");
        if (fp == NULL) {
        fprintf(stderr, "%s(): Failed to run command \"/bin/ls /dev/ | /bin/grep i2c-\"\n",__func__);
        exit(1);
    }

    // Get first i2c-x device
    while (fgets(path, sizeof(path)-1, fp) != NULL) {
        snprintf(i2cDevice, 1024, "%s", path);
        break;
    }

    // If I2c device driver is not enabled or installed, then exit. 
    if(strcmp(i2cDevice,"") == 0) {
        fprintf(stderr, "%s(): Filed to locate a \"i2c-x\" device driver in /dev/. \
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

//Writes data to the I2C, returns bytes written.
size_t WirePi::write(uint8_t data)
{

    if(transmitting) {
        // in master transmitter mode
        // don't bother if buffer is full
        if(txBufferLength >= BUFFER_LENGTH){
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

//Writes data to the I2C, returns bytes written. 
size_t WirePi::write(uint8_t *data, size_t quantity)
{

    size_t ret = 0;

    if (transmitting) {
        // in master transmitter mode
        for(size_t i = 0; i < quantity; ++i){
            write(data[i]);
        }
        ret = quantity;
    } else {
        // in slave send mode
        // reply to master
        ret = i2c_write_bytes(fd, data, quantity);
    }
    
    return ret;
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

//Used by the master to request bytes from a slave device
void WirePi::requestFrom(unsigned char address,int quantity)
{
    // TODO, set I2C addres here
    // Then create an Available function ???
}

//Reads a byte that was transmitted from a slave device to a master after a call to WirePi::requestFrom()
unsigned char WirePi::read()
{

    if (fd < 0) {
        fprintf(stderr, "%s(): Initialize I2C first with Wire.begin() \n", __func__);
        exit(1);
    }

    /*
    union i2c_smbus_data data;
    if (i2c_smbus_access(fd,I2C_SMBUS_READ,0,I2C_SMBUS_BYTE,&data)) {
        fprintf(stderr, "%s(): I2C error: %s\n", __func__, strerror (errno));
        return -1;
    } else {
        return (unsigned char) data.byte;
    }
    */
}



/////////////////////////////////////////////
//          SPIPi class (SPI)             //
////////////////////////////////////////////

////  Public methods ////


 SPIPi::SPIPi(){
	 
 }

void SPIPi::begin(){

}

void SPIPi::end(){  

}

void SPIPi::setBitOrder(uint8_t order){
    // BCM2835_SPI_BIT_ORDER_MSBFIRST is the only one suported by SPI0
}

// defaults to 0, which means a divider of 65536.
// The divisor must be a power of 2. Odd numbers
// rounded down. The maximum SPI clock rate is
// of the APB clock
void SPIPi::setClockDivider(uint16_t divider){

}

void SPIPi::setDataMode(uint8_t mode){

}

// Writes (and reads) a single byte to SPI
uint8_t SPIPi::transfer(uint8_t value){
    uint8_t ret = 0;
    return ret;
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
