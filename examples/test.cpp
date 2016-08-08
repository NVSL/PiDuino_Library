#include <Arduino.h>

int ledPin = 4; // GPIO4

void setup() {
        pinMode(ledPin, OUTPUT);
}

void loop() {
        printf("LED ON\n");
        //digitalWrite(ledPin, HIGH);
        isAlphaNumeric(4);
        delay(1000);
        printf("LED OFF\n");
        //digitalWrite(ledPin, LOW);
        delay(1000);
}


/*
// Simple GPIO 
#include "piDuino.h"

int GPIO_OUTPUT = 14;

void setup() {
	pinMode(GPIO_OUTPUT, OUTPUT);
}

void loop() {
	digitalWrite(GPIO_OUTPUT, HIGH);
	delay(1000);
	digitalWrite(GPIO_OUTPUT, LOW);
	delay(1000);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/


/*
// Testing Arduino header Serial printf
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

void setup() {
        Serial.begin(115200);     // opens serial port, sets data rate to 9600 bps
        Wire.begin();
        SPI.begin();
		SPI.setBitOrder(MSBFIRST);
		SPI.setClockDivider(125000);
		SPI.setDataMode(SPI_MODE1);
}

void loop() {
  Serial.printf ("Integer = %d, Float = %f, Char = %c, String = %s \n\r", 
  				10, 3.1416, 'a', "Hello World");
  delay(5000);
}
*/

/*
// Testing Serial printf
#include "piDuino.h"

void setup() {
	printf("%s\n", LINUXDUINO_VERSION);
    Serial.begin(115200);     // opens serial port, sets data rate to 9600 bps
}

void loop() {
  Serial.printf ("Integer = %d, Float = %f, Char = %c, String = %s \n\r", 
  				10, 3.1416, 'a', "Hello World");
  delay(5000);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/


/*
// Test readStringCommand
#include "piDuino.h"
#include "stdio.h"

void setup() {
        Serial.begin(115200);     // opens serial port, sets data rate to 9600 bps
}

void loop() {
  char data[50];
  memset(data, 0, sizeof(data));
  //  Read
  Serial.readStringCommand('\r', data, sizeof(data));
  printf ("Data = %s\n", data);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/

/*
// Test readString
#include "piDuino.h"
#include "stdio.h"

String data = "";

void setup() {
        Serial.begin(115200);     // opens serial port, sets data rate to 9600 bps
        //Serial.setTimeout(10000);  // Set timeout of 5 seconds
}

void loop() {
  //  Read
  data = Serial.readStringUntil('a');
  printf ("Serial ended\n");
  Serial.println(data);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/

/*
// Test readString
#include "piDuino.h"
#include "stdio.h"

String data = "";

void setup() {
        Serial.begin(115200);     // opens serial port, sets data rate to 9600 bps
        //Serial.setTimeout(10000);  // Set timeout of 5 seconds
}

void loop() {
  //  Read
  data = Serial.readString();
  printf ("Serial ended\n");
  Serial.println(data);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/


/*
// String implementarion for piDuino
// Notes: __FlashStringHelper was removed from String Object 
// and macro F(string) returns string
#include "piDuino.h"
#include <string>

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // send an intro:
  Serial.println("\n\nString Constructors:");
  Serial.println();
}

void loop() {
  std::string normalstr = "A normal string";
  Serial.println(normalstr.c_str());      // prints "Hello String"

  // using a constant String:
  String stringOne = "Hello String";
  Serial.println(stringOne);      // prints "Hello String"

  // converting a constant char into a String:
  stringOne =  String('a');
  Serial.println(stringOne);       // prints "a"

  // converting a constant string into a String object:
  String stringTwo =  String("This is a string");
  Serial.println(stringTwo);      // prints "This is a string"

  // concatenating two strings:
  stringOne =  String(stringTwo + " with more");
  // prints "This is a string with more":
  Serial.println(stringOne);

  // using a constant integer:
  stringOne =  String(13);
  Serial.println(stringOne);      // prints "13"

  // using an int and a base:
  stringOne =  String(15, DEC);
  // prints "453" or whatever the value of analogRead(A0) is
  Serial.println(stringOne);

  // using an int and a base (hexadecimal):
  stringOne =  String(45, HEX);
  // prints "2d", which is the hexadecimal version of decimal 45:
  Serial.println(stringOne);

  // using an int and a base (binary)
  stringOne =  String(255, BIN);
  // prints "11111111" which is the binary value of 255
  Serial.println(stringOne);

  // using a long and a base:
  stringOne =  String(millis(), DEC);
  // prints "123456" or whatever the value of millis() is:
  Serial.println(stringOne);

  //using a float and the right decimal places:
  stringOne = String(5.698, 3);
  Serial.println(stringOne);

  //using a float and less decimal places to use rounding:
  stringOne = String(5.698, 2);
  Serial.println(stringOne);

  // do nothing while true:
  while (true);

}



int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/

/*
// Test tones, NOTE*** is important to declare 
// pinmode() before using tone, in arduino you
// don't need to declare it. 
#include "piDuino.h"

int GPIO_OUTPUT = 12;

void setup() {
	pinMode(GPIO_OUTPUT, PWM_OUTPUT);
}

void loop() {
	printf("Setting PWM\n");
	//tone(GPIO_OUTPUT, 1000, 10000);
	//tone(GPIO_OUTPUT, 1000, 5000);
	//tone(GPIO_OUTPUT, 1000, 1000, true);
	//delay(10000);
	tone(GPIO_OUTPUT, 2000);
	delay(1000);
	tone(GPIO_OUTPUT, 3000, 1000);
	tone(GPIO_OUTPUT, 4000, 1000);
	tone(GPIO_OUTPUT, 5000, 1000);
	tone(GPIO_OUTPUT, 10000, 1000);
	//noTone(GPIO_OUTPUT);
	delay(5000);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/

/*
// Test setPwmFrequency
#include "piDuino.h"

int GPIO_OUTPUT = 12;

void setup() {
	pinMode(GPIO_OUTPUT, PWM_OUTPUT);
}

void loop() {
	printf("Setting PWM\n");
	analogWrite(GPIO_OUTPUT, 128);
	delay(1000);
	setPwmFrequency(GPIO_OUTPUT, 1000);
	delay(1000);
	setPwmFrequency(GPIO_OUTPUT, 2000);
	delay(1000);
	setPwmFrequency(GPIO_OUTPUT, 3000);
	delay(1000);
	setPwmFrequency(GPIO_OUTPUT, 4000);
	delay(1000);
	setPwmFrequency(GPIO_OUTPUT, 5000);
	delay(1000);
	setPwmFrequency(GPIO_OUTPUT, 10000);
	delay(1000);
	analogWrite(GPIO_OUTPUT, 0);
	delay(5000);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}

*/


/*
// Soft Frequency for Buzzer
#include "piDuino.h"

int GPIO_OUTPUT = 19;

void setup() {
	pinMode(GPIO_OUTPUT, OUTPUT);
}

void loop() {
	digitalWrite(GPIO_OUTPUT, HIGH);
	//printf("ON \n");
	delay(100);
	digitalWrite(GPIO_OUTPUT, LOW);
	//printf("OFF \n");
	delay(100);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/



/*
// Simple GPIO 
#include "piDuino.h"

int GPIO_OUTPUT = 14;

void setup() {
	pinMode(GPIO_OUTPUT, OUTPUT);
}

void loop() {
	digitalWrite(GPIO_OUTPUT, HIGH);
	delay(1000);
	digitalWrite(GPIO_OUTPUT, LOW);
	delay(1000);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/



/*
// Simple GPIO test with USER EXIT FUNCTION
#include "piDuino.h"

int GPIO_OUTPUT = 14;

void myExit (void) {
	printf("My exit commands\n");
	exit(1);
}

void setup() {
	ARDUINO_EXIT_FUNC = &myExit;
	pinMode(GPIO_OUTPUT, OUTPUT);
}

void loop() {
	digitalWrite(GPIO_OUTPUT, HIGH);
	delay(1000);
	digitalWrite(GPIO_OUTPUT, LOW);
	delay(1000);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/


/*
// Ultrasonic sensor.
#include "piDuino.h"

#define trigPin 23
#define echoPin 24

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
	long long duration, distance;
	digitalWrite(trigPin, LOW);  // Added this line
	delayMicroseconds(2); // Added this line
	digitalWrite(trigPin, HIGH);
	delayMicroseconds(100); // Added this line
	digitalWrite(trigPin, LOW);
	duration = pulseIn(echoPin, HIGH);
	distance = (duration/2) / 29.1;
	printf("duration = %d, Distance =  %d \n", duration, distance);
  	delay(500);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/

/* Pulse in simple test
#include "piDuino.h"

int pin = 7;
unsigned long duration = 0;

void setup()
{
  pinMode(pin, INPUT);
}


void loop()
{
  printf("Starting Loop\n");
  duration = pulseIn(pin, HIGH);
  printf("Duration: %lu \n", duration);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/ 

/*
// Gpio delay test.
#include "piDuino.h"

int GPIO_OUTPUT = 14;
int GPIO_INPUT = 23;
int input_val = 0;

struct timespec start, end;

int timeDiffmicros(timespec start, timespec end)
{
    return (int) ((end.tv_sec - start.tv_sec) * 1e6 + (end.tv_nsec - start.tv_nsec) * 1e-3);
}

void setup() {
	pinMode(GPIO_OUTPUT, OUTPUT);
	pinMode(GPIO_INPUT, INPUT);
}

void loop() {
	clock_gettime(CLOCK_REALTIME, &start);
	digitalWrite(GPIO_OUTPUT, HIGH);
	clock_gettime(CLOCK_REALTIME, &end);
	input_val = digitalRead(GPIO_INPUT);
	printf("OUTPUT = 1, INPUT = %d, Delay = %d\n", input_val, timeDiffmicros(start, end));
	delay(1000);
	clock_gettime(CLOCK_REALTIME, &start);
	digitalWrite(GPIO_OUTPUT, LOW);
	clock_gettime(CLOCK_REALTIME, &end);
	input_val = digitalRead(GPIO_INPUT);
	printf("OUTPUT = 0, INPUT = %d, Delay = %d\n", input_val, timeDiffmicros(start, end));
	delay(1000);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/


/*
// Interrupts
#include "piDuino.h"

const byte ledPin = 14;
const byte interruptPin = 23;
volatile byte state = LOW;
volatile boolean unattach = false;
volatile int counter = 0;

void blink() {
  state = !state;
  printf("On interrupt \n");
  if (counter >= 20) {
  	unattach = true;
  }
  counter ++;
}

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, RISING);
}

void loop() {
  digitalWrite(54, state);
  if (unattach) {
  	printf("Detaching\n");
	detachInterrupt(digitalPinToInterrupt(interruptPin));
	unattach = false;
	state = LOW;
  }
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/



/*
// Pin test with gpio.h
#include "piDuino.h"

int GPIO_OUTPUT = 14;
int GPIO_INPUT = 15;
int input_val = 0;

void setup() {
	pinMode(GPIO_OUTPUT, OUTPUT);
	pinMode(GPIO_INPUT, INPUT);
}

void loop() {
	digitalWrite(GPIO_OUTPUT, HIGH);
	input_val = digitalRead(GPIO_INPUT);
	printf("OUTPUT = 1, INPUT = %d\n", input_val);
	delay(1000);
	digitalWrite(GPIO_OUTPUT, LOW);
	input_val = digitalRead(GPIO_INPUT);
	printf("OUTPUT = 0, INPUT = %d\n", input_val);
	delay(1000);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/


/*
// Pin test with gpio.h
#include "gpio.h"
#include <unistd.h>
#include <stdio.h>

int GPIO_OUTPUT = 14;
int GPIO_INPUT = 15;
int input_val = 0;

void run() {
	setup();
	setup_gpio(GPIO_OUTPUT, OUTPUT, PUD_OFF);
	setup_gpio(GPIO_INPUT, INPUT, PUD_OFF);
}

void loop() {
	output_gpio(GPIO_OUTPUT, HIGH);
	input_val = input_gpio(GPIO_INPUT);
	printf("OUTPUT = 1, IMPUT = %d\n", input_val);
	sleep(1);
	output_gpio(GPIO_OUTPUT, LOW);
	input_val = input_gpio(GPIO_INPUT);
	printf("OUTPUT = 0, IMPUT = %d\n", input_val);
	sleep(1);
}

int main () {
	run();
	while(1){
		loop();
	}
	return (0);
}
*/


/*
// Times test
#include "piDuino.h"

void setup() {
}

void loop() {
	printf("Delay Since Start millis %d \n", (int) millis());
	printf("Delay Since Start micros %d \n", (int) micros());
	delayMicroseconds(1000000);
	delay(1000);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/


/*
// if(Serial) test
#include "piDuino.h"

void setup() {
        Serial.begin(115200);     // opens serial port, sets data rate to 9600 bps
}

void loop() {

	char buff[] = "Hello world";

	if(Serial) {
		Serial.write('A');
		Serial.write("Lol");
		Serial.write(buff, strlen(buff));
		Serial.write('\r');
		Serial.write('\n');
		delay(1000);
	}

}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/


/*
// Test ReadBytesUntil
#include "piDuino.h"
#include "stdio.h"
int incomingByte = 0;   // for incoming serial data

void setup() {
        Serial.begin(115200);     // opens serial port, sets data rate to 9600 bps
        Serial.setTimeout(5000);  // Set timeout of 5 seconds
}

void loop() {
  char buff[10];

  // Clean buffer
  for (int i = 0; i<10; i++) {
    buff[i] = 0;
  }

  //  Read 5 bytes
  Serial.readBytesUntil('g', buff, 5);
  buff[5] = 0;
  Serial.print("Recieved = ");
  Serial.println(buff);
  
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/

/*
// Test ReadBytes
#include "piDuino.h"
#include "stdio.h"
int incomingByte = 0;   // for incoming serial data

void setup() {
        Serial.begin(115200);     // opens serial port, sets data rate to 9600 bps
        Serial.setTimeout(5000);  // Set timeout of 5 seconds
}

void loop() {
  char buff[10];

  // Clean buffer
  for (int i = 0; i<10; i++) {
    buff[i] = 0;
  }

  //  Read 5 bytes
  Serial.readBytes(buff, 5);
  buff[5] = 0;
  Serial.print("Recieved = ");
  Serial.println(buff);
  
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/

/*
// Test Read
#include "piDuino.h"
#include "stdio.h"
int incomingByte = 0;   // for incoming serial data

void setup() {
        Serial.begin(115200);     // opens serial port, sets data rate to 9600 bps
}

void loop() {

        // send data only when you receive data:
        if (Serial.available() > 0) {
                // read the incoming byte:
                incomingByte = Serial.read();

                // say what you got:
                Serial.print("I received: ");
                Serial.println(incomingByte, DEC);
        }
}


int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/

/*
// Prints TODO add string String and (arg, ..)
#include "piDuino.h"
#include "stdio.h"

void setup() {
        Serial.begin(115200);     // opens serial port, sets data rate to 9600 bps
}

void loop() {

	char message[] = "A char message";

	Serial.print(" A const char message (no new line) ");
	Serial.println(" A const char message (with new line) ");
	Serial.print(message);
	Serial.println(message);

	Serial.println(15, DEC);
	Serial.println(15, OCT);
	Serial.println(15, HEX);
	Serial.println(15, BIN);
	Serial.println(-15, DEC);
	Serial.println(-15, OCT);
	Serial.println(-15, HEX);
	Serial.println(-15, BIN);

	delay(10000);

}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/

/*
// Parse Int
#include "piDuino.h"
#include "stdio.h"

void setup() {
        Serial.begin(115200);     // opens serial port, sets data rate to 9600 bps
}

void loop() {
		int value;

		printf("Write Something...\n");
		delay(10000);
		value = Serial.parseInt('1');
		printf("Your number is = %d\n", value);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/


/*
// Parse Float
#include "piDuino.h"
#include "stdio.h"

void setup() {
        Serial.begin(115200);     // opens serial port, sets data rate to 9600 bps
}

void loop() {
		float value;
		char c;

		printf("Write Something...\n");
		delay(10000);
		value = Serial.parseFloat();
		printf("Your number is = %g\n", value);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/

/*
// peek, -1 when nothing and afected by timeOut 
#include "piDuino.h"
#include "stdio.h"

void setup() {
        Serial.begin(115200);     // opens serial port, sets data rate to 9600 bps
}

void loop() {
		char c;

		printf("Write Something...\n");
		c = Serial.peek();
		printf("Your char is = %c\n", c);
		delay(1000);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/

/*
#include "piDuino.h"
#include "stdio.h"

void setup() {
        Serial.begin(115200);     // opens serial port, sets data rate to 9600 bps
}

void loop() {

		bool isOKFound = false;
        // send data only when you receive data:
		Serial.setTimeout(10000);
		// If target OK found return true, if timeout or teminator "l" found return false
		isOKFound = Serial.findUntil("OK", "l");    //Wait for 'OK' for 5 seconds
		if (isOKFound) {
			printf("OK found yeii\n");
		} else {
			printf("Noting found \n");
		}

		Serial.flush();

		printf("Time OVER\n");
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/

/*
#include "piDuino.h"
#include "stdio.h"

int incomingByte = 0;   // for incoming serial data

void setup() {
        Serial.begin(115200);     // opens serial port, sets data rate to 9600 bps
        while (!Serial) {
    		printf("Waiting for Serial ...\n");
  		}

}

void loop() {

        // send data only when you receive data:
        if (Serial.available() > 0) {
            // read the incoming byte:
            incomingByte = Serial.read();

            // say what you got:
            Serial.println("I got something");
            Serial.println(incomingByte, DEC);
        }

        //delay(1000);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/

/*
#include "piDuino.h"
#include "stdio.h"

void setup() {
	Serial.begin(115200);
}

void loop() {
	Serial.println("Hello World");
	delay(1000);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/


/*
// SPI buffer (Wires must be well attached in order to send data without errors)
#include "piDuino.h"
#include "stdio.h"

void setup() {
	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
	SPI.setClockDivider(125000);
	SPI.setDataMode(SPI_MODE1);
}

void loop() {
	char c[] =  "Hello World!\n";
	char ret;

    SPI.transfer (c, strlen(c));

	delay(1000);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}

*/
/*
// SPI without settings
#include "piDuino.h"
#include "stdio.h"

void setup() {
	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
	SPI.setClockDivider(2000000);
	SPI.setDataMode(SPI_MODE1);
}

void loop() {
	char c;
	char ret;

  	for (const char * p = "Hello, world!\n" ; c = *p; p++) {
    	ret = SPI.transfer (c);
    	printf("%c", ret);
  	}
  	printf("\n");

	delay(1000);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/

/*
//SPI with settings
#include "piDuino.h"
#include "stdio.h"

SPISettings settingsA(2000000, MSBFIRST, SPI_MODE1); 

void setup() {
	SPI.begin();
}

void loop() {
	char c;
	char ret;
	SPI.beginTransaction(settingsA);
  	for (const char * p = "Hello, world!\n" ; c = *p; p++) {
    	ret = SPI.transfer (c);
    	printf("%c", ret);
  	}
  	printf("\n");
    SPI.endTransaction();

	delay(1000);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/


/*
#include "piDuino.h"
#include "stdio.h"

void setup() {
	Wire.begin();
}

void loop() {
	Wire.requestFrom(0x70, 6);    // request 6 bytes from slave device #8

	while (Wire.available()) { // slave may send less than requested
	char c = Wire.read(); // receive a byte as character
	printf("%c\n", c);
	}

	Wire.beginTransmission(0x70);
	Wire.write("hello ");
	Wire.endTransmission();

	delay(500);
}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/



/*
#include "piDuino.h"
#include "stdio.h"

uint16_t displaybuffer[8]; 

void setup() {
	Wire.begin();
}

void loop() {

	printf("Running ... \n");

	// Begin
	
	// Init Display
	Wire.beginTransmission(0x70);
	Wire.write(33);
	Wire.endTransmission();

	Wire.beginTransmission(0x70);
	Wire.write(129);
	Wire.endTransmission();

	Wire.beginTransmission(0x70);
	Wire.write(239);  
	Wire.endTransmission();

	// Fill display
	for (uint8_t i=0; i<8; i++) {
    	displaybuffer[i] = 0xFF;
  	}

  	// Write to display
  	Wire.beginTransmission(0x70);
  	Wire.write((uint8_t)0x00);
	for (uint8_t i=0; i<8; i++) {
		//Wire.writeReg(i*2,displaybuffer[i]); // start at address $00   

	  	
		//Wire.write(i*2);       
		//Wire.write(displaybuffer[i]);  
		

		Wire.write(displaybuffer[i] & 0xFF);       
		Wire.write(displaybuffer[i] >> 8);      

	}
	Wire.endTransmission();
	delay(2000);              // wait for a second

}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/



/*
#include "piDuino.h"
#include "LEDArray.h"
#include "stdio.h"


//Adafruit_8x8matrix matrix = Adafruit_8x8matrix();
LEDArray led = LEDArray();


uint16_t displaybuffer[8]; 

void setup() {
	//pinMode(4, OUTPUT);
}

void loop() {

	printf("Writing...\n");

	led.setup();
	led.drawHappyFace();
	
	delay(1000);


}

int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/

/*
#include "piDuino.h"
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"


Adafruit_8x8matrix matrix = Adafruit_8x8matrix();

void setup() {
  matrix.begin(0x70);  // pass in the address
}

void loop() {
	printf("Running ... \n");

	matrix.clear();      // clear display
	matrix.drawLine(0,0, 7,7, LED_ON);
	matrix.writeDisplay();  // write the changes we just made to the display
	delay(1000);
}


int main () {
	setup();
	while(1){
		loop();
	}
	return (0);
}
*/