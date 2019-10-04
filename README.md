# [PiDuino Library](http://nvsl.github.io/PiDuino_Library/)

```diff
- NOTE: You can check the javascript version: https://github.com/NVSL/Linuxduino 
```

PiDuino Library is a C++ library that lets you write programs for Raspberry Pi as if you were writing an Arduino program. 

This library **does not support Analog to Digital Conversion (or ADC)** functions as the Raspberry Pi hardware does not have an integrated ADC and we don´t want to create hardware dependant code. If you need ADC is recomended to use external libraries. 

Web Page: [http://nvsl.github.io/PiDuino_Library/](http://nvsl.github.io/PiDuino_Library/)

## Implemented Arduino Functions and Libraries
PiDuino Library aims to support the most basic official Arduino functions published in the [Arduino official website] (https://www.arduino.cc/en/Reference/Libraries) as of April 2016. Note that the Arduino API has many legacy functions that although keeped no longer used, the ported libraries and functions are only the ones published on the official website.

**Implemented Arduino Functions :**
Digital I/O, Analog I/O, Advanced I/O, Time, Math, Trigonometry, Random Numbers, Bits and Bytes, External Interrupts, Interrupts, Communication

**Implemented Arduino Libraries :**
Serial, Wire, SPI

**Notes:**
* **B** some binary representation (e.g  B1000000[7 bits] and  B110[3 bits]) conflicts with some termios.h definitions so only 8 bit binary representations are supported (e.g B01000000[8 bits], B00010001[8 bits], etc). To represent a binary with less than 8 bits please use **0b** instead. 


## Implemented Arduino Functions

**Notes:**
* Functions that begin with (EXTRA) are functions that were added to provide extra functionality to the library.


#### - Digital I/O

| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| pinMode(pin, mode)  | YES |
| digitalWrite(pin, value) | YES |
| digitalRead(pin)  | YES |

#### - Analog I/O

| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| analogReference(type) | NO - No hardware support for RPi |
| analogRead(pin) | NO - No hardware support for RPi |
| analogWrite(pin, value) - PWM | YES |
| (EXTRA) setPwmDutyCycle (pin, dutycycle)  | YES |
| (EXTRA) setPwmFrequency (pin, frequency, dutycycle)  | YES |
| (EXTRA) setPwmFrequency (pin, frequency)  | YES |
| (EXTRA) setPwmPeriod (pin, microseconds)  | YES |
 
#### - Advanced I/O

| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| tone(pin, frequency)   | YES |
| tone(pin, frequency, duration) | YES |
| noTone(pin)  | YES |
| shiftOut(dataPin, clockPin, bitOrder, value)  | YES |
| byte incoming = shiftIn(dataPin, clockPin, bitOrder) | YES |
| pulseIn(pin, value)  | YES |
| pulseIn(pin, value, timeout) | YES |

#### - Time

| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| time = millis()  | YES |
| time = micros()  | YES |
| delay(ms)  | YES |
| delayMicroseconds(us)  | YES |

#### - Math

| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| min(x, y)  | YES |
| max(x, y) | YES |
| abs(x)  | YES |
| constrain(x, a, b) | YES |
| map(value, fromLow, fromHigh, toLow, toHigh)  | YES |
| pow(base, exponent)  | YES |
| radians(deg) | YES |
| degrees(rad) | YES |
| sqrt(x)  | YES |
| sq(x)  | YES |

#### - Trigonometry

| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| sin(rad) | YES |
| cos(rad) | YES |
| tan(rad) | YES |

#### - Characters

| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| isAlphaNumeric(thisChar) | YES |
| isAlpha(thisChar) | YES |
| isAscii(thisChar) | YES |
| isWhitespace(thisChar) | YES |
| isControl(thisChar) | YES |
| isDigit(thisChar) | YES |
| isGraph(thisChar) | YES |
| isLowerCase(thisChar) | YES |
| isPrintable(thisChar) | YES |
| isPunct(thisChar) | YES |
| isSpace(thisChar) | YES |
| isUpperCase(thisChar) | YES |
| isHexadecimalDigit(thisChar) | YES |

#### - Random Numbers

| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| randomSeed(seed)  | YES |
| random(max)  | YES |
| random(min, max)  | YES |

#### - Bits and Bytes

| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| lowByte(x) | YES |
| highByte(x) | YES |
| bitRead(x, n) | YES |
| bitWrite(x, n, b) | YES |
| bitSet(x, n) | YES |
| bitClear(x, n) | YES |
| bit(n) | YES |

#### - External Interrupts

| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| attachInterrupt(digitalPinToInterrupt(pin), ISR, mode)  | YES |
| detachInterrupt(interrupt)  | YES |
| detachInterrupt(digitalPinToInterrupt(pin))  | YES |

#### - Interrupts

| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| interrupts()  | In Progress |
| noInterrupts() | In Progress |

#### - Communication

| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| Serial  | YES |
| Stream | NO - Not implemented for now |




### Implemented Arduino Libraries



#### - Serial
| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| if (Serial)  | YES |
| Serial.available() | YES |
| Serial.availableForWrite() | YES |
| Serial.begin(speed) | YES |
| Serial.begin(speed, config)| YES |
| (EXTRA) Serial.begin(driverName, speed) | YES |
| (EXTRA) Serial.begin(driverName, speed, config)| YES |
| Serial.end() | YES |
| Serial.find(target) | YES |
| Serial.findUntil(target, terminal) | YES |
| Serial.flush() | YES |
| Serial.parseFloat() | YES |
| Serial.parseInt() | YES |
| Serial.parseInt(char skipChar) | YES |
| Serial.peek() | YES |
| Serial.print(val)  | YES |
| Serial.print(val, format)| YES |
| Serial.println(val)  | YES |
| Serial.println(val, format) | YES |
| (EXTRA) Serial.printf(format, ...) | YES |
| Serial.read() | YES |
| Serial.readBytes(buffer, length)| YES |
| Serial.readBytesUntil(character, buffer, length) | YES |
| Serial.readString() | YES |
| Serial.readStringUntil(terminator) | YES |
| (EXRTA) Serial.readStringCommand(character, buffer, length) | YES |
| Serial.setTimeout(time) | YES |
| Serial.write(val)  | YES |
| Serial.write(str)  | YES |
| Serial.write(buf, len) | YES |
| serialEvent() | In Progress |

#### - Wire
| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| Wire.begin() | YES |
| (EXTRA) Wire.begin(driverName) | YES |
| Wire.begin(address) | NO/In Progress - Linux I2C Slave driver was added recently, still verifying |
| Wire.requestFrom(address, quantity) | YES |
| Wire.requestFrom(address, quantity, stop) | NO - There is no way to send an I2C stop msg to the driver |
| Wire.beginTransmission(address) | YES |
| Wire.endTransmission() | YES |
| Wire.endTransmission(stop)  | NO - There is no way to send an I2C stop msg to the driver |
| Wire.write(value) | YES |
| Wire.write(string)  | YES |
| Wire.write(data, length) | YES |
| Wire.available() | YES |
| Wire.read()  | YES |
| Wire.onReceive(handler) | NO/In Progress - Linux I2C Slave driver was added recently, still verifying |
| Wire.onRequest(handler) | NO/In Progress - Linux I2C Slave driver was added recently, still verifying |

#### - SPI

| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| SPISettings | YES |
| SPI.begin() | YES |
| (EXTRA) SPI.begin(driverName) | YES |
| SPI.end()| YES |
| SPI.beginTransaction(mySettings) | YES |
| SPI.endTransaction() | YES |
| SPI.setBitOrder(order) | YES |
| SPI.setClockDivider(divider) | YES - Sets clock or if you use dividers (e.g SPI_CLOCK_DIV4) then sets clock at the divider speed for a 16Mhz Arduino microcontroller |
| SPI.setDataMode(mode) | YES |
| receivedVal = SPI.transfer(val) | YES |
| receivedVal16 = SPI.transfer16(val16) | NO - Is almost depreceated and could be fixed better adding a bitsPerWord in settings that includes 16 bits and 32 bits |
| SPI.transfer(buffer, size) | YES |
| SPI.usingInterrupt(interruptNumber) | In Progress |


---------------------------------------------------

## TODO'S

* ~~Make a SPI and I2C begin("/dev/i2c-x") to let developers choose any i2c-x number.~~ 
* Check Linux i2c-slave. 
* ~~Maybe only add B(8 bits) only and that way prevent conflicts with termios.h~~
* Defintily separate files and make it C compatible (At the end)
* ~~Make an Arduino.h header containing piDuino.h so developers don't need to change too much code.~~ 
* ~~Add arduino String object~~
* Maybe add a SPISettings(..., bitsPerWord, CS(High?low)) and their bit routines config. to SPI. For bitPerWord transfer len will need to be multiplied by (bitsPerWord/8)
* ~~Add string, String, (arg, ...) to prints.~~
* ~~Make an Arduino init function~~
* ~~Implement an exit rutine when SIGINT (Ctrl^Z) to turn GPIO OFF~~

