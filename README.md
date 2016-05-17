# PiDuino Library

PiDuino Library is a C++ library that lets you write programs for Raspberry Pi as if you were writing an Arduino program. 

This library **does not allow you to change hadrware peripheral intefaces (e.g change I2C pins for GPIO pins)** in real time as this library relays on the I2C, SPI, TTY(Serial) and GPIO linux drivers to interface with the pins I/Os. 

This library also **does not support Analog to Digital Conversion (or ADC)** functions as the Raspberry Pi hardware does not have an integrated ADC and we don´t want to create hardware dependant code. If you need ADC is recomended to use external libraries. 

## Implemented Arduino Functions and Libraries
PiDuino Library aims to support the most basic official Arduino functions published in the [Arduino official website] (https://www.arduino.cc/en/Reference/Libraries) as of April 2016. Note that Arduino API have many legacy functions that although keeped no longer used, the ported libraries and functions are only the ones published in the official website.

**Implemented Arduino Functions :**
Digital I/O, Analog I/O, Advanced I/O, Time, Math, Trigonometry, Random Numbers, Bits and Bytes, External Interrupts, Interrupts, Communication

**Implemented Arduino Libraries :**
Serial, Wire, SPI

**Notes:**
* Data Type String - object:
will not be implemented in the first release. 
* **B** for binary representation (e.g B10100001) conflicts with some termios.h definitions so use **0b** instead. 



### Implemented Arduino Functions



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
| analogWrite() - PWM | In progress |

#### - Advanced I/O

| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| tone(pin, frequency)   | In Progress |
| tone(pin, frequency, duration) | In Progress |
| noTone(pin)  | In Progress |
| shiftOut(dataPin, clockPin, bitOrder, value)  | In Progress |
| byte incoming = shiftIn(dataPin, clockPin, bitOrder) | In Progress |
| pulseIn(pin, value)  | In Progress |
| pulseIn(pin, value, timeout) | In Progress |

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
| attachInterrupt(digitalPinToInterrupt(pin), ISR, mode)  | In Progress |
| detachInterrupt(interrupt)  | In Progress |
| detachInterrupt(digitalPinToInterrupt(pin))  | In Progress |

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
| Serial.read() | YES |
| Serial.readBytes(buffer, length)| YES |
| Serial.readBytesUntil(character, buffer, length) | YES |
| Serial.readString() | In Progress - Needs String which haven't been implemented yet |
| Serial.readStringUntil(terminator) | In Progress - Needs String which haven't been implemented yet |
| Serial.setTimeout(time) | YES |
| Serial.write(val)  | YES |
| Serial.write(str)  | YES |
| Serial.write(buf, len) | YES |
| serialEvent() | In Progress |

#### - Wire
| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| Wire.begin() | YES |
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

* Make a SPI and I2C begin("/dev/i2c-x") to let developers choose any i2c-x number. 
* Check Linux i2c-slave. 
* Maybe only add B(8 bits) only and that way prevent conflicts with termios.h
* Defintily separate files and make it C compatible (At the end)
* Make an Arduino.h header containing piDuino.h so developers don't need to change too much code. 
* Add arduino String object.
* Maybe add a SPISettings(..., bitsPerWord, CS(High?low)) and their bit routines config. to SPI. For bitPerWord transfer len will need to be multiplied by (bitsPerWord/8)
* Add string, String, (arg, ...) to prints.
* Check/DO printFloat()
* Make TimeElapsed() Private. 

