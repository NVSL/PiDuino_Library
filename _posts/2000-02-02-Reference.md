---
title:  "Reference"
author: "Jorge Garza"
categories: reference
sdate: "Aug 5, 2016"
---

### Reference

**PiDuino Library (libpiduino)** aims to support the **core** official Arduino reference published in the [Arduino official website](https://www.arduino.cc/en/Reference/Libraries) as of April 2016 plus the Wire and SPI libraries. Note that the Arduino API has many legacy functions that although keep no longer used, the ported functions and libraries are only the ones published on the official website.

**libpiduino 1.0.0** has 99 functions implemented out of 111 functions, this means that about **89%** of the core official Arduino functions plus the I2C and SPI libraries were implemented. Also 11 (EXTRA) functions which give more functionality to the library were included. Note that only hardware PWM is included in this release and software PWM will be included in the next one. 

___________________________________________________

> **(*)** - Functions that have changes compared the official Arduino reference.  
> **(#)** - Functions that require sudo to run them.  
> **(EXTRA)** - Extra functions not in the official Arduino reference.  

___________________________________________________

#### - Digital I/O

| Function | Implemented <br> (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| **(*)** [pinMode(pin, mode)]({{ site.baseurl }}{% post_url 2000-02-02-pinMode %})    | YES |
| [digitalWrite(pin, value)]({{ site.baseurl }}{% post_url 2000-02-02-digitalWrite %}) | YES |
| [digitalRead(pin)]({{ site.baseurl }}{% post_url 2000-02-02-digitalRead %})  | YES |

#### - Analog I/O (ADC and PWM)

| Function | Implemented <br> (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| analogReference(type) | NO - No hardware support for RPi |
| analogRead(pin) | NO - No hardware support for RPi |
| **(#)** [analogWrite(pin, value)]({{ site.baseurl }}{% post_url 2000-02-02-analogWrite %}) - PWM | YES |
| **(EXTRA, #)** [setPwmDutyCycle(pin, dutycycle)]({{ site.baseurl }}{% post_url 2000-02-02-setPwmDutyCycle %}) | YES |
| **(EXTRA, #)** [setPwmFrequency(pin, frequency, dutycycle)]({{ site.baseurl }}{% post_url 2000-02-02-setPwmDutyFrequency%})  | YES |
| **(EXTRA, #)** [setPwmFrequency(pin, frequency)]({{ site.baseurl }}{% post_url 2000-02-02-setPwmFrequency%})  | YES |
| **(EXTRA, #)** [setPwmPeriod(pin, microseconds)]({{ site.baseurl }}{% post_url 2000-02-02-setPwmPeriod%}) | YES |
 
#### - Advanced I/O

| Function | Implemented <br> (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| **(#)** [tone(pin, frequency)]({{ site.baseurl }}{% post_url 2000-02-02-tone%})    | YES |
| **(#)** [tone(pin, frequency, duration)]({{ site.baseurl }}{% post_url 2000-02-02-tone-duration%})   | YES |
| **(EXTRA, #)** [tone(pin, frequency, duration, block)]({{ site.baseurl }}{% post_url 2000-02-02-tone-duration-block%})   | YES |
| **(#)** [noTone(pin) ]({{ site.baseurl }}{% post_url 2000-02-02-noTone%})   | YES |
| shiftOut(dataPin, clockPin, bitOrder, value)  | YES |
| shiftIn(dataPin, clockPin, bitOrder) | YES |
| [pulseIn(pin, value)]({{ site.baseurl }}{% post_url 2000-02-02-pulseIn%})  | YES |
| [pulseIn(pin, value, timeout)]({{ site.baseurl }}{% post_url 2000-02-02-pulseIn-duration%})  | YES |

#### - Time

| Function | Implemented <br> (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| time = [millis()]({{ site.baseurl }}{% post_url 2000-02-02-millis%})   | YES |
| time = [micros()]({{ site.baseurl }}{% post_url 2000-02-02-micros%})   | YES |
| [delay(ms)]({{ site.baseurl }}{% post_url 2000-02-02-delay%})   | YES |
| [delayMicroseconds(us)]({{ site.baseurl }}{% post_url 2000-02-02-delayMicroseconds%})   | YES |

#### - External Interrupts

| Function | Implemented <br> (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| attachInterrupt(digitalPinToInterrupt(pin), ISR, mode)  | YES |
| detachInterrupt(interrupt)  | YES |
| detachInterrupt(digitalPinToInterrupt(pin))  | YES |

#### - Serial

| Function | Implemented <br> (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| if (Serial)  | YES |
| Serial.available() | YES |
| Serial.availableForWrite() | YES |
| [Serial.begin(speed) ]({{ site.baseurl }}{% post_url 2000-02-02-Serial-begin%}) | YES |
| [Serial.begin(speed, config) ]({{ site.baseurl }}{% post_url 2000-02-02-Serial-begin-config%})| YES |
| **(EXTRA)** [Serial.begin(driverName, speed)]({{ site.baseurl }}{% post_url 2000-02-02-Serial-Extra-begin%}) | YES |
| **(EXTRA)** [Serial.begin(driverName, speed, config)]({{ site.baseurl }}{% post_url 2000-02-02-Serial-Extra-begin-config%}) | YES |
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
| **(EXTRA)** [Serial.printf(format, ...) ]({{ site.baseurl }}{% post_url 2000-02-02-Serial-Extra-printf%}) | YES |
| Serial.read() | YES |
| Serial.readBytes(buffer, length)| YES |
| Serial.readBytesUntil(character, buffer, length) | YES |
| Serial.readString() | YES |
| Serial.readStringUntil(terminator) | YES |
| **(EXRTA)** [Serial.readStringCommand(character, buffer, length)]({{ site.baseurl }}{% post_url 2000-02-02-Serial-Extra-readStringCommand%}) | YES |
| Serial.setTimeout(time) | YES |
| Serial.write(val)  | YES |
| Serial.write(str)  | YES |
| Serial.write(buf, len) | YES |
| serialEvent() | In Progress |

#### - Wire (I2C)

| Function | Implemented <br> (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| [Wire.begin()]({{ site.baseurl }}{% post_url 2000-02-02-Wire-begin%}) - I2C Master | YES |
| **(EXTRA)** [Wire.begin(driverName)]({{ site.baseurl }}{% post_url 2000-02-02-Wire-Extra-begin%}) - I2C Master | YES |
| Wire.begin(address) - I2C Slave | NO/In Progress - Linux I2C Slave driver <br> was added recently, still verifying |
| Wire.requestFrom(address, quantity) | YES |
| Wire.requestFrom(address, quantity, stop) | NO - There is no way to send <br> an I2C stop msg to the driver |
| Wire.beginTransmission(address) | YES |
| Wire.endTransmission() | YES |
| Wire.endTransmission(stop)  | NO - There is no way to send <br> an I2C stop msg to the driver |
| Wire.write(value) | YES |
| Wire.write(string)  | YES |
| Wire.write(data, length) | YES |
| Wire.available() | YES |
| Wire.read()  | YES |
| Wire.onReceive(handler) | NO/In Progress - Linux I2C Slave driver <br> was added recently, still verifying |
| Wire.onRequest(handler) | NO/In Progress - Linux I2C Slave driver <br> was added recently, still verifying |

#### - SPI

| Function | Implemented <br> (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| SPISettings | YES |
| [SPI.begin()]({{ site.baseurl }}{% post_url 2000-02-02-SPI-begin%}) - SPI Master | YES |
| **(EXTRA)** [SPI.begin(driverName)]({{ site.baseurl }}{% post_url 2000-02-02-SPI-Extra-begin%}) - SPI Master | YES |
| SPI.end()| YES |
| SPI.beginTransaction(mySettings) | YES |
| SPI.endTransaction() | YES |
| SPI.setBitOrder(order) | YES |
| SPI.setClockDivider(divider) | YES |
| SPI.setDataMode(mode) | YES |
| receivedVal = SPI.transfer(val) | YES |
| receivedVal16 = SPI.transfer16(val16) | NO - Is almost depreciated and could <br> be fixed better adding a bitsPerWord in <br> settings that includes 16 bits and 32 bits |
| SPI.transfer(buffer, size) | YES |
| SPI.usingInterrupt(interruptNumber) | In Progress |

#### - Math

| Function | Implemented <br> (YES/NO-Comment/In Progress) |
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

| Function | Implemented <br> (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| sin(rad) | YES |
| cos(rad) | YES |
| tan(rad) | YES |

#### - Characters

| Function | Implemented <br> (YES/NO-Comment/In Progress) |
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

| Function | Implemented <br> (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| randomSeed(seed)  | YES |
| random(max)  | YES |
| random(min, max)  | YES |

#### - Bits and Bytes

| Function | Implemented <br> (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| lowByte(x) | YES |
| highByte(x) | YES |
| bitRead(x, n) | YES |
| bitWrite(x, n, b) | YES |
| bitSet(x, n) | YES |
| bitClear(x, n) | YES |
| bit(n) | YES |

#### - Interrupts

| Function | Implemented <br> (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| interrupts()  | NO |
| noInterrupts() | NO |

**Notes:**

**B** binary representation (e.g  **B1000000**[7 bits] and  **B110**[3 bits]) conflicts with some termios.h definitions so only 8 bit binary representations are supported (e.g **B01000000**[8 bits], **B00010001**[8 bits], etc). To represent a binary with less or more than 8 bits please use **0b** instead. 