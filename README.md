# PiDuino_Library

PiDuino Library is a C++ library that lets you write program for Raspberry Pi as if you were writing an Arduino program. 

This library does not allow you to change hadrware peripheral intefaces (e.g change I2C pins for GPIO pins) on real time as this library relay on the I2C, SPI, TTY(Serial) and GPIO linux drivers to interface with the pins I/Os. 

This library also does not support Analog to Digital Conversion (or ADC) functions as the Raspberry Pi hardware does not have an integrated ADC and we don´t want to create hardware dependant code. If you need ADC is recomended to use external libraries. 

## Implemented Arduino Functions and Libraries
. 
PiDuino Library aims to support the most basic official Arduino functions published in the [Arduino official web page] (https://www.arduino.cc/en/Reference/Libraries) as of April 2016. Note that Arduino API have many legacy functions that although keeped no longer used.

| Implemented Arduino Functions | Status |
| ------ | ----------- |
| Digital I/O  | Testing |
| Analog I/O (PWM only) |  In progress |
| Advanced I/O | In progress |
| Time | In progress |
| Math | In progress |
| Trigonometry| In progress |
| Characters | In progress |
| Random Numbers | In progress |
| Bits and Bytes | In progress |
| External Interrupts | In progress |
| Interrupts | In progress |
| Communication | Testing |

| Implemented Arduino Libraries | Status |
| ------ | ----------- |
| Serial  | Testing |
| Wire |  In progress |
| SPI   | In progress |


### Implemented Arduino Functions

#### Digital I/O

| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| pinMode(pin, mode)  | In Progress |
| digitalWrite(pin, value) | In Progress |
| digitalRead(pin)  | In Progress |

#### Analog I/O

| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| analogReference(type) | NO - No hardware support for RPi |
| analogRead(pin) | NO - No hardware support for RPi |
| analogWrite() - PWM | In progress |

#### Advanced I/O

| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| tone(pin, frequency)   | In Progress |
| tone(pin, frequency, duration) | In Progress |
| noTone(pin)  | In Progress |
| shiftOut(dataPin, clockPin, bitOrder, value)  | In Progress |
| byte incoming = shiftIn(dataPin, clockPin, bitOrder) | In Progress |
| pulseIn(pin, value)  | In Progress |
| pulseIn(pin, value, timeout) | In Progress |

#### Time

| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| time = millis()  | In Progress |
| time = micros()  | In Progress |
| delay(ms)  | In Progress |
| delayMicroseconds(us)  | YES |

#### Math

| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| min(x, y)  | In Progress |
| max(x, y) | In Progress |
| abs(x)  | In Progress |
| constrain(x, a, b) | In Progress |
| map(value, fromLow, fromHigh, toLow, toHigh)  | In Progress |
| pow(base, exponent)  | In Progress |
| sqrt(x)  | In Progress |

#### Trigonometry

TODO

#### Random Numbers

| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| randomSeed(seed)  | In Progress |
| random(max)  | In Progress |
| random(min, max)  | In Progress |

#### Bits and Bytes

TODO

#### External Interrupts

| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| attachInterrupt(digitalPinToInterrupt(pin), ISR, mode)  | In Progress |
| detachInterrupt(interrupt)  | In Progress |
| detachInterrupt(digitalPinToInterrupt(pin))  | In Progress |

#### Interrupts

| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| interrupts()  | In Progress |
| noInterrupts() | In Progress |

#### Communication

| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| Serial  | YES - See Library |
| Stream | NO - Not implemented for now |



### Implemented Arduino Libraries

#### Serial
| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| if (Serial)  | In Progress |
| Serial.available() | In Progress |
| Serial.availableForWrite() | In Progress |
| Serial.begin(speed) | In Progress |
| Serial.available() | In Progress |
| Serial.begin(speed, config)| In Progress |
| Serial.end() | In Progress |
| Serial.find(target) | In Progress |
| Serial.findUntil(target, terminal) | In Progress |
| Serial.flush() | In Progress |
| Serial.parseFloat() | In Progress |
| Serial.parseInt() | In Progress |
| Serial.parseInt(char skipChar) | In Progress |
| Serial.peek() | In Progress |
| Serial.print(val)  | In Progress |
| Serial.available() | In Progress |
| Serial.available() | In Progress |
| Serial.available() | In Progress |
| Serial.available() | In Progress |
| Serial.available() | In Progress |
| Serial.available() | In Progress |
| Serial.available() | In Progress |
| Serial.available() | In Progress |
| Serial.available() | In Progress |

#### Wire
| Function | Implemented (YES/NO-Comment/In Progress) |
| ------ | ----------- |
| data   | path to data files to supply the data that will be passed into templates. |
| engine | engine to be used for processing templates. Handlebars is the default. |
| ext    | extension to be used for dest files. |

#### SPI
