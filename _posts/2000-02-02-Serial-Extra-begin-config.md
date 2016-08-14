---
title:  "Serial.begin"
author: "Jorge Garza"
categories: function
sdate: "Aug 5, 2016"
---

### Serial.begin(driverName, speed, config)

Opens a serial port with at specified serial device driver name and with the specified speed (baud rate) and serial port config options. 

**Parameters**

> **driverName** - Serial port driver Name (e.g "/dev/ttyAMA0")

> **speed** - Serial port baud rate. (See Notes)

> **config** - Serial port configuration options like Data Size, Parity and StopBits. (See Notes)

**Returns**

None

____________________

**Example**

{% highlight c %}
#include <Arduino.h>

void setup() {
	// Open Serial port with:
	// Device driver name = "/dev/ttyAMA0"
	// baud rate: 9600
	// Data size: 8 bits
	// Parity: None
	// Stop Bits: 1 bit
	Serial.begin("/dev/ttyAMA0", 9600, SERIAL_8N1);
}

void loop() {
	Serial.println("Hello World");
	delay(1000);
}
{% endhighlight %}

____________________

**Notes**

- Available speeds (baud rates). Default is 9600

| Baud rates |
|:------:|
| 50            |
| 75            |       
| 110           |
| 134           |
| 150           |
| 200           |
| 300           |
| 600           |
| 1200          |
| 1800          |
| 2400          |
| 9600          |
| 19200         |
| 38400         |
| 57600         |
| 115200        |
| 230400        |
| 460800        |
| 500000        |
| 576000        |
| 921600        |
| 1000000       |
| 1152000       |
| 1500000       |
| 2000000       |
| 2500000       |
| 3000000       |
| 3500000       |
| 4000000       |

- Available serial port configurations. Default is SERIAL_8N1

| Configuration | Data Size | Parity | Stop Bits |
|:------:|:------:||:------:|:------:|
|SERIAL_5N1| 5 bits | None | 1 bit |
|SERIAL_6N1| 6 bits | None | 1 bit |
|SERIAL_7N1| 7 bits | None | 1 bit |
|SERIAL_8N1| 8 bits | None | 1 bit |
|SERIAL_5N2| 5 bits | None | 2 bits |
|SERIAL_6N2| 6 bits | None | 2 bits |
|SERIAL_7N2| 7 bits | None | 2 bits |
|SERIAL_8N2| 8 bits | None | 2 bits |
|SERIAL_5E1| 5 bits | Even | 1 bit |
|SERIAL_6E1| 6 bits | Even | 1 bit |
|SERIAL_7E1| 7 bits | Even | 1 bit |
|SERIAL_8E1| 8 bits | Even | 1 bit |
|SERIAL_5E2| 5 bits | Even | 2 bits |
|SERIAL_6E2| 6 bits | Even | 2 bits |
|SERIAL_7E2| 7 bits | Even | 2 bits |
|SERIAL_8E2| 8 bits | Even | 2 bits |
|SERIAL_5O1| 5 bits | Odd | 1 bit |
|SERIAL_6O1| 6 bits | Odd | 1 bit |
|SERIAL_7O1| 7 bits | Odd | 1 bit |
|SERIAL_8O1| 8 bits | Odd | 1 bit |
|SERIAL_5O2| 5 bits | Odd | 2 bits |
|SERIAL_6O2| 6 bits | Odd | 2 bits |
|SERIAL_7O2| 7 bits | Odd | 2 bits |
|SERIAL_8O2| 8 bits | Odd | 2 bits |

- Serial default device driver is "/dev/ttyAMA0". if you want to change the default device driver change the *SERIAL_DRIVER_NAME* constant as the example below. 

{% highlight c %}
#include <Arduino.h>

void setup() {
        strcpy(SERIAL_DRIVER_NAME, "/dev/ttyAMA1");
        Serial.begin(9600);
}

void loop() {
        Serial.println("Hello World");
        delay(1000);
}
{% endhighlight %}


