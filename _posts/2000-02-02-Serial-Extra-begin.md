---
title:  "Serial.begin"
author: "Jorge Garza"
categories: function
sdate: "Aug 5, 2016"
---

### Serial.begin(driverName, speed)

Opens a serial port at the specified device driver name and  with the specified speed (baud rate). 

**Parameters**

> **driverName** - Serial port driver Name (e.g "/dev/ttyAMA0")

> **speed** - Serial port baud rate. (See Notes)

**Returns**

None

____________________

**Example**

{% highlight c %}
#include <Arduino.h>

void setup() {
	Serial.begin("/dev/ttyAMA0", 9600);
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




