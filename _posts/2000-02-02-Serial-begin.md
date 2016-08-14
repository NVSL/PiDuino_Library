---
title:  "Serial.begin"
author: "Jorge Garza"
categories: function
sdate: "Aug 5, 2016"
---

### Serial.begin(speed)

Opens a serial port at the specified speed (baud rate). 

**Parameters**

> **speed** - Serial port baud rate. (See Notes)

**Returns**

None

____________________

**Example**

{% highlight c %}
#include <Arduino.h>

void setup() {
	Serial.begin(9600);
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

- If you want to open a different Serial device driver port see the (EXTRA) function [Serial.begin(driverName, speed)]({{ site.baseurl }}{% post_url 2000-02-02-Serial-Extra-begin%}).

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




