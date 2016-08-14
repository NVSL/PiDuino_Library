---
title:  "Serial.begin"
author: "Jorge Garza"
categories: function
sdate: "Aug 5, 2016"
---

### Serial.printf(format, ...)

Writes to the serial port using the same format as the printf() function. 

**Parameters**

> **format** - character array constant. (e.g "Hello")

> **...** - Arguments 

**Returns**

Number of bytes written. 

____________________

**Example**

{% highlight c %}
#include <Arduino.h>

int i = 0;

void setup() {
	Serial.begin(9600);
}

void loop() {
	Serial.printf("Hello World %d \n\r", i++);
	delay(1000);
}
{% endhighlight %}

____________________

**Notes**

- Most serial terminals require the carrier return "\r" together with the new line feed "\n" to print a new line. 