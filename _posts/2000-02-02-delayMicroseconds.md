---
title:  "delayMicroseconds"
author: "Jorge Garza"
categories: function
sdate: "Aug 5, 2016"
---

### delayMicroseconds(us)

Pauses the program for the specified amount of time in microseconds.

**Parameters**

> **us** - Delay in microseconds. 

**Returns**

None

____________________

**Example**

{% highlight c %}
#include "Arduino.h"

void setup() {
}

void loop() {
	printf("Delay since start in millis %lu \n", millis());
	printf("Delay since start in micros %lu \n", micros());
	delayMicroseconds(1000000);	// Delay 1 second
	delay(1000);               	// Delay 1 second
}
{% endhighlight %}




