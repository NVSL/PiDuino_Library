---
title:  "digitalWrite"
author: "Jorge Garza"
categories: function
sdate: "Aug 5, 2016"
---

### digitalRead(pin, value)

Reads a GPIO pin value. 

**Parameters**

> **pin** - The GPIO pin number.

**Returns**

*HIGH* (1) or *LOW* (0)

____________________

**Example**

{% highlight c %}
#include <Arduino.h>

// This program reads a pin and outputs
// its value at the specified output pin. 

int inputPin = 17; // GPIO17
int ledPin = 4; // GPIO4
int val = 0;

void setup() {
	pinMode(inputPin, INPUT);
	pinMode(ledPin, OUTPUT);
}

void loop() {
	val = digitalRead(inputPin);
	digitalWrite(ledPin);
}
{% endhighlight %}




