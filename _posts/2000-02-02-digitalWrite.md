---
title:  "digitalWrite"
author: "Jorge Garza"
categories: function
sdate: "Aug 5, 2016"
---

### digitalWrite(pin, value)

Writes a HIGH or LOW value at the specified GPIO pin

**Parameters**

> **pin** - The GPIO pin number.

> **value** - Can be the followings:  
> *HIGH* : Outputs (3.3V).  
> *LOW* : Outputs (0V).  


**Returns**

None

____________________

**Example**

{% highlight c %}
#include <Arduino.h>

int ledPin = 4; // GPIO4

void setup() {
        pinMode(ledPin, OUTPUT);
}

void loop() {
        digitalWrite(ledPin, HIGH);
        delay(1000);
        digitalWrite(ledPin, LOW);
        delay(1000);
}
{% endhighlight %}




