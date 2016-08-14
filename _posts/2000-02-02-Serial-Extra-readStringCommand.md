---
title:  "Serial.readStringCommand"
author: "Jorge Garza"
categories: function
sdate: "Aug 5, 2016"
---

### Serial.readStringCommand(character, buffer, length) 

Saves all received characters in a buffer until the specified character is received. This function is ideal to receive serial commands from a terminal or another device. 

**Parameters**

> **format** - character array constant. (e.g "Hello")

> **...** - Arguments 

**Returns**

Number of bytes written. 

____________________

**Example**

{% highlight c %}
#include <Arduino.h>

// This program try to imitate a shell terminal. Echo must be disabled. 

void setup() {
        Serial.begin(9600);
}

void loop() {
  char data[50];
  memset(data, 0, sizeof(data));

  // Reads inputs characters until enter ("\r\n") is hit.
  Serial.readStringCommand('\r', data, sizeof(data)); // Waits here

  printf("Recieved = %s \n", data);
  Serial.printf("$ %s \r\n", data);
}
{% endhighlight %}

____________________

**Notes**

- Most serial terminals require the carrier return "\r" together with the new line feed "\n" to print a new line. 