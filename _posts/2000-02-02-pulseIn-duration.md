---
title:  "pulseIn"
author: "Jorge Garza"
categories: function
sdate: "Aug 5, 2016"
---

### pulseIn(pin, value, timeout)

Measures the length of a pulse in microseconds. This function starts measuring when the specified input value (HIGH or LOW) is read. If value is HIGH for example, pulseIn() waits for the pin to go HIGH, starts timing, then waits for the pin to go LOW and stops timing. 

**Parameters**

> **pin** - The GPIO pin number.

> **value** - *HIGH* or *LOW*. The start input value to begin measuring. 

> **timeout** - In microseconds.

**Returns**

The length of the pulse in microseconds or 0 if no complete pulse was received within the given timeout.

____________________

**Example**

{% highlight c %}
// Ultrasonic sensor HC-SR04 read
#include "Arduino.h"

		    // Ultrasonic Vcc  pin to 5V
#define trigPin 23  // Ultrasonic Trig pin to GPIO23
#define echoPin 24  // Ultrasonic Echo pin <--^^1k^^--> GPIO24 <--^^2k^^--> GND
		    // Ultrasonic GND  pin to GND

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
	long long duration, distance;
	digitalWrite(trigPin, LOW); 
	delayMicroseconds(2);
	digitalWrite(trigPin, HIGH);
	delayMicroseconds(100);
	digitalWrite(trigPin, LOW);
	duration = pulseIn(echoPin, HIGH, 1000);
	distance = (duration/2) / 29.1;
	printf("duration = %d, Distance =  %d \n", duration, distance);
  	delay(500);
}
{% endhighlight %}





