---
title:  "tone"
author: "Jorge Garza"
categories: function
sdate: "Aug 5, 2016"
---

### tone(pin, frequency, duration, block)

Generates a PWM wave at the specified pin and frequency with a duty cycle of (50%). The generated wave will stop after the specified duration (in milliseconds). You can setup the function to block (true) or to be non-blocking (false) for the specified duration.  

**Parameters**

> **pin** - The GPIO pin number.

> **frequency** - PWM frequency (in hz). 
Available ranges are between 19 and 37500.

> **duration** - Duration of the tone in milliseconds.

> **block** - True to block the function or false to make the function be non-blocking.

**Returns**

None

____________________

**Example**

{% highlight c %}
#include <Arduino.h>

int GPIO_PIN = 12; // GPIO12

void setup() {
	pinMode(GPIO_PIN, PWM_OUTPUT);
}

void loop() {
	// Set the tone to 1 KHz and blocks for 1 second 
	tone(GPIO_PIN, 1000, 1000, true);
	// Set the tone to 2 KHz and blocks for 1 second 
	tone(GPIO_PIN, 2000, 1000, true);
	// Set the tone to 3 KHz and blocks for 1 second 
	tone(GPIO_PIN, 3000, 1000, true);
	// Set the tone to 4 KHz and blocks for 1 second 
	tone(GPIO_PIN, 4000, 1000, true);
}
{% endhighlight %}

____________________

**Notes**

- *PWM_OUTPUT* mode uses the internal PWM hardware of the BCM283x SoC.  
The available PWM pins are the followings:

| GPIO PIN | Available in RPi (40-pins) connector | PWM channel |
|:------:|:-----------:|:-----------:|
|   12   |      YES |   0 |
|   13   |      YES |   1 |
|   18   |      YES |   0 |
|   19   |      YES |   1 |
|   40   |      NO  |   0 |
|   41   |      NO  |   1 |
|   45   |      NO  |   1 |
|   52   |      NO  |   0 |
|   53   |      NO  |   1 |




