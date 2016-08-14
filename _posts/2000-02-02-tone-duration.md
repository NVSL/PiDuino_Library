---
title:  "tone"
author: "Jorge Garza"
categories: function
sdate: "Aug 5, 2016"
---

### tone(pin, frequency, duration)

Generates a PWM wave at the specified pin and frequency with a duty cycle of (50%). The generated wave will stop after the specified duration (in milliseconds). This is a **non-blocking** function meaning that it will continue executing the rest of the functions. if you want it to block it for the specified duration use the (EXTRA) function [tone(pin, frequency, duration, block)]({{ site.baseurl }}{% post_url 2000-02-02-tone-duration-block%}). 

**Parameters**

> **pin** - The GPIO pin number.

> **frequency** - PWM frequency (in hz). 
Available ranges are between 19 and 37500.

> **duration** - Duration of the tone in milliseconds.

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
	// Set the tone to 1 KHz for 1 second
	tone(GPIO_PIN, 1000, 1000);
	// Delay 2 seconds
	delay(2000);
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




