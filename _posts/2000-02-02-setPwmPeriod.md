---
title:  "setPwmPeriod"
author: "Jorge Garza"
categories: function
sdate: "Aug 5, 2016"
---

### setPwmPeriod(pin, microseconds)

Sets the PWM (Pulse Width Modulation) wave period in microseconds (us).

**Parameters**

> **pin** - The GPIO pin number.

> **microseconds** - PWM frequency (in us).  
Available ranges are between 52631 (19 hz) and 26 (37500 hz).

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
	// Set PWM to 10000 us (100 Hz)
	setPwmDutyCycle(GPIO_PIN, 10000); 
	delay(1000);
	// Set PWM to 1000 us (1 KHz)
	setPwmDutyCycle(GPIO_PIN, 1000); 
	delay(1000);
	// Set PWM to 100 us (10 kHz)
	setPwmDutyCycle(GPIO_PIN, 100); 
	delay(1000);
}
{% endhighlight %}

____________________

**Notes**

- The default PWM frequency is 490 Hz
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




