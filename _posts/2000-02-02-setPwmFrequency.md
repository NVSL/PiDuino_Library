---
title:  "setPwmFrequency"
author: "Jorge Garza"
categories: function
sdate: "Aug 5, 2016"
---

### setPwmFrequency(pin, frequency)

Sets the PWM (Pulse Width Modulation) wave frequency in Hz.

**Parameters**

> **pin** - The GPIO pin number.

> **frequency** - PWM frequency (in Hz). Available ranges are between 19 and 37500.

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
	// Set PWM to 100 Hz
	setPwmDutyCycle(GPIO_PIN, 100); 
	delay(1000);
	// Set PWM to 1 Khz
	setPwmDutyCycle(GPIO_PIN, 1000); 
	delay(1000);
	// Set PWM to 10 kHz
	setPwmDutyCycle(GPIO_PIN, 10000); 
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



