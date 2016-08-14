---
title:  "setPwmDutyCycle"
author: "Jorge Garza"
categories: function
sdate: "Aug 5, 2016"
---

### setPwmDutyCycle(pin, dutycycle)

Writes a PWM (Pulse Width Modulation) wave at the output of the selected pin.

**Parameters**

> **pin** - The GPIO pin number.

> **dutycycle** - PWM duty cycle. Available ranges between 0 and 255.

**Returns**

None

____________________

**Example**

{% highlight c %}
#include <Arduino.h>

int GPIO_PIN = 12; // GPIO12

void setup() {
	pinMode(GPIO_PIN, PWM_OUTPUT);
	setPwmDutyCycle(GPIO_PIN, 128); // Set PWM to half duty cycle
}

void loop() {
	
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




