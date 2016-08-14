---
title:  "Wire.begin"
author: "Jorge Garza"
categories: function
sdate: "Aug 5, 2016"
---

### Wire.begin()

Initialize I2C as Master.

**Parameters**

None

**Returns**

None

____________________

**Example**

{% highlight c %}
#include <Arduino.h>
#include <Wire.h>

void setup() {
	Wire.begin();
}

void loop() {

	// Send two bytes to an I2C slave device with address 0x70
	Wire.beginTransmission(0x70);
	Wire.write(42);
	Wire.write(42);
	Wire.endTransmission();

	// Request 6 bytes from an I2C slave device with address 0x70
	Wire.requestFrom(0x70, 6);
	// Wait for data from I2C slave device
	while (Wire.available()) { 	  
		char c = Wire.read(); 		  
		printf("Recieved = %c \n", c);
	}

}
{% endhighlight %}

____________________

**Notes**

- If you want to open a different I2C device driver port see the (EXTRA) function [Wire.begin(driverName)]({{ site.baseurl }}{% post_url 2000-02-02-Wire-Extra-begin%}).

- Wire default device driver is "" (empty) meaning that it will search for any existent I2C device driver and will give priority to "/dev/i2c-1". if you want to change the default device driver change the *I2C_DRIVER_NAME* constant as the example below. 

{% highlight c %}
#include <Arduino.h>
#include <Wire.h>

void setup() {
        strcpy(I2C_DRIVER_NAME, "/dev/i2c-4");
        Wire.begin(); // Will try open device driver "/dev/i2c-4"
}

void loop() {
        ...
}
{% endhighlight %}




