---
title:  "Wire.begin"
author: "Jorge Garza"
categories: function
sdate: "Aug 5, 2016"
---

### Wire.begin(driverName)

Initialize I2C as Master and will use the specified I2C device driver name.

**Parameters**

> **driverName** - Name of I2C device driver port. (e.g "/dev/i2c-1")

**Returns**

None

____________________

**Example**

{% highlight c %}
#include <Arduino.h>

void setup() {
	Wire.begin("/dev/i2c-0");
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

- Wire default device driver is "" (empty) meaning that it will search for any existent I2C device driver and will give priority to "/dev/i2c-1". if you want to change the default device driver change the *I2C_DRIVER_NAME* constant as the example below. 

{% highlight c %}
#include <Arduino.h>

void setup() {
        strcpy(I2C_DRIVER_NAME, "/dev/i2c-4");
        Wire.begin();  // Will try open device driver "/dev/i2c-4"
}

void loop() {
        ...
}
{% endhighlight %}




