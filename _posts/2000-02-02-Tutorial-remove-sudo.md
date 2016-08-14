---
title:  "Getting rid of sudo"
author: "Jorge Garza"
categories: tutorial
sdate: "Aug 5, 2016"
---

### Getting rid of sudo

All accesses to hardware through device drivers should require root permission or sudo. But in raspbian there is a file that allows you to have access to some device drivers without permissions. This setting is located in the file **/etc/udev/rules.d/99-com.rules**. 

99-com.rules

```
SUBSYSTEM=="gpio*", PROGRAM="/bin/sh -c 'chown -R root:gpio /sys/class/gpio && chmod -R 770 /sys/class/gpio; chown -R root:gpio /sys/devices/virtual/gpio && chmod -R 770 /sys/devices/virtual/gpio; chown -R root:gpio /sys/devices/platform/soc/*.gpio/gpio && chmod -R 770 /sys/devices/platform/soc/*.gpio/gpio'"
SUBSYSTEM=="input", GROUP="input", MODE="0660"
SUBSYSTEM=="i2c-dev", GROUP="i2c", MODE="0660"
SUBSYSTEM=="spidev", GROUP="spi", MODE="0660"
SUBSYSTEM=="bcm2835-gpiomem", GROUP="gpio", MODE="0660"
```

The above file states that the following drives can be used without sudo
```
/sys/class/gpio ...
/dev/i2c-x
/dev/spidevx.x
/dev/gpiomem
```

This library also use "/dev/mem" to use the internal hardware PWM from the BCM283x SoC which is not listed by default in the *99-com.rules* file. 
To add this device driver add the following at the end of the file. 

```
sudo usermod -a -G kmem pi



