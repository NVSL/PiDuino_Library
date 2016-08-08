ACLOCAL_AMFLAGS = -I m4

# PiDuino Library
lib_LTLIBRARIES = libpiduino.la
libpiduino_la_SOURCES =  Core.cpp  Serial.cpp  SPI.cpp  Wire.cpp  WString.cpp 
include_HEADERS = Arduinoh/Arduino.h piDuino.h Core.h Serial.h SPI.h Wire.h WString.h
libpiduino_la_LDFLAGS = -no-undefined -version-info 1:0:0
libpiduino_la_LIBADD = -lpthread -lm