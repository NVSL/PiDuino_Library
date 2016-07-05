/*
  Arduino.h - Arduino for linux (LinuxDuino) main loop

  Copyright (c) 2016 Jorge Garza <jgarzagu@ucsd.edu>

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef Arduino_h
#define Arduino_h

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include "Core.h"
#include "Serial.h"
#include "WString.h"

void setup(void);
void loop(void);

int main(void)
{	
	setup();
    
	for (;;) {
		loop();
	}
        
	return 0;
}

#endif