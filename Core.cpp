/*
  Core.cpp - piDuino Digital, PWM and Arduino functions

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

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/mman.h>
#include <string.h>
#include <time.h>
#include <ctype.h>
#include <pthread.h>
#include <poll.h>
#include <linux/types.h>
#include <unistd.h>
#include <pthread.h>
#include "Core.h"


/* 
    ### BCM283x SoC Notes for GPIO and PWM ###
    - BCM283x SoCs have a maximum of 54 GPIOS
    - BCM283x SoCs have 2 independent PWM channels (0 and 1) that uses
    the same PWM clock as the base frequency. The following are the
    available PWM pins for this chip:
    GPIO PIN   RPi2 pin   PWM Channel  ALT FUN
    12         YES        0            0
    13         YES        1            0
    18         YES        0            5
    19         YES        1            5
    40                    0            0
    41                    1            0
    45                    1            0
    52                    0            1
    53                    1            1
*/


/////////////////////////////////////////////
//          Digital I/O                   //
////////////////////////////////////////////

// -- Digital I/O --
// BCM2708 Registers for GPIO (Do not put them in .h)
#define BCM2708_PERI_BASE   0x20000000
#define GPIO_BASE           (BCM2708_PERI_BASE + 0x200000)
#define OFFSET_FSEL         0   // 0x0000
#define OFFSET_SET          7   // 0x001c / 4
#define OFFSET_CLR          10  // 0x0028 / 4
#define OFFSET_PINLEVEL     13  // 0x0034 / 4
#define OFFSET_PULLUPDN     37  // 0x0094 / 4
#define OFFSET_PULLUPDNCLK  38  // 0x0098 / 4
#define GPIO_FSEL_INPUT     0   // Pin Input mode
#define GPIO_FSEL_OUTPUT    1   // Pin Output mode
#define PAGE_SIZE  (4*1024)
#define BLOCK_SIZE (4*1024)
static volatile uint32_t *gpio_map = NULL;
static bool g_open_gpiomem_flag = false;
int g_gpio_pin_set[SOC_GPIO_PINS];           // Used to know which gpio pins are set (HIGH) or not set (LOW) 
char GPIO_DRIVER_NAME[128] = "/dev/gpiomem";

// -- Analog I/O --
#define BCM2708_PERI_BASE   0x20000000
#define PWM_BASE           (BCM2708_PERI_BASE + 0x20C000)
#define CLOCK_BASE         (BCM2708_PERI_BASE + 0x101000)
#define PWMCLK_CNTL         40
#define PWMCLK_DIV          41
#define PWM_CONTROL         0
#define PWM0_RANGE          4
#define PWM0_DATA           5
#define PWM1_RANGE          8
#define PWM1_DATA           9
#define GPIO_FSEL_ALT0      4  
#define GPIO_FSEL_ALT1      5  
#define GPIO_FSEL_ALT5      2  
static volatile uint32_t *pwm_map = NULL;
static volatile uint32_t *clk_map = NULL;
static bool g_open_pwmmem_flag = false;
int g_pwm_pin_set[SOC_GPIO_PINS];               // Used to know which pwm pins are set (HIGH) or not set (LOW)
int g_pwm_dutycycle_value[SOC_GPIO_PINS];       // Pwm duty cycle value of pwm pins
uint32_t PWM_DUTYCYCLE_RESOLUTION = 256;        // Set pwm duty cycle resolution between 0 and 255 bits
uint32_t PWM_DEFAULT_FREQUENCY = 490;           // Set default pwm frequency to 490 Hz (Arduino default pwm freq)
char PWM_DRIVER_NAME[128] = "/dev/mem";

// Sets pin (gpio) mode as INPUT,INTPUT_PULLUP,INTPUT_PULLDOWN,OUTPUT,PWM_OUTPUT
void pinMode(uint8_t pin, uint8_t mode)
{
    int mem_fd, pwm_mem_fd;
    int clk_offset = OFFSET_PULLUPDNCLK + (pin/32);
    int shift_offset = (pin%32);
    int offset = OFFSET_FSEL + (pin/10);
    int shift = (pin%10)*3;
    int gpio_fsel_alt = 0;
    int pwm_channel = 0;

    // Check if the pin number is valid
    if (pin >= SOC_GPIO_PINS) {
        fprintf(stderr, "%s(): pin number should be less than "
            "%d, yours is %d \n", __func__, SOC_GPIO_PINS, pin);
        exit(1);
    }

    // Initialize gpiomem only once
    if (g_open_gpiomem_flag == false) {
        
        if ((mem_fd = open(GPIO_DRIVER_NAME, O_RDWR|O_SYNC) ) < 0) {
            fprintf(stderr, "%s(): gpio driver %s: %s\n",__func__, 
                GPIO_DRIVER_NAME, strerror (errno));
            exit(1);
        }

        gpio_map = (uint32_t *)mmap( NULL, BLOCK_SIZE, 
            PROT_READ|PROT_WRITE|PROT_EXEC, MAP_SHARED|MAP_LOCKED, mem_fd, GPIO_BASE);

        if (gpio_map < 0) {
            fprintf(stderr, "%s(): gpio error: %s\n",__func__, strerror (errno));
            exit(1);
        }

        // gpio memory mapping initialized correctly
        g_open_gpiomem_flag = true;
    }

    // Initialize different pin mode options
    if (mode == INPUT || mode == INPUT_PULLUP 
        || mode == INPUT_PULLDOWN || mode == OUTPUT) {

        // Save gpio pin number so at program close we put it to default state
        // Also clear pwm pin to prevent any errro if different pin mode is set multiple times
        g_gpio_pin_set[pin] = HIGH;
        g_pwm_pin_set[pin] = LOW;

        // Set resistor mode PULLUP, PULLDOWN or PULLOFF resistor (OUTPUT always PULLOFF)
        if (mode == INPUT_PULLDOWN) {
           *(gpio_map+OFFSET_PULLUPDN) = (*(gpio_map+OFFSET_PULLUPDN) & ~3) | 0x01;
        } else if (mode == INPUT_PULLUP) {
           *(gpio_map+OFFSET_PULLUPDN) = (*(gpio_map+OFFSET_PULLUPDN) & ~3) | 0x02;
        } else { // mode == PULLOFF
           *(gpio_map+OFFSET_PULLUPDN) &= ~3;
        }
        usleep(1);
        *(gpio_map+clk_offset) = 1 << shift_offset;
        usleep(1);
        *(gpio_map+OFFSET_PULLUPDN) &= ~3;
        *(gpio_map+clk_offset) = 0;

        // Set pin mode INPUT/OUTPUT
        if (mode == OUTPUT) {
            *(gpio_map+offset) = (*(gpio_map+offset) & ~(7<<shift)) | (GPIO_FSEL_OUTPUT<<shift);
        } else { // mode == INPUT or INPUT_PULLUP or INPUT_PULLDOWN
            *(gpio_map+offset) = (*(gpio_map+offset) & ~(7<<shift)) | (GPIO_FSEL_INPUT<<shift);
        }

    } else if(mode == PWM_OUTPUT) {

        // Check if the pin is compatible for PWM and assign its channel
        switch (pin) {
            case 12: pwm_channel = 0; gpio_fsel_alt = GPIO_FSEL_ALT0; break;
            case 13: pwm_channel = 1; gpio_fsel_alt = GPIO_FSEL_ALT0; break;
            case 18: pwm_channel = 0; gpio_fsel_alt = GPIO_FSEL_ALT5; break;
            case 19: pwm_channel = 1; gpio_fsel_alt = GPIO_FSEL_ALT5; break;
            case 40: pwm_channel = 0; gpio_fsel_alt = GPIO_FSEL_ALT0; break;
            case 41: pwm_channel = 1; gpio_fsel_alt = GPIO_FSEL_ALT0; break;
            case 45: pwm_channel = 1; gpio_fsel_alt = GPIO_FSEL_ALT0; break;
            case 52: pwm_channel = 0; gpio_fsel_alt = GPIO_FSEL_ALT1; break;
            case 53: pwm_channel = 1; gpio_fsel_alt = GPIO_FSEL_ALT1; break;
            default:
                fprintf(stderr, "%s(): pin %d can not be set as PWM_OUTPUT\n", __func__, pin);
                exit(1);
                break;
        }

        // Initialize mem only once (this requires sudo)
        if (g_open_pwmmem_flag == false) {

            if ((pwm_mem_fd = open(PWM_DRIVER_NAME, O_RDWR|O_SYNC) ) < 0) {
                fprintf(stderr, "%s(): pwm driver %s: %s\n",__func__, 
                    PWM_DRIVER_NAME, strerror (errno));
                exit(1);
            }

            pwm_map = (uint32_t *)mmap(NULL, BLOCK_SIZE, 
                PROT_READ|PROT_WRITE|PROT_EXEC, MAP_SHARED|MAP_LOCKED, pwm_mem_fd, PWM_BASE);

            if (pwm_map < 0) {
                fprintf(stderr, "%s(): pwm error: %s\n", __func__, strerror (errno));
                exit(1);
            }

            clk_map = (uint32_t *)mmap(NULL, BLOCK_SIZE, 
                PROT_READ|PROT_WRITE|PROT_EXEC, MAP_SHARED|MAP_LOCKED, pwm_mem_fd, CLOCK_BASE);
            
            if (clk_map < 0) {
                fprintf(stderr, "%s(): pwm error: %s\n", __func__, strerror (errno));
                exit(1);
            }

            // pwm memory mapping initialized correctly
            g_open_pwmmem_flag = true;
        }

        // Save pwm pin number so at program close we put it to default state
        // Also clear gpio pin to prevent any errro if different pin mode is set multiple times
        g_pwm_pin_set[pin] = HIGH;
        g_gpio_pin_set[pin] = LOW;

        // Set pin to its corresponding ALT mode or (PWM MODE)
        *(gpio_map+offset) = 
            (*(gpio_map+offset) & ~(7 << shift)) | ((gpio_fsel_alt << shift) & (7 << shift));

        // Set frequency to default Arduino frequency (490Hz) and duty cycle value to zero
        setPwmFrequency(pin, PWM_DEFAULT_FREQUENCY, 0);

        // Ser PWM range to default of 256 bits of resolution
        if (pwm_channel == 1) {
            *(pwm_map + PWM1_RANGE) = PWM_DUTYCYCLE_RESOLUTION;
        } else {
            *(pwm_map + PWM0_RANGE) = PWM_DUTYCYCLE_RESOLUTION;
        }

        // Set PWM in MARKSPACE MODE and Enable PWM 
        if (pwm_channel == 1) {
            *(pwm_map + PWM_CONTROL) |= ( 0x8000 | 0x0100 );  // (PWM1_MS_MODE | PWM1_ENABLE )
        } else {
            *(pwm_map + PWM_CONTROL) |= ( 0x0080 | 0x0001 );  // (PWM0_MS_MODE | PWM0_ENABLE )
        }

        } else {
            fprintf(stderr, "%s(): pin mode %d is not an available mode \n", __func__, mode);
            exit(1);
        }

}

// Sets a pin (gpio) output to 1 or 0
void digitalWrite(uint8_t pin, uint8_t val)
{
    int offset;

    // Check if the pin number is valid
    if (pin >= SOC_GPIO_PINS) {
        fprintf(stderr, "%s(): pin number should be less than "
            "%d, yours is %d \n", __func__, SOC_GPIO_PINS, pin);
        exit(1);
    }

    // Check if pin has been initialized 
    if (g_gpio_pin_set[pin] != HIGH) {
        fprintf(stderr, "%s(): please initialize pin %d first "
            "using pinMode() function \n",__func__, pin);
        exit(1);
    }

    if (val) { // value == HIGH
        offset = OFFSET_SET + (pin / 32);
    } else {    // value == LOW
        offset = OFFSET_CLR + (pin / 32);
    }
    *(gpio_map+offset) = 1 << pin % 32;
}

// Returns the value of a pin (gpio) input (1 or 0)
int digitalRead(uint8_t pin)
{
    int offset, value, mask;

    // Check if the pin number is valid
    if (pin >= SOC_GPIO_PINS) {
        fprintf(stderr, "%s(): pin number should be less than "
            "%d, yours is %d \n", __func__, SOC_GPIO_PINS, pin);
        exit(1);
    }

    // Check if pin has been initialized 
    if (g_gpio_pin_set[pin] != HIGH) {
        fprintf(stderr, "%s(): please initialize pin %d first "
            "using pinMode() function \n",__func__, pin);
        exit(1);
    }


    offset = OFFSET_PINLEVEL + (pin/32);
    mask = (1 << pin%32);
    value = *(gpio_map+offset) & mask;
    return (value) ? HIGH : LOW;
}

/////////////////////////////////////////////
//          Analog I/O                    //
////////////////////////////////////////////


// Changes the duty Cycle of the PWM
void analogWrite(uint8_t pin, uint32_t value) 
{
    int pwm_channel = 0;

    // Check if pin has been initialized
    if (g_pwm_pin_set[pin] != HIGH) {
        fprintf(stderr, "%s(): please initialize pin %d first "
            "using pinMode() function \n",__func__, pin);
        exit(1);
    } else {
        // Check if the pin is valid for PWM and assign its channel
        switch (pin) {
            case 12: pwm_channel = 0; break;
            case 13: pwm_channel = 1; break;
            case 18: pwm_channel = 0; break;
            case 19: pwm_channel = 1; break;
            case 40: pwm_channel = 0; break;
            case 41: pwm_channel = 1; break;
            case 45: pwm_channel = 1; break;
            case 52: pwm_channel = 0; break;
            case 53: pwm_channel = 1; break;
            default:
                fprintf(stderr, "%s(): pin %d can not be assigned for "
                 "analogWrite() with PWM_OUTPUT mode\n",__func__, pin);
                exit(1);
                break;
        }
    }

    // Check if duty cycle resolution match
    if (value >= PWM_DUTYCYCLE_RESOLUTION) {
        fprintf(stderr, "%s(): dutycycle %d should be less than the "
            "max pwm resolution = %d \n",
            __func__, value, PWM_DUTYCYCLE_RESOLUTION);
            exit(1);
    }


    // Set PWM0 Duty Cycle Value
    g_pwm_dutycycle_value[pin] = value;
    if (pwm_channel == 1) {
        *(pwm_map + PWM1_DATA) = g_pwm_dutycycle_value[pin];
    } else {
        *(pwm_map + PWM0_DATA) = g_pwm_dutycycle_value[pin];
    }

}

// Does the same as anaogWrite but the function name makes more sense.
void setPwmDutyCycle (uint8_t pin, uint32_t dutycycle)
{
    analogWrite(pin, dutycycle);
}

void setPwmPeriod (uint8_t pin, uint32_t microseconds) 
{
    setPwmFrequency(pin, (1000000 / microseconds), g_pwm_dutycycle_value[pin]);
}

void setPwmFrequency (uint8_t pin, uint32_t frequency) 
{
    setPwmFrequency(pin, frequency, g_pwm_dutycycle_value[pin]);
}

// Sets PWM frequency (in Hertz) and pwm duty cycle
void setPwmFrequency (uint8_t pin, uint32_t frequency, uint32_t dutycycle) 
{
    int pwm_channel = 0;
    int divisor;
    double period;
    double countDuration;

    if (g_pwm_pin_set[pin] != HIGH) {
        fprintf(stderr, "%s(): please initialize pin %d first "
            "using pinMode() function \n",__func__, pin);
        exit(1);
    }

    // Check if the pin is valid for PWM and assign its channel
    switch (pin) {
        case 12: pwm_channel = 0; break;
        case 13: pwm_channel = 1; break;
        case 18: pwm_channel = 0; break;
        case 19: pwm_channel = 1; break;
        case 40: pwm_channel = 0; break;
        case 41: pwm_channel = 1; break;
        case 45: pwm_channel = 1; break;
        case 52: pwm_channel = 0; break;
        case 53: pwm_channel = 1; break;
        default:
            fprintf(stderr, "%s(): pin %d can not be assigned for "
                "this function with PWM_OUTPUT mode \n",__func__, pin);
            exit(1);
            break;
    }

    // Check if duty cycle resolution match
    if (dutycycle >= PWM_DUTYCYCLE_RESOLUTION) {
        fprintf(stderr, "%s(): duty cycle %d should be less than the "
            "max pwm resolution = %d \n",
            __func__, dutycycle, PWM_DUTYCYCLE_RESOLUTION);
            exit(1);
    }

    // -- Set frequency and duty cycle

    // stop clock and waiting for busy flag doesn't work, so kill clock
    *(clk_map + PWMCLK_CNTL) = 0x5A000000 | 0x01;
    usleep(10);

    // wait until busy flag is set 
    while ( (*(clk_map + PWMCLK_CNTL)) & 0x80);

    //calculate divisor value for PWM1 clock...base frequency is 19.2MHz
    period = 1.0/frequency; 
    countDuration = period/(PWM_DUTYCYCLE_RESOLUTION*1.0f);
    divisor = (int)(19200000.0f / (1.0/countDuration));

    if( divisor < 0 || divisor > 4095 ) {
        fprintf(stderr, "%s(): pwm frequency %d with pwm duty cycle "
            "resolution/range of %d bits not supported \n",__func__,
             frequency, PWM_DUTYCYCLE_RESOLUTION);
        exit(-1);
    }

    // Set divisor
    *(clk_map + PWMCLK_DIV) = 0x5A000000 | (divisor << 12);

    // source=osc and enable clock
    *(clk_map + PWMCLK_CNTL) = 0x5A000011;

    // Set PWM0 Duty Cycle pin Value to zero
    g_pwm_dutycycle_value[pin] = dutycycle;
    if (pwm_channel == 1) {
        *(pwm_map + PWM1_DATA) = g_pwm_dutycycle_value[pin];
    } else {
        *(pwm_map + PWM0_DATA) = g_pwm_dutycycle_value[pin];
    }

}


/////////////////////////////////////////////
//          Advanced I/O                  //
////////////////////////////////////////////

// Arguments for Tone threads
struct ThreadToneArg {
    int pin;
    unsigned long duration;
};

pthread_t idToneThread[SOC_GPIO_PINS];

// This is function will be running in a thread if
// non-blocking tone() is called.
void * toneThreadFunction(void *args)
{
    ThreadToneArg *arguments = (ThreadToneArg *)args;
    int pin = arguments->pin;
    unsigned long duration = arguments->duration;

    usleep(duration*1000);
    noTone(pin);

    return (NULL);
}


// Set tone frequency (in hertz) and duration (in milliseconds)
void tone(uint8_t pin, uint32_t frequency, unsigned long duration, uint32_t block)
{
    pthread_t *threadId;
    struct ThreadToneArg *threadArgs;

    // Set frequency at 50% duty cycle
    setPwmFrequency(pin, frequency, PWM_DUTYCYCLE_RESOLUTION / 2);

    // Tone duration: If duration == 0, don't stop the tone, 
    // else perform duration either blocking or non-blocking
    if (duration == 0) {
        return;
    } else {

        threadId = &idToneThread[SOC_GPIO_PINS];
        threadArgs = (ThreadToneArg *)malloc(sizeof(ThreadToneArg));
        threadArgs->pin = pin;
        threadArgs->duration = duration;

        // Cancel any existent threads for the pwm pin
        if (*threadId != 0) {
            pthread_cancel(*threadId);
        }

        // If block == true  stop the tone after a sleep delay
        // If block == false then start a thread that will stop the tone
        // after certain duration and parallely continue with the rest of the func. 
        if  (block) {
            usleep(duration*1000);
            noTone(pin);
        } else {
            pthread_create (threadId, NULL, toneThreadFunction, (void *)threadArgs);
        }
    }
}

void noTone(uint8_t pin) {
    analogWrite(pin, 0);
}

uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) 
{
    uint8_t value = 0;
    uint8_t i;

    for (i = 0; i < 8; ++i) {
        digitalWrite(clockPin, HIGH);
        if (bitOrder == LSBFIRST)
            value |= digitalRead(dataPin) << i;
        else
            value |= digitalRead(dataPin) << (7 - i);
        digitalWrite(clockPin, LOW);
    }
    return value;
}


void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val)
{
    uint8_t i;

    for (i = 0; i < 8; i++)  {
        if (bitOrder == LSBFIRST)
            digitalWrite(dataPin, !!(val & (1 << i)));
        else    
            digitalWrite(dataPin, !!(val & (1 << (7 - i))));
            
        digitalWrite(clockPin, HIGH);
        digitalWrite(clockPin, LOW);        
    }
}

// Measures the length (in microseconds) of a pulse on the pin; state is HIGH
// or LOW, the type of pulse to measure. timeout is 1 second by default.
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout)
{
    struct timespec start, end;
    clock_gettime(CLOCK_REALTIME, &start);

    // wait for any previous pulse to end
    while (digitalRead(pin) == state) {
        clock_gettime(CLOCK_REALTIME, &end);
        // timeDiffmicros > timeout?
        if (((end.tv_sec - start.tv_sec) * 1e6  + 
            (end.tv_nsec - start.tv_nsec) * 1e-3) > timeout)
            return 0;
    }

    // wait for the pulse to start
    while (digitalRead(pin) != state) {
        clock_gettime(CLOCK_REALTIME, &end);
         // timeDiffmicros > timeout?
        if (((end.tv_sec - start.tv_sec) * 1e6  + 
            (end.tv_nsec - start.tv_nsec) * 1e-3) > timeout)
            return 0;
    }

    clock_gettime(CLOCK_REALTIME, &start);
    // wait for the pulse to stop
    while (digitalRead(pin) == state) {
        clock_gettime(CLOCK_REALTIME, &end);
         // timeDiffmicros > timeout?
        if (((end.tv_sec - start.tv_sec) * 1e6  + 
            (end.tv_nsec - start.tv_nsec) * 1e-3) > timeout)
            return 0;
    }

    // return microsecond elapsed
    return ((end.tv_sec - start.tv_sec) * 1e6  + 
            (end.tv_nsec - start.tv_nsec) * 1e-3);
}


/////////////////////////////////////////////
//          Time                          //
////////////////////////////////////////////

// Returns the time in milliseconds since the program started.
unsigned long millis(void) 
{
    struct timespec timenow, start, end;
    clock_gettime(CLOCK_REALTIME, &timenow);
    start = Arduino.timestamp;
    end = timenow;
    // timeDiffmillis:
    return ((end.tv_sec - start.tv_sec) * 1e3 + (end.tv_nsec - start.tv_nsec) * 1e-6);
}

// Returns the time in microseconds since the program started.
unsigned long micros(void)
{
    struct timespec timenow, start, end;
    clock_gettime(CLOCK_REALTIME, &timenow);
    start = Arduino.timestamp;
    end = timenow;
    // timeDiffmicros
    return ((end.tv_sec - start.tv_sec) * 1e6 + (end.tv_nsec - start.tv_nsec) * 1e-3);
}

// Sleep the specified milliseconds
void delay(unsigned long millis)
{
    usleep(millis*1000);
}

// Sleep the specified microseconds
void delayMicroseconds(unsigned int us)
{
    usleep(us);
}

/////////////////////////////////////////////
//          Math                          //
////////////////////////////////////////////

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/////////////////////////////////////////////
//          Characters                    //
////////////////////////////////////////////

// Checks for an alphanumeric character. 
// It is equivalent to (isalpha(c) || isdigit(c)).
boolean isAlphaNumeric(int c) 
{
  return ( isalnum(c) == 0 ? false : true);
}

// Checks for an alphabetic character. 
// It is equivalent to (isupper(c) || islower(c)).
boolean isAlpha(int c)
{
  return ( isalpha(c) == 0 ? false : true);
}

// Checks whether c is a 7-bit unsigned char value 
// that fits into the ASCII character set.
boolean isAscii(int c)
{
  return ( isascii (c) == 0 ? false : true);
}

// Checks for a blank character, that is, a space or a tab.
boolean isWhitespace(int c)
{
  return ( isblank (c) == 0 ? false : true);
}

// Checks for a control character.
boolean isControl(int c)
{
  return ( iscntrl (c) == 0 ? false : true);
}

// Checks for a digit (0 through 9).
boolean isDigit(int c)
{
  return ( isdigit (c) == 0 ? false : true);
}

// Checks for any printable character except space.
boolean isGraph(int c)
{
  return ( isgraph (c) == 0 ? false : true);
}

// Checks for a lower-case character.
boolean isLowerCase(int c)
{
  return (islower (c) == 0 ? false : true);
}

// Checks for any printable character including space.
boolean isPrintable(int c)
{
  return ( isprint (c) == 0 ? false : true);
}

// Checks for any printable character which is not a space 
// or an alphanumeric character.
boolean isPunct(int c)
{
  return ( ispunct (c) == 0 ? false : true);
}

// Checks for white-space characters. For the avr-libc library, 
// these are: space, formfeed ('\f'), newline ('\n'), carriage 
// return ('\r'), horizontal tab ('\t'), and vertical tab ('\v').
boolean isSpace(int c)
{
  return ( isspace (c) == 0 ? false : true);
}

// Checks for an uppercase letter.
boolean isUpperCase(int c)
{
  return ( isupper (c) == 0 ? false : true);
}

// Checks for a hexadecimal digits, i.e. one of 0 1 2 3 4 5 6 7 
// 8 9 a b c d e f A B C D E F.
boolean isHexadecimalDigit(int c)
{
  return ( isxdigit (c) == 0 ? false : true);
}

// Converts c to a 7-bit unsigned char value that fits into the 
// ASCII character set, by clearing the high-order bits.
int toAscii(int c)
{
  return toascii (c);
}

// Converts the letter c to lower case, if possible.
int toLowerCase(int c)
{
  return tolower (c);
}

// Converts the letter c to upper case, if possible.
int toUpperCase(int c)
{
  return toupper (c);
}

/////////////////////////////////////////////
//          Random Functions              //
////////////////////////////////////////////

void randomSeed(unsigned long seed)
{
    if (seed != 0) {
        srandom(seed);
    }
}

long random(long howbig)
{
    if (howbig == 0) {
        return 0;
    }
    return random() % howbig;
}

long random(long howsmall, long howbig)
{
  if (howsmall >= howbig) {
    return howsmall;
  }
  long diff = howbig - howsmall;
  return random(diff) + howsmall;
}

/////////////////////////////////////////////
//          External Interrupts           //
////////////////////////////////////////////

// Arguments for External Interrupt threads
struct ThreadExtArg {
    void (*func)();
    int pin;
};

pthread_t idExtThread[SOC_GPIO_PINS];

// This is the function that will be running in a thread if
// attachInterrupt() is called 
void * threadFunction(void *args)
{
    ThreadExtArg *arguments = (ThreadExtArg *)args;
    int pin = arguments->pin;
    
    int GPIO_FN_MAXLEN = 32;
    int RDBUF_LEN = 5;
    
    char fn[GPIO_FN_MAXLEN];
    int fd,ret;
    struct pollfd pfd;
    char rdbuf [RDBUF_LEN];
    
    memset(rdbuf, 0x00, RDBUF_LEN);
    memset(fn,0x00,GPIO_FN_MAXLEN);
    
    snprintf(fn, GPIO_FN_MAXLEN-1, "/sys/class/gpio/gpio%d/value",pin);
    fd=open(fn, O_RDONLY);
    if (fd<0) {
        fprintf(stderr, "%s(): gpio error: %s\n", __func__, strerror (errno));
        exit(1);
    }
    pfd.fd=fd;
    pfd.events=POLLPRI;
    
    ret=read(fd,rdbuf,RDBUF_LEN-1);
    if (ret<0) {
        fprintf(stderr, "%s(): gpio error: %s\n", __func__, strerror (errno));
        exit(1);
    }
    
    while(1) {
        memset(rdbuf, 0x00, RDBUF_LEN);
        lseek(fd, 0, SEEK_SET);
        ret=poll(&pfd, 1, -1);
        if (ret<0) {
            fprintf(stderr, "%s(): gpio error: %s\n", __func__, strerror (errno));
            close(fd);
            exit(1);
        }
        if (ret==0) {
            // Timeout
            continue;
        }
        ret=read(fd,rdbuf,RDBUF_LEN-1);
        if (ret<0) {
            fprintf(stderr, "%s(): gpio error: %s\n", __func__, strerror (errno));
            exit(1);
        }
        //Interrupt. We call user function.
        arguments->func();
    }
}

void attachInterrupt(uint8_t pin, void (*f)(void), int mode)
{
    pthread_t *threadId = &idExtThread[pin];
    struct ThreadExtArg *threadArgs = (ThreadExtArg *)malloc(sizeof(ThreadExtArg));
    threadArgs->func = f;
    threadArgs->pin = pin;

    // Return if the interrupt pin number is out of range
    // NOT_AN_INTERRUPT is set when digitalPinToInterrupt(p) is used for an invalid pin
    if (pin == (uint8_t) NOT_AN_INTERRUPT) {
        fprintf(stderr, "%s(): interrupt pin number out of range\n",__func__);
        return;
    }

    // Check if the pin number is valid
    if (pin >= SOC_GPIO_PINS) {
        fprintf(stderr, "%s(): pin number should be less than "
            "%d, yours is %d \n", __func__, SOC_GPIO_PINS, pin);
        exit(1);
    }
    
    // Export pin for interrupt
    FILE *fp = fopen("/sys/class/gpio/export","w");
    if (fp == NULL) {
        fprintf(stderr, "%s(): export gpio error: %s\n",__func__, strerror (errno));
        exit(1);
    } else {
        fprintf(fp,"%d",pin); 
    }
    fclose(fp);
    
    // Tell the system to create the file /sys/class/gpio/gpio<GPIO number>
    char * interruptFile = NULL;
    asprintf(&interruptFile, "/sys/class/gpio/gpio%d/edge",pin);
    
    //Set detection edge condition
    fp = fopen(interruptFile,"w");
    if (fp == NULL) {
        // First time may fail because the file may not be ready.
        // if that the case then we wait two seconds and try again.
        sleep(2);
        fp = fopen(interruptFile,"w");
        if (fp == NULL) {
            fprintf(stderr, "%s(): set gpio edge interrupt of (%s) error: %s\n",
                __func__, interruptFile, strerror (errno));
            exit(1);
        }
    }
    switch(mode) {
        case RISING: fprintf(fp,"rising");break;
        case FALLING: fprintf(fp,"falling");break;
        default: fprintf(fp,"both");break;  // Change
    }
    fclose(fp);
    
    // Cancel any existent threads for the interrupt pin
    if (*threadId != 0) {
        pthread_cancel(*threadId);
    }

    // Create a thread passing the pin, function and mode
    pthread_create (threadId, NULL, threadFunction, (void *)threadArgs);
    
}

void detachInterrupt(uint8_t pin)
{
    pthread_t *threadId = &idExtThread[pin];

    // Return if the interrupt pin number is out of range
    // NOT_AN_INTERRUPT is set when digitalPinToInterrupt(p) is used for an invalid pin
    if (pin == (uint8_t) NOT_AN_INTERRUPT) {
        fprintf(stderr, "%s(): interrupt pin number out of range\n",__func__);
        return;
    }

    // Check if the pin number is valid
    if (pin >= SOC_GPIO_PINS) {
        fprintf(stderr, "%s(): pin number should be less than "
            "%d, yours is %d \n", __func__, SOC_GPIO_PINS, pin);
        exit(1);
    }

    // Cancel Thread
    pthread_cancel(*threadId);

    // Unexport gpio pin
    FILE *fp = fopen("/sys/class/gpio/unexport","w");
    if (fp == NULL) {
        fprintf(stderr, "%s(): unexport gpio error: %s\n",__func__, strerror (errno));
        exit(1);
    } else {
        fprintf(fp,"%d",pin); 
    }
    fclose(fp);
    
}


/////////////////////////////////////////////
//    Extra Arduino Functions for Linux   //
////////////////////////////////////////////
void (*ARDUINO_EXIT_FUNC)(void) = NULL;

// Every time an arduino program is ran it executes the following functions.
ArduinoLinux::ArduinoLinux()
{
    // Gets a timestamp when the program starts
    clock_gettime(CLOCK_REALTIME, &timestamp);

    // Set a callback function to detect when program is closed.
    // This is important so later we can turn off any gpio and pwm running. 
    if (signal(SIGINT, ArduinoLinux::onArduinoExit) == SIG_ERR)  // Ctrl^C
        fprintf(stderr, "%s(): can't catch signal SIGINT: %s\n",__func__, strerror (errno));
    if (signal(SIGTERM, ArduinoLinux::onArduinoExit) == SIG_ERR) // Kill command
        fprintf(stderr, "%s(): can't catch signal SIGKILL: %s\n",__func__, strerror (errno));
    if (signal(SIGHUP, ArduinoLinux::onArduinoExit) == SIG_ERR)  // Terminal closes
        fprintf(stderr, "%s(): can't catch signal SIGHUP: %s\n",__func__, strerror (errno));

}


// Catch Ctrl^C (SIGINT) and kill (SIGKILL) signals to set gpio and pwm to default state
void ArduinoLinux::onArduinoExit(int signumber)
{
    int i;

    // Shut down 
    if (signumber == SIGINT || signumber == SIGTERM ||  signumber == SIGHUP) {

        // If user wants to call a function at the end, here he can call it. 
        // He can exit so the rest of the code don't take place.
        if (ARDUINO_EXIT_FUNC != NULL) {
            // Call User exit func
            (*ARDUINO_EXIT_FUNC)();
        } else {

            // Set PWM and GPIO used pins to default state = input with no pull-up resistor
            for (i=0; i<SOC_GPIO_PINS; i++) {
                if (g_gpio_pin_set[i] == HIGH || g_pwm_pin_set[i] == HIGH) {
                    pinMode(i, INPUT);
                }
            }

            exit(0);

        }

    }
}


ArduinoLinux Arduino = ArduinoLinux();
