/*
 * variant.h
 * Nordic nRF52840 Dongle (PCA10059)
 * MIT License
 */

#pragma once

#include "WVariant.h"

////////////////////////////////////////////////////////////////////////////////
// Low frequency clock source

#define VARIANT_MCK       (64000000ul)

//#define USE_LFXO      // 32.768 kHz crystal oscillator
#define USE_LFRC    // 32.768 kHz RC oscillator

////////////////////////////////////////////////////////////////////////////////
// Number of pins (Arduino pin number = port pin, P1.xx = 32 + xx)

#define PINS_COUNT           (48)
#define NUM_DIGITAL_PINS     (48)
#define NUM_ANALOG_INPUTS    (4)
#define NUM_ANALOG_OUTPUTS   (0)

////////////////////////////////////////////////////////////////////////////////
// UART pin definition

#define PIN_SERIAL1_TX       (20)  // P0.20
#define PIN_SERIAL1_RX       (24)  // P0.24

////////////////////////////////////////////////////////////////////////////////
// I2C pin definition

#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (29)  // P0.29
#define PIN_WIRE_SCL         (31)  // P0.31

////////////////////////////////////////////////////////////////////////////////
// SPI pin definition

#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_SCK          (45)  // P1.13
#define PIN_SPI_MISO         (47)  // P1.15
#define PIN_SPI_MOSI         (42)  // P1.10

#define PIN_SPI_NSS          (22)  // P0.22

////////////////////////////////////////////////////////////////////////////////
// Builtin LEDs (active low)

#define PIN_LED1             (6)   // P0.06 LD1 green
#define PIN_LED2             (8)   // P0.08 LD2 red
#define LED_RED              (8)   // P0.08 LD2 red
#define LED_GREEN            (41)  // P1.09 LD2 green
#define LED_BLUE             (12)  // P0.12 LD2 blue
#define PIN_LED              PIN_LED1
#define LED_PIN              PIN_LED
#define LED_BUILTIN          PIN_LED
#define LED_STATE_ON         0

////////////////////////////////////////////////////////////////////////////////
// Builtin buttons

#define PIN_BUTTON1          (38)  // P1.06 SW1
#define BUTTON_PIN           PIN_BUTTON1
