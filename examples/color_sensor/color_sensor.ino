/* This file is taken from MyOwnBricks project.
 * MyOwnBricks is a library for the emulation of LEGO PoweredUp sensors on microcontrollers
 * Copyright (C) 2021-2022 Ysard - <ysard@users.noreply.github.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
 *   Sensor   Pro Micro
 *   SCL      SCL pin 3
 *   SDA      SDA pin 2
 *   INT      PCINT4, port PB4, pin 8
 *   VIN      VCC (3.3V)
 *   GND      GND
 *
 *   LED pin of the sensor can be connected to its INT pin.
 *   By doing this the leds will turn off when the measurements are done.
 *
 *   Pro Micro:
 *   Serial: UART via USB
 *   Serial1: pin 1 (TX), pin 0 (RX)
 */
#include <Wire.h>
#include "tcs34725.h"

#define MANHATTAN
#include "color_detection_methods.hpp"


#define RGB_SENSOR_INTERRUPT_PIN     8
#define RGB_SENSOR_INTERRUPT_PORT    PB4 // Port alias of pin 8 to be used in tstPin()
// Equivalent of digitalRead but for PORTB pins & much more quicker for a use in an ISR
// https://www.arduino.cc/en/Reference/PortManipulation
#define tstPin(b)                             ((PINB & (1 << (b))) != 0)
#define LUX_TO_PERCENTAGE(val)                (getPercentage(val, 0.0105, -0.0843))
#define REFLECTED_LIGHT_TO_PERCENTAGE(val)    (getPercentage(val, 0.0017, -8))

uint8_t       sensorColor;
uint8_t       reflectedLight;
uint8_t       ambientLight;
uint16_t      red, green, blue, clear, lux;
volatile bool sensorReady;

// Default settings: TCS34725_GAIN_4X,  TCS34725_INTEGRATIONTIME_154MS
TCS34725      rgb_sensor;

/**
 * @brief Callback for PCINT4 interrupt (PCINT0 - PCINT7)
 */
ISR(PCINT0_vect) {
    // If RGB_SENSOR_INTERRUPT_PORT is LOW, sensor is ready
    if (!tstPin(RGB_SENSOR_INTERRUPT_PORT))
        sensorReady = true;
}


/**
 * @brief Map lux/reflected light values to percentages
 *    Weights of the equation must be calculated empirically
 *    Map equation: y = ax + b
 *    System to solve:
 *      100% = MaxRawValue * a + b
 *      0% = MinRawValue * a + b
 *
 *    See macros LUX_TO_PERCENTAGE and REFLECTED_LIGHT_TO_PERCENTAGE.
 */
uint8_t getPercentage(const uint16_t rawValue, const float& a_coef, const float& b_coef) {
    int8_t percent = static_cast<int8_t>(rawValue * a_coef + b_coef);
    if (percent > 100)
        return 100;
    if (percent < 0)
        return 0;
    return static_cast<uint8_t>(percent);
}


void setup() {
    Serial.begin(115200);
    while (!Serial) {
        // Wait for serial port to connect.
    }

    // Device config
    sensorColor = COLOR_NONE;

    // Colour sensor config
    // Configure PinChange Interrupt
    // See https://github.com/NicoHood/PinChangeInterrupt
    // Note: INT-0,1,2,3 are occupied by UART and i2c transmissions on pro-micro
    // /!\ DO NOT activate pullup from the arduino, the INT pin is usually already
    // pulled up into the sensor board itself to 3.3V. These pins (SCL, SDA, INT)
    // ARE NOT tolerant to more than VDD + 0.5V. Note that I2C pins are connected
    // to level shifters, but not the others.
    pinMode(RGB_SENSOR_INTERRUPT_PIN, INPUT); // TCS interrupt output is Active-LOW and Open-Drain
    cli();                                    // Disable all interrupts: Avoid first and not wanted trigger of the interrupt
    PCICR  |= 0b00000001;                     // enable PORTB pin change interrupt
    PCMSK0 |= 0b00010000;                     // enable PB4, PCINT4, pin 8
    sei();                                    // Enable all interrupts

    while (!rgb_sensor.begin()) {
        Serial.println(F("TCS34725 NOT found"));
        delay(200);
    }
    Serial.println(F("Found sensor"));

    // Set persistence filter to generate an interrupt for every RGB Cycle,
    // regardless of the integration limits
    rgb_sensor.tcs.write8(TCS34725_PERS, TCS34725_PERS_NONE);
    // RGBC interrupt enable. When asserted, permits RGBC interrupts to be generated.
    rgb_sensor.tcs.setInterrupt(true);
}


void loop()
{
    if (sensorReady) {
        // Data measurement
        // noDelay param set to true: Asynchronous mode, must be used with interrupt configured.
        bool status = rgb_sensor.updateData(true);

        if (status) {
            // Ambient light (lux) computation
            rgb_sensor.updateLux();

            int16_t lux = lround(rgb_sensor.lux);

            // Sometimes lux values are below 0; this coincides with erroneous data
            // Moreover, we discard data taken below 40 lux.
            if ((lux >= 40) && (static_cast<uint16_t>(lux) <= rgb_sensor.maxlux)) {
                // Set ambient light (lux) - map 0-100
                //ambientLight = map(rgb_sensor.lux, 0, rgb_sensor.maxlux, 0, 100);
                ambientLight = LUX_TO_PERCENTAGE(lux); // cast ?

                // RGBC Channels are usable
                // Map values to max ~440;
                // Continuous values from 0-65535 (16bits) to 0-1023 (10bits)
                red   = rgb_sensor.r_comp >> 6,
                green = rgb_sensor.g_comp >> 6,
                blue  = rgb_sensor.b_comp >> 6,

                // Set clear channel as reflected light - map 0-100
                reflectedLight = REFLECTED_LIGHT_TO_PERCENTAGE(rgb_sensor.c_comp);

                // Set detected color
                sensorColor = detectColor(red, green, blue);
            } else {
                sensorColor = COLOR_NONE;
            }
            clear = rgb_sensor.c_comp >> 6;
            /*
            // Human readable debugging
            Serial.print("Lux: "); Serial.print(rgb_sensor.lux, DEC);
            Serial.print("; max: "); Serial.print(rgb_sensor.maxlux);
            Serial.print("; R: "); Serial.print(red, DEC);
            Serial.print("; G: "); Serial.print(green, DEC);
            Serial.print("; B: "); Serial.print(blue, DEC);
            Serial.print("; C: "); Serial.println(clear, DEC);
            */

            // Spreadsheet debugging
            Serial.print(rgb_sensor.lux, DEC); Serial.print(";");
            Serial.print(rgb_sensor.maxlux); Serial.print(";");
            Serial.print(red, DEC); Serial.print(";");
            Serial.print(green, DEC); Serial.print(";");
            Serial.print(blue, DEC); Serial.print(";");
            Serial.println(clear, DEC);
        } else {
            sensorColor = COLOR_NONE;
            Serial.println(F("not valid data! wait next measure"));
        }
        // Interrupt tear down
        rgb_sensor.tcs.clearInterrupt();
        sensorReady = false;
        PCIFR      &= ~(1 << PCIF0); // clear PC interrupt flag in case of bounce
    }
}
