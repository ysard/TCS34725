 
An improved version of [Adafruit TCS34725 (tested on v1.4.1)](https://github.com/adafruit/Adafruit_TCS34725)
wrapper given in their examples.

The wrapper provides improved lux and color temperature calculations as well as
a basic autorange mechanism.

The current improvement provides better POO integration: The code is split into
several functions avoiding redoing unnecessary calculations for better efficiency.
Moreover, a refactoring has been done to support interrupts from the TCS34725 chip.

So this version fully supports asynchronous as shown in the example below.


The attributes are mainly public:

```c++
uint16_t r_raw, g_raw, b_raw, c_raw;
uint16_t ir;
uint16_t r_comp, g_comp, b_comp, c_comp;
uint16_t saturation, saturation75;
uint16_t maxlux;
float cratio, color_temp, lux;
```

RGB channels are updated via `updateData()` and lux calculation based on them is
updated via `updateLux()`. Light source identification is available on the attribute
`cratio`, updated by `updateClearChannelRatio()`.

Note that by default `updateData()` is blocking, by polling raw data until
autorange is satisfying (i.e. no saturation occurs on RGB channels) and integration
time is out (i.e. data is ready on the sensor side) via `getRawData()`.

By calling `updateData(true)` the function is no longer blocking. We assume that
the data is now ready to be read. To make such an assertion, the use of an interrupt
is mandatory. The reading of the channels is done this time with `getRawData_noDelay()`.

Example of interrupt setup:

```c++
attachInterrupt(digitalPinToInterrupt(SENSOR_INTERRUPT_PIN), ISR_sensor, FALLING);

// Set persistence filter to generate an interrupt for every RGB Cycle,
// regardless of the integration limits
rgb_sensor.tcs.write8(TCS34725_PERS, TCS34725_PERS_NONE);
// RGBC interrupt enable. When asserted, permits RGBC interrupts to be generated.
rgb_sensor.tcs.setInterrupt(true);
```
