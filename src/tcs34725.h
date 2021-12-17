#ifndef CUSTOM_TCS34725_H
#define CUSTOM_TCS34725_H

#include <Wire.h>
#include "Adafruit_TCS34725.h"

//
// An experimental wrapper class that implements the improved lux and color temperature from
// TAOS and a basic autorange mechanism.
//
// Written by ductsoup, public domain
// Updated by Ysard (2021) for better POO integration and efficiency
//

// RGB Color Sensor with IR filter and White LED - TCS34725
// I2C 7-bit address 0x29, 8-bit address 0x52
//
// http://www.adafruit.com/product/1334
// http://learn.adafruit.com/adafruit-color-sensors/overview
// http://www.adafruit.com/datasheets/TCS34725.pdf
// http://www.ams.com/eng/Products/Light-Sensors/Color-Sensor/TCS34725
// http://www.ams.com/eng/content/view/download/265215 <- DN40, calculations
// http://www.ams.com/eng/content/view/download/181895 <- DN39, some thoughts on autogain
// http://www.ams.com/eng/content/view/download/145158 <- DN25 (original Adafruit calculations)
//
// connect LED to digital 4 or GROUND for ambient light sensing
// connect SCL to analog 5
// connect SDA to analog 4
// connect Vin to 3.3-5V DC
// connect GROUND to common ground

// some magic numbers for this device from the DN40 application note
#define TCS34725_R_Coef       0.136
#define TCS34725_G_Coef       1.000
#define TCS34725_B_Coef       -0.444
// Device factor
#define TCS34725_DF           310.0
// Color temp coef for CT calculation
#define TCS34725_CT_Coef      3810.0
// Color temp offset for CT calculation
#define TCS34725_CT_Offset    1391.0
// Glass attenuation factor; put 1.08 if behind clear glass
#define TCS34725_GA           1.0

// Autorange class for TCS34725
class TCS34725 {
public:
    TCS34725();

    bool begin();
    void getRawData_noDelay(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
    bool updateData(bool noDelay);
    void updateClearChannelRatio();
    void updateLux();
    void updateColorTemperature();

    bool isAvailable, isSaturated;

    uint16_t r_raw, g_raw, b_raw, c_raw;
    uint16_t ir;
    uint16_t r_comp, g_comp, b_comp, c_comp;
    uint16_t saturation, saturation75;
    uint16_t maxlux;
    float cratio, color_temp, lux;

    Adafruit_TCS34725 tcs;

private:
    void setGainTime(void);
    bool autorange(bool noDelay);

    struct tcs_agc
    {
        tcs34725Gain_t again;
        uint8_t        atime;
        uint16_t       mincnt;
        uint16_t       maxcnt;
    };
    static const tcs_agc m_agc_lst[];
    uint16_t m_againx, m_atime, m_atime_ms;
    uint16_t m_agc_cur;
    float m_count_per_lux;
};

#endif
