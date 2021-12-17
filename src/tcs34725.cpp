#include "tcs34725.h"

// Gain/time combinations to use and the min/max limits for hysteresis
// that avoid saturation. They should be in order from dim to bright.
//
// Also set the first min count and the last max count to 0 to indicate
// the start and end of the list.
const TCS34725::tcs_agc TCS34725::m_agc_lst[] =
{
    { TCS34725_GAIN_60X, TCS34725_INTEGRATIONTIME_614MS,     0, 20000 },
    { TCS34725_GAIN_60X, TCS34725_INTEGRATIONTIME_154MS,  4990, 63000 },
    { TCS34725_GAIN_16X, TCS34725_INTEGRATIONTIME_154MS, 16790, 63000 },
    { TCS34725_GAIN_4X,  TCS34725_INTEGRATIONTIME_154MS, 15740, 63000 },
    { TCS34725_GAIN_1X,  TCS34725_INTEGRATIONTIME_154MS, 15740,     0 }
};


/**
 * @brief Constructor
 *      Settings by default: TCS34725_GAIN_4X,  TCS34725_INTEGRATIONTIME_154MS
 */
TCS34725::TCS34725() : isAvailable(0), isSaturated(0), m_agc_cur(3) {}


/**
 * @brief Initialize the sensor
 */
bool TCS34725::begin() {
    tcs = Adafruit_TCS34725(m_agc_lst[m_agc_cur].atime, m_agc_lst[m_agc_cur].again);
    if ((isAvailable = tcs.begin())) {
        setGainTime();
    }
    return(isAvailable);
}


/**
 * @brief Set the gain and the integration time
 */
void TCS34725::setGainTime() {
    tcs.setGain(m_agc_lst[m_agc_cur].again);
    tcs.setIntegrationTime(m_agc_lst[m_agc_cur].atime);
    m_atime    = uint16_t(m_agc_lst[m_agc_cur].atime);
    m_atime_ms = uint16_t((256 - m_atime) * 2.4);

    switch (m_agc_lst[m_agc_cur].again) {
        case TCS34725_GAIN_1X:
            m_againx = 1;
            break;

        case TCS34725_GAIN_4X:
            m_againx = 4;
            break;

        case TCS34725_GAIN_16X:
            m_againx = 16;
            break;

        case TCS34725_GAIN_60X:
            m_againx = 60;
            break;
    }
    m_count_per_lux = (m_atime_ms * m_againx) / (TCS34725_GA * TCS34725_DF);
/*
    Serial.print("gain;");
    Serial.print(m_againx);
    Serial.print("; atime;");
    Serial.println(m_atime);
    */
}


/**
 * @brief getRawData() does a delay(Integration_Time) after the sensor readout.
 *      We don't need to wait for the next integration cycle because we receive an
 *      interrupt when the integration cycle is complete
 */
void TCS34725::getRawData_noDelay(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) {
  *c = tcs.read16(TCS34725_CDATAL);
  *r = tcs.read16(TCS34725_RDATAL);
  *g = tcs.read16(TCS34725_GDATAL);
  *b = tcs.read16(TCS34725_BDATAL);
}


/**
 * @brief Read the sensor and autorange if necessary
 * @param noDelay If true, asynchronous mode, must be used with interrupt configured.
 *      If false, RGBC channels are polled.
 * @see getRawData_noDelay() and getRawData()
 * @return true if the RGBC measure is usable. false otherwise.
 *      In this case, at least the next interrupt/integration cycle must be waited.
 *      If noDelay is false, always return true.
 */
bool TCS34725::autorange(bool noDelay=false) {
    if (noDelay)
        this->getRawData_noDelay(&r_raw, &g_raw, &b_raw, &c_raw);
    else
        tcs.getRawData(&r_raw, &g_raw, &b_raw, &c_raw);

    if (m_agc_lst[m_agc_cur].maxcnt && (c_raw > m_agc_lst[m_agc_cur].maxcnt)) {
        m_agc_cur++;
    } else if (m_agc_lst[m_agc_cur].mincnt && (c_raw < m_agc_lst[m_agc_cur].mincnt)) {
        m_agc_cur--;
    } else {
        // Settings are ok, measure is acceptable
        return true;
    }
    // Integration time & gain need to be changed
    setGainTime();

    if (noDelay)
        // A new measure is needed: let's wait the next interrupt
        return false;

    // A new measure is needed: let's wait the next cycle
    delay(m_atime_ms * 2); // shock absorber
    tcs.getRawData(&r_raw, &g_raw, &b_raw, &c_raw);
    return true;
}


/**
 * @brief Retrieve data from the sensor and do the calculations
 * @param noDelay If true, asynchronous mode, must be used with interrupt configured.
 *      If false, RGBC channels are polled.
 * @return true if RGBC data is correct, false if a new measure is needed because
 *      of autorange adjustment or channel saturation.
 */
bool TCS34725::updateData(bool noDelay=false) {
    if (!this->autorange(noDelay))
        // A new measure is needed: let's wait the next interrupt
        return false;

    // DN40 calculations

    /* Analog/Digital saturation:
     *
     * (a) As light becomes brighter, the clear channel will tend to
     *     saturate first since R+G+B is approximately equal to C.
     *     When the clear channel saturates, the IR calculation algorithm
     *     breaks down and will no longer work.
     * (b) The TCS34725 accumulates 1024 counts per 2.4ms of integration
     *     time, up to a maximum values of 65535. This means analog
     *     saturation can occur up to an integration time of 153.6ms
     *     (64*2.4ms=153.6ms).
     * (c) If the integration time is > 153.6ms, digital saturation will
     *     occur before analog saturation. Digital saturation occurs when
     *     the count reaches 65535.
     */
    // ATIME_ms > 154ms (Digital Saturation), else Analog Saturation
    saturation = ((256 - m_atime) > 63) ? 65535 : 1024 * (256 - m_atime);

    /* Ripple saturation notes:
     *
     * (a) If there is ripple in the received signal, the value read from C
     *     will be less than the max, but still have some effects of being
     *     saturated. This means that you can be below the 'sat' value, but
     *     still be saturating. At integration times >150ms this can be
     *     ignored, but <= 150ms you should calculate the 75% saturation
     *     level to avoid this problem.
     */
    /* Adjust sat to 75% to avoid analog saturation if atime < 153.6ms */
    saturation75 = (m_atime_ms < 150) ? (saturation - saturation / 4) : saturation;
    isSaturated  = (m_atime_ms < 150 && c_raw > saturation75) ? 1 : 0;
    /* Check for saturation and mark the sample as invalid if true */
    if (isSaturated) {
        return false;
    }
    // IR calculations & channels correction
    // 0 if Low light with no IR, else (r + g + b - c) / 2
    ir     = (r_raw + g_raw + b_raw > c_raw) ? (r_raw + g_raw + b_raw + c_raw) >> 1 : 0;
    r_comp = r_raw - ir;
    g_comp = g_raw - ir;
    b_comp = b_raw - ir;
    c_comp = c_raw - ir;

    return true;
}


/**
 * @brief Update Clear Channel Ratio
 *
 * @note Clear Channel Ratio is used for light source identification.
 *      Clear Channel Ratio = IR / Clear channel; usually always < 1
 *
 *      - very high (~0.3): incandescent light
 *      - medium (1/2 max value): sunlight
 *      - low (<0.1): fluorescent/LED light
 */
void TCS34725::updateClearChannelRatio() {
    cratio = float(ir) / float(c_raw);
}


/**
 * @brief Update lux & maxlux values
 *      Lux value can be negative at low light; In this case
 *      all values (included RGBC) should be discarded.
 *      Idem if maxlux < lux.
 */
void TCS34725::updateLux() {
    // Note: CPL (Counts per Lux) is calculated only when the ATIME or AGAIN has changed.
    maxlux = static_cast<uint16_t>(65535 / (m_count_per_lux * 3));
    // TODO: can be negative at low light, should return 0
    // TODO: multiply coefs by 1000 and use atime in us (not ms) to avoid fractionnal manipulations
    lux = (TCS34725_R_Coef * r_comp + TCS34725_G_Coef * g_comp + TCS34725_B_Coef * b_comp) / m_count_per_lux;
}


/**
 * @brief Update Color Temperature value
 *
 * @note A simple method of measuring color temp is to use the ratio
 *      of blue to red light, taking IR cancellation into account.
 */
void TCS34725::updateColorTemperature() {
    color_temp = (TCS34725_CT_Coef * b_comp) / r_comp + TCS34725_CT_Offset;
}
