/*
 * Filters.h
 *
 *  Created on: Feb 13, 2020
 *      Author: Yannick
 */

#ifndef BIQUAD_H_
#define BIQUAD_H_
#include "cppmain.h"

#ifdef __cplusplus

enum class BiquadType : uint8_t {
    lowpass = 0,
    highpass,
    bandpass,
    notch,
    peak,
    lowshelf,
    highshelf
};

class Biquad{
public:
	Biquad();
    Biquad(BiquadType type, float Fc, float Q, float peakGainDB);
    ~Biquad();
    float process(float in);
    void setBiquad(BiquadType type, float Fc, float Q, float peakGain);
    void setFc(float Fc); //frequency
    void setQ(float Q);
    void calcBiquad(void);

protected:

    BiquadType type;
    float a0, a1, a2, b1, b2;
    float Fc, Q, peakGain;
    float z1, z2;
};


#endif

#endif /* FILTERS_H_ */
