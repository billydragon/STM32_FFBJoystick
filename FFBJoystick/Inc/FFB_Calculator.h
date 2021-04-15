/*
 * FFB_Calculator.h
 *
 *  Created on: Feb 10, 2021
 *      Author: billy
 */

#ifndef INC_FFB_CALCULATOR_H_
#define INC_FFB_CALCULATOR_H_
#include "FFB_Calculator.h"
#include "main.h"
#include "FFBMain.h"
#include "Joystick.h"
#include "PIDReportHandler.h"
#include "PIDReportType.h"

struct __attribute__((packed)) Gains
{
  uint8_t totalGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t constantGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t rampGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t squareGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t sineGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t triangleGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t sawtoothdownGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t sawtoothupGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t springGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t damperGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t inertiaGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t frictionGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t customGain = FORCE_FEEDBACK_MAXGAIN;
};

struct __attribute__((packed)) EffectParams
{
		float springMaxPosition = 0;
		float springPosition = 0;

		float damperMaxVelocity = 0;
		float damperVelocity = 0;

		float inertiaMaxAcceleration = 0;
		float inertiaAcceleration = 0;

		float frictionMaxPositionChange = 0;
		float frictionPositionChange = 0;
};


extern bool FFB_effect_activated;

//force feedback gain
extern Gains *m_gains;

//force feedback effect params
extern EffectParams *m_effect_params;

///force calculate funtion

int32_t ApplyEnvelope (volatile TEffectState &effect, int32_t value);
int32_t ApplyGain (uint8_t value, uint8_t gain);
int32_t ConstantForceCalculator (volatile TEffectState &effect);
int32_t RampForceCalculator (volatile TEffectState &effect);
int32_t SquareForceCalculator (volatile TEffectState &effect);
int32_t SinForceCalculator (volatile TEffectState &effect);
int32_t TriangleForceCalculator (volatile TEffectState &effect);
int32_t SawtoothDownForceCalculator (volatile TEffectState &effect);
int32_t SawtoothUpForceCalculator (volatile TEffectState &effect);
int32_t ConditionForceCalculator (volatile TEffectState &effect, float metric, float scale, uint8_t axis);
void forceCalculator (int32_t *forces);
int32_t getEffectForce (volatile TEffectState &effect, Gains _gains, EffectParams _effect_params, uint8_t axis);
//force feedback Interfaces
void getForce (int32_t *forces);
//set gain functions
int8_t setGains (Gains *_gains);
//set effect params funtions
int8_t setEffectParams (EffectParams *_effect_params);
float NormalizeRange (int32_t x, int32_t maxValue);
void setFilterParameter();
#endif /* INC_FFB_CALCULATOR_H_ */
