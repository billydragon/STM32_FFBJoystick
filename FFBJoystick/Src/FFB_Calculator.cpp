/*
 * FFB_Calc.c
 *
 *  Created on: Feb 10, 2021
 *      Author: billy
 */
#include "FFB_Calculator.h"
#include "FFBMain.h"
#include "PIDReportHandler.h"
#include "PIDReportType.h"
//#include "filters.h"
#include "FFBConfig.h"
#include "QEncoder.h"
#include "BiQuad.h"

#define FRICTION_SATURATION 10000
#define INERTIA_SATURATION 10000

float damper_f = 50 , damper_q = 0.2;
float friction_f = 50 , friction_q = 0.2;
float inertia_f = 20 , inertia_q = 0.2;
const uint32_t calcfrequency = 1000; // HID frequency 1khz
uint32_t cfFilter_f = calcfrequency/2; // 500 = off
const float cfFilter_q = 0.8;


//Biquad DamperFilterLp(BiquadType::lowpass, (float)damper_f / (float)calcfrequency, damper_q, (float)0.0);
//Biquad InertiaFilterLp(BiquadType::lowpass, (float)inertia_f / (float)calcfrequency, inertia_q, (float)0.0);
//Biquad FrictionFilterLp(BiquadType::lowpass, (float)friction_f / (float)calcfrequency, friction_q, (float)0.0);
//Biquad ConstantFilterLp(BiquadType::lowpass, (float)cfFilter_f / (float)calcfrequency, cfFilter_q, (float)0.0);

Biquad * DamperFilterLp[FFB_AXIS_COUNT];
Biquad * InertiaFilterLp[FFB_AXIS_COUNT];
Biquad * FrictionFilterLp[FFB_AXIS_COUNT];
Biquad * ConstantFilterLp[FFB_AXIS_COUNT];

extern USBD_HandleTypeDef hUsbDeviceFS;
extern FFBConfig config;
extern QEncoder encoder;


PIDReportHandler pidReportHandler;

Gains *m_gains;

//force feedback effect params
EffectParams *m_effect_params;

bool FFB_effect_activated = false;

void setConstantFilter(int ax);
void setDamperFilter(int ax);
void setInertiaFilter(int ax);
void setFrictionFilter(int ax);

/*
void setFilterParameter()
{

	damperFilter.setCutoffFreqHZ(config.SysConfig.Filter_Parameter.Damper_CF_Freq, true);
	damperFilter.setSamplingTime(config.SysConfig.Filter_Parameter.Damper_SamplingTime, true);

	inertiaFilter.setCutoffFreqHZ(config.SysConfig.Filter_Parameter.Inertia_CF_Freq, true);
	inertiaFilter.setSamplingTime(config.SysConfig.Filter_Parameter.Inertia_SamplingTime, true);

	frictionFilter.setCutoffFreqHZ(config.SysConfig.Filter_Parameter.Friction_CF_Freq, true);
	frictionFilter.setSamplingTime(config.SysConfig.Filter_Parameter.Friction_SamplingTime, true);
	constantFilter.setCutoffFreqHZ(config.SysConfig.Filter_Parameter.Constant_CF_Freq, true);
	constantFilter.setSamplingTime(config.SysConfig.Filter_Parameter.Constant_SamplingTime,true);

}
*/

void InitBiquadLp()
{

	for(int i = 0; i < FFB_AXIS_COUNT; i++)
	{
		if(ConstantFilterLp[i] != nullptr )
		{
		ConstantFilterLp[i]->setBiquad(BiquadType::lowpass, config.SysConfig.Filter_Parameter.Constant_CF_Freq / (float)calcfrequency,
											config.SysConfig.Filter_Parameter.Constant_SamplingTime, (float)0.0);
		}
		else
		{
			ConstantFilterLp[i] = new Biquad(BiquadType::lowpass, config.SysConfig.Filter_Parameter.Constant_CF_Freq / (float)calcfrequency,
														config.SysConfig.Filter_Parameter.Constant_SamplingTime, (float)0.0);
		}

		if(DamperFilterLp[i] != nullptr)
		{
			DamperFilterLp[i]->setBiquad(BiquadType::lowpass, config.SysConfig.Filter_Parameter.Damper_CF_Freq / (float)calcfrequency,
														 config.SysConfig.Filter_Parameter.Damper_SamplingTime, (float)0.0);
		}
		else
		{
			DamperFilterLp[i] = new Biquad(BiquadType::lowpass, config.SysConfig.Filter_Parameter.Damper_CF_Freq / (float)calcfrequency,
													 config.SysConfig.Filter_Parameter.Damper_SamplingTime, (float)0.0);
		}

		if(FrictionFilterLp[i] != nullptr)
		{
			FrictionFilterLp[i]->setBiquad(BiquadType::lowpass, config.SysConfig.Filter_Parameter.Friction_CF_Freq / (float)calcfrequency,
													config.SysConfig.Filter_Parameter.Friction_SamplingTime, (float)0.0);
		}
		else
		{
			FrictionFilterLp[i] = new Biquad(BiquadType::lowpass, config.SysConfig.Filter_Parameter.Friction_CF_Freq / (float)calcfrequency,
														config.SysConfig.Filter_Parameter.Friction_SamplingTime, (float)0.0);
		}

		if(InertiaFilterLp[i] != nullptr)
		{
			InertiaFilterLp[i]->setBiquad(BiquadType::lowpass, config.SysConfig.Filter_Parameter.Inertia_CF_Freq / (float)calcfrequency,
												  config.SysConfig.Filter_Parameter.Inertia_SamplingTime, (float)0.0);
		}
		else
		{
			InertiaFilterLp[i] = new Biquad(BiquadType::lowpass, config.SysConfig.Filter_Parameter.Inertia_CF_Freq / (float)calcfrequency,
													  config.SysConfig.Filter_Parameter.Inertia_SamplingTime, (float)0.0);
		}

	}
}

void setConstantFilter(int ax)
{
	float freq = config.SysConfig.Filter_Parameter.Constant_CF_Freq;
	float q = config.SysConfig.Filter_Parameter.Constant_SamplingTime;
	if(freq == 0){
		freq = calcfrequency / 2;
	}
	cfFilter_f = constrain(freq, 1, calcfrequency / 2);
	float f = (float)cfFilter_f / (float)calcfrequency;
	ConstantFilterLp[ax]->setFc(f);
	ConstantFilterLp[ax]->setQ(q);

}

void setDamperFilter(int ax)
{
	float freq = config.SysConfig.Filter_Parameter.Damper_CF_Freq;
	float q = config.SysConfig.Filter_Parameter.Damper_SamplingTime;
	if(freq == 0){
		freq = calcfrequency / 2;
	}
	damper_f = constrain(freq, 1, calcfrequency / 2);
	float f = (float)damper_f / (float)calcfrequency;
	DamperFilterLp[ax]->setFc(f);
	DamperFilterLp[ax]->setQ(q);

}

void setInertiaFilter(int ax)
{
	float freq = config.SysConfig.Filter_Parameter.Inertia_CF_Freq;
	float q = config.SysConfig.Filter_Parameter.Inertia_SamplingTime;
	if(freq == 0){
		freq = calcfrequency / 2;
	}
	inertia_f = constrain(freq, 1, calcfrequency / 2);
	float f = (float)inertia_f / (float)calcfrequency;
	InertiaFilterLp[ax]->setFc(f);
	InertiaFilterLp[ax]->setQ(q);

}

void setFrictionFilter(int ax)
{
	float freq = config.SysConfig.Filter_Parameter.Friction_CF_Freq;
	float q = config.SysConfig.Filter_Parameter.Friction_SamplingTime;
	if(freq == 0){
		freq = calcfrequency / 2;
	}
	friction_f = constrain(freq, 1, calcfrequency / 2);
	float f = (float)friction_f / (float)calcfrequency;
	FrictionFilterLp[ax]->setFc(f);
	FrictionFilterLp[ax]->setQ(q);
}


void getForce (int32_t *forces)
{
  //RecvfromUsb();
  forceCalculator (forces);
}

void forceCalculator (int32_t *forces)
{
  forces[0] = 0;
  forces[1] = 0;

  for (int id = 0; id < MAX_EFFECTS; id++)
    {
      volatile TEffectState &effect = pidReportHandler.g_EffectStates[id];

      		if (effect.state != EFFECT_STATE_INACTIVE && effect.duration != USB_DURATION_INFINITE)
      		{
      			if(HAL_GetTick() < effect.startTime)
      			{
					continue;
				}
      			if(HAL_GetTick() > (effect.startTime + effect.duration))
				{
					effect.state = EFFECT_STATE_INACTIVE;
				}
      		}

      		// Filter out inactive effects
      		if (effect.state == EFFECT_STATE_INACTIVE)
      		{
      			continue;
      		}

      if ((effect.state == MEFFECTSTATE_PLAYING) && ((effect.elapsedTime <= effect.duration)
	      || (effect.duration == USB_DURATION_INFINITE))  && !pidReportHandler.devicePaused)
    	  {
			  if (effect.enableAxis == DIRECTION_ENABLE || effect.enableAxis & X_AXIS_ENABLE)
				{
				  forces[X_AXIS] += (int32_t) (getEffectForce (effect, m_gains[X_AXIS], m_effect_params[X_AXIS], X_AXIS));

				}

			  if (effect.enableAxis == DIRECTION_ENABLE || effect.enableAxis & Y_AXIS_ENABLE)
				{
				  forces[Y_AXIS] += (int32_t) (getEffectForce (effect, m_gains[Y_AXIS], m_effect_params[Y_AXIS], Y_AXIS));

				}

    	  }
			//printf("Force_X: %ld, Force_Y: %ld \n", forces[0], forces[1]);

    }

  forces[0] = (int32_t) ((float) 1.00 * forces[0] * m_gains[0].totalGain / 255);
  forces[1] = (int32_t) ((float) 1.00 * forces[1] * m_gains[1].totalGain / 255);

  forces[0] = constrain(forces[0], -32767, 32767);		//16 bits
  forces[1] = constrain(forces[1], -32767, 32767);


}

int32_t getEffectForce (volatile TEffectState &effect, Gains _gains, EffectParams _effect_params, uint8_t axis)
{

  uint16_t direction;
  uint8_t condition = 0;
  //bool useForceDirectionForConditionEffect = (effect.enableAxis == DIRECTION_ENABLE && effect.conditionBlocksCount == 1);

  if (effect.enableAxis == DIRECTION_ENABLE)
    {
		  direction = effect.directionX;
		  // If the Direction Enable flag is set, only one Condition Parameter Block is defined
		  if (effect.conditionBlocksCount > 1) {
				  condition = axis;
			 }
    }
  else
    {
      direction = axis == 0 ? effect.directionX : effect.directionY;
      condition = axis;
    }

  bool rotateConditionForce = (FFB_AXIS_COUNT > 1 && effect.conditionBlocksCount == 1);
  //bool rotateConditionForce = (FFB_AXIS_COUNT > 1 && effect.conditionBlocksCount < FFB_AXIS_COUNT);
  //float angle = ((float)direction * 360.0 / 35999.0) * DEG_TO_RAD;
  float angle = ((float)direction * (2*PI) / 36000.0f) * DEG_TO_RAD;
  float angle_ratio = axis == 0 ? sin (angle) : -1 * cos (angle);
  angle_ratio = rotateConditionForce ? angle_ratio : 1.0;

  int32_t force = 0;
  float metric =0;
  //setFilterParameter();
  switch (effect.effectType)
    {
    case USB_EFFECT_CONSTANT: //1
    		force = -1.5 * ConstantForceCalculator (effect) * angle_ratio * _gains.constantGain;

    		setConstantFilter(axis);
    		if (cfFilter_f < calcfrequency / 2)
			{
    			force = ConstantFilterLp[axis]->process(force);
			}
    		//printf("Constatnt: metric: %f, force: %ld", metric, force);
      break;
    case USB_EFFECT_RAMP: //2
    		force =  -1.0 * RampForceCalculator (effect) * angle_ratio * _gains.rampGain;
      break;
    case USB_EFFECT_SQUARE: //3
    		force = -1.0 * SquareForceCalculator (effect) * angle_ratio * _gains.squareGain;
      break;
    case USB_EFFECT_SINE: //4
    		force =  -1.0 * SinForceCalculator (effect) * angle_ratio * _gains.sineGain;
      break;
    case USB_EFFECT_TRIANGLE: //5
    		force = -1.0 * TriangleForceCalculator (effect) * angle_ratio * _gains.triangleGain;
      break;
    case USB_EFFECT_SAWTOOTHDOWN: //6
    		force = -1.0 * SawtoothDownForceCalculator (effect) * angle_ratio * _gains.sawtoothdownGain;
      break;
    case USB_EFFECT_SAWTOOTHUP: //7
    		force = -1.0 * SawtoothUpForceCalculator (effect) * angle_ratio * _gains.sawtoothupGain;
      break;

    case USB_EFFECT_SPRING://8
   			metric = _effect_params.springPosition;
   			force = ConditionForceCalculator(effect, metric, 0.0006f, condition) * angle_ratio * _gains.springGain;
   			//printf("Spring: metric: %f, force: %ld \n", metric, force);
   		break;
   	case USB_EFFECT_DAMPER://9

   			setDamperFilter(axis);
   			metric = DamperFilterLp[axis]->process(_effect_params.damperVelocity) * 0.25;	//.0625f;
   		   force = ConditionForceCalculator(effect, metric, 0.60f , condition) * angle_ratio * _gains.damperGain;
   		   //printf("Damper: metric: %f, force: %ld\n", metric, force);
   		break;
   	case USB_EFFECT_INERTIA://10

   			setInertiaFilter(axis);
   			metric = InertiaFilterLp[axis]->process(_effect_params.inertiaAcceleration * 4);
   			force = ConditionForceCalculator(effect, metric,0.45f, condition) * angle_ratio * _gains.inertiaGain;
   			//printf("Inertia: metric: %f, force: %ld\n", metric, force);
   		break;
   	case USB_EFFECT_FRICTION://11

   			setFrictionFilter(axis);
   			metric = FrictionFilterLp[axis]->process(_effect_params.frictionPositionChange) * 0.25;	//.25;
   			force = ConditionForceCalculator(effect, metric , 0.60f, condition) * angle_ratio * _gains.frictionGain;
   			//printf("Friction: metric: %f, force: %ld\n", metric, force);
   			break;

    case USB_EFFECT_CUSTOM: //12
      break;
    }
  effect.elapsedTime = (uint64_t) HAL_GetTick () - effect.startTime;
  return (force / 255); //Apply Users Gain
}


int32_t ConstantForceCalculator (volatile TEffectState &effect)
{

  float tempforce = ((float) effect.magnitude * 1.0f * (int32_t)(1 + effect.gain))/10000;

  if(effect.useEnvelope)
     {
   	  tempforce = ApplyEnvelope (effect, tempforce);
     }


  return tempforce;
}

int32_t RampForceCalculator (volatile TEffectState &effect)
{
	uint32_t duration = effect.duration;
  int32_t tempforce = effect.startMagnitude + effect.elapsedTime * (effect.endMagnitude - effect.startMagnitude)/ duration;
  tempforce *= (int32_t)(1 + effect.gain) /10000;

  if(effect.useEnvelope)
    {
  	  tempforce = ApplyEnvelope (effect, tempforce);
    }
    return tempforce;
}


#if(1)

int32_t SquareForceCalculator (volatile TEffectState &effect)
{
  int32_t offset = effect.offset;
  uint32_t magnitude = effect.magnitude;
  uint32_t elapsedTime = effect.elapsedTime;
  uint32_t phase = effect.phase;
  uint32_t period = effect.period;

  int32_t maxMagnitude = offset + magnitude;
  int32_t minMagnitude = offset - magnitude;
  uint32_t phasetime = (phase * period) / 35999;
  uint32_t timeTemp = elapsedTime + phasetime;
  uint32_t reminder = timeTemp % period;
  int32_t tempforce;
  if (reminder > (period / 2))
    tempforce = minMagnitude;
  else
    tempforce = maxMagnitude;
  if(effect.useEnvelope)
    {
  	  tempforce = ApplyEnvelope (effect, tempforce);
    }
    return tempforce;
}


#else
int32_t SquareForceCalculator (volatile TEffectState &effect)
{
	uint32_t elapsedTime = effect.elapsedTime;
	int32_t tempforce = ((elapsedTime + effect.phase) % ((uint32_t)effect.period + 2)) < (uint32_t)(effect.period + 2) / 2 ? -effect.magnitude : effect.magnitude;
	tempforce = tempforce + effect.offset;
	 if(effect.useEnvelope)
	    {
	  	  tempforce = ApplyEnvelope (effect, tempforce);
	    }
	    return tempforce;
}


#endif

int32_t SinForceCalculator (volatile TEffectState &effect)
{
  float offset = effect.offset;
  float magnitude = effect.magnitude;
  float phase = effect.phase;
  float timeTemp = effect.elapsedTime;
  float period = effect.period;
  float angle = ((timeTemp / period) + (phase / 35999) * period) * 2 * PI;
  float sine = sin (angle);
  float tempforce = sine * magnitude;
  tempforce += offset;
  if(effect.useEnvelope)
    {
  	  tempforce = ApplyEnvelope (effect, tempforce);
    }
    return tempforce;
}

int32_t TriangleForceCalculator (volatile TEffectState &effect)
{
  float offset = effect.offset;
  float magnitude = effect.magnitude;
  float elapsedTime = effect.elapsedTime;
  uint32_t phase = effect.phase;
  uint32_t period = effect.period;
  float periodF = effect.period;

  float maxMagnitude = offset + magnitude;
  float minMagnitude = offset - magnitude;
  //uint32_t phasetime = (phase * period) / 255;
  uint32_t phasetime = (phase * period) / 35999;
  uint32_t timeTemp = elapsedTime + phasetime;
  float reminder = timeTemp % period;
  float slope = ((maxMagnitude - minMagnitude) * 2) / periodF;
  float tempforce = 0;
  if (reminder > (periodF / 2))
	  tempforce = slope * (periodF - reminder);
  else
	  tempforce = slope * reminder;
  tempforce += minMagnitude;
  if(effect.useEnvelope)
    {
  	  tempforce = ApplyEnvelope (effect, tempforce);
    }
    return tempforce;
}

int32_t SawtoothDownForceCalculator (volatile TEffectState &effect)
{
  float offset = effect.offset;
  float magnitude = effect.magnitude;
  float elapsedTime = effect.elapsedTime;
  float phase = effect.phase;
  uint32_t period = effect.period;
  float periodF = effect.period;

  float maxMagnitude = offset + magnitude;
  float minMagnitude = offset - magnitude;
  //int32_t phasetime = (phase * period) / 255;
  int32_t phasetime = (phase * period) / 35999;
  uint32_t timeTemp = elapsedTime + phasetime;
  float reminder = timeTemp % period;
  float slope = (maxMagnitude - minMagnitude) / periodF;
  float tempforce = 0;
  tempforce = slope * (period - reminder);
  tempforce += minMagnitude;
  if(effect.useEnvelope)
    {
  	  tempforce = ApplyEnvelope (effect, tempforce);
    }
    return tempforce;
}

int32_t SawtoothUpForceCalculator (volatile TEffectState &effect)
{
  float offset = effect.offset;
  float magnitude = effect.magnitude;
  float elapsedTime = effect.elapsedTime;
  uint32_t phase = effect.phase;
  uint32_t period = effect.period;
  float periodF = effect.period;

  float maxMagnitude = offset + magnitude;
  float minMagnitude = offset - magnitude;
  //int32_t phasetime = (phase * period) / 255;
  int32_t phasetime = (phase * period) / 35999;
  uint32_t timeTemp = elapsedTime + phasetime;
  float reminder = timeTemp % period;
  float slope = (maxMagnitude - minMagnitude) / periodF;
  float tempforce = 0;
  tempforce = slope * reminder;
  tempforce += minMagnitude;
  if(effect.useEnvelope)
  {
	  tempforce = ApplyEnvelope (effect, tempforce);
  }
  return tempforce;
}




int32_t ConditionForceCalculator (volatile TEffectState &effect, float metric, float scale, uint8_t axis)
{
  float deadBand;
  float cpOffset;
  float negativeCoefficient;
  float negativeSaturation;
  float positiveSaturation;
  float positiveCoefficient;

  deadBand = effect.conditions[axis].deadBand;
  cpOffset = effect.conditions[axis].cpOffset;
  negativeCoefficient = effect.conditions[axis].negativeCoefficient;
  negativeSaturation = effect.conditions[axis].negativeSaturation;
  positiveSaturation = effect.conditions[axis].positiveSaturation;
  positiveCoefficient = effect.conditions[axis].positiveCoefficient;

  float tempForce = 0;

  if (metric < (cpOffset - deadBand))
    {

# if(0)
	  tempForce = (metric -  (float)1.0f*(cpOffset - deadBand)/10000) * negativeCoefficient * scale ;
	  tempForce = (tempForce < -negativeSaturation ? -negativeSaturation : tempForce);

#else
	  tempForce = (metric -  (cpOffset - deadBand)) * negativeCoefficient * scale;
	  tempForce = constrain(tempForce,-negativeSaturation,positiveSaturation);
#endif

    }
  else if (metric > (cpOffset + deadBand))
    {
#if(0)
		  tempForce = (metric - (float)1.0f*(cpOffset + deadBand)/10000) * positiveCoefficient * scale;
		  tempForce = (tempForce > positiveSaturation ? positiveSaturation : tempForce);

#else
		  tempForce = (metric -  (cpOffset + deadBand)) * positiveCoefficient * scale;
		  tempForce = constrain(tempForce,-negativeSaturation,positiveSaturation);
#endif

    }
	  else return 0;

	  tempForce = -tempForce * (1 + effect.gain) / 10000;

	  encoder.Update_Metric(axis);

  return (int32_t) tempForce;
}



int32_t ApplyGain (uint16_t value, uint16_t gain)
{
  int32_t value_32 = (int32_t) value;
  return ((value_32 * (1+ gain)) / 10000);

}

int32_t ApplyEnvelope (volatile TEffectState &effect, int32_t value)
{
  //int32_t magnitude = ApplyGain (effect.magnitude, effect.gain);
  //int32_t attackLevel = ApplyGain (effect.attackLevel, effect.gain);
  //int32_t fadeLevel = ApplyGain (effect.fadeLevel, effect.gain);
  int32_t magnitude = effect.magnitude;
  int32_t attackLevel = effect.attackLevel;
  int32_t fadeLevel = effect.fadeLevel;
  int32_t newValue = magnitude;
  int32_t attackTime = effect.attackTime;
  int32_t fadeTime = effect.fadeTime;
  int32_t elapsedTime = effect.elapsedTime;
  int32_t duration = effect.duration;

  if (elapsedTime < attackTime)
    {
      newValue = (magnitude - attackLevel) * elapsedTime;
      newValue /= attackTime;
      newValue += attackLevel;
    }
  if (duration != USB_DURATION_INFINITE && elapsedTime > (duration - fadeTime))
    {
      newValue = (magnitude - fadeLevel) * (duration - elapsedTime);
      newValue /= fadeTime;
      newValue += fadeLevel;
    }

  newValue *= value;
  newValue /= 10000;
  //newValue /= 0xFFFF; //16 bits
  return newValue;
}


void end ()
{

}
//set gain functions
int8_t setGains (Gains *_gains)
{
  if (_gains != nullptr)
    {
      //it should be added some limition here,but im so tired,it's 2:24 A.M now!
      m_gains = _gains;
      return 0;
    }
  return -1;
}

//set effect params funtions
int8_t setEffectParams (EffectParams *_effect_params)
{
  if (_effect_params != nullptr)
    {
      m_effect_params = _effect_params;
      return 0;
    }
  return -1;
}

float NormalizeRange (int32_t x, int32_t maxValue)
{
  return (float) x * 1.0f / maxValue;
}
