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
#include "filters.h"
#include "FFBConfig.h"


#define FRICTION_SATURATION 10000
#define INERTIA_SATURATION 10000

const float cutoff_freq_damper = 5.0;  //Cutoff frequency in Hz
const float sampling_time_damper = 0.001; //Sampling time in seconds.
IIR::ORDER order = IIR::ORDER::OD1; // Order (OD1 to OD4)
Filter damperFilter (cutoff_freq_damper, sampling_time_damper, order);
Filter interiaFilter (cutoff_freq_damper, sampling_time_damper, order);
Filter frictionFilter (cutoff_freq_damper, sampling_time_damper, order);

extern USBD_HandleTypeDef hUsbDeviceFS;
extern FFBConfig config;
PIDReportHandler pidReportHandler;

Gains *m_gains;

//force feedback effect params
EffectParams *m_effect_params;

bool FFB_effect_activated = false;

void setFilterParameter()
{
	float_t hz_ = config.SysConfig.Filter_Parameter.Cutoff_Freq;
	float_t ts_ = config.SysConfig.Filter_Parameter.SamplingTime;
	damperFilter.setCutoffFreqHZ(hz_, true);
	damperFilter.setSamplingTime(ts_, true);

	interiaFilter.setCutoffFreqHZ(hz_, true);
	interiaFilter.setSamplingTime(ts_, true);

	frictionFilter.setCutoffFreqHZ(hz_, true);
	frictionFilter.setSamplingTime(ts_, true);

}

void getForce (int32_t *forces)
{
  //RecvfromUsb();
  forceCalculator (forces);
}

int32_t getEffectForce (volatile TEffectState &effect, Gains _gains,
		EffectParams _effect_params, uint8_t axis)
{

  uint8_t direction;
  uint8_t condition;
  bool useForceDirectionForConditionEffect = (effect.enableAxis == DIRECTION_ENABLE && effect.conditionBlocksCount == 1);

  if (effect.enableAxis == DIRECTION_ENABLE)
    {
      direction = effect.directionX;
      condition = 0; // If the Direction Enable flag is set, only one Condition Parameter Block is defined
      if (effect.conditionBlocksCount > 1) {
              condition = axis;
         }
      else
         {

      	      condition = 0; // only one Condition Parameter Block is defined
      	 }
    }
  else
    {
      direction = axis == 0 ? effect.directionX : effect.directionY;
      condition = axis;
    }

  float angle = ((float)direction * 360.0 / 255.0) * DEG_TO_RAD;
  //float angle = ((float)direction * 360.0 / 65535.0) * DEG_TO_RAD;
  float angle_ratio = axis == 0 ? sin (angle) : -1 * cos (angle);

  int32_t force = 0;
  switch (effect.effectType)
    {
    case USB_EFFECT_CONSTANT: //1

    		force = ConstantForceCalculator (effect);
    		if(effect.useEnvelope)
			   {
				force += ApplyEnvelope(effect, (int32_t)force * _gains.constantGain * angle_ratio);
			   }
    		else
    		{
    			force += force * _gains.constantGain * angle_ratio;
    		}
      break;
    case USB_EFFECT_RAMP: //2
    		force = RampForceCalculator (effect);
    		if(effect.useEnvelope)
    		{
    			force -= ApplyEnvelope(effect, (int32_t)force * _gains.rampGain * angle_ratio) ;
    		}
    		else
    		{
    			force -= force *  _gains.rampGain * angle_ratio;
    		}
      break;
    case USB_EFFECT_SQUARE: //3
    		force = SquareForceCalculator (effect) * _gains.squareGain * angle_ratio;
      break;
    case USB_EFFECT_SINE: //4
    		force = SinForceCalculator (effect) * _gains.sineGain * angle_ratio;
      break;
    case USB_EFFECT_TRIANGLE: //5
    		force = TriangleForceCalculator (effect) * _gains.triangleGain * angle_ratio;
      break;
    case USB_EFFECT_SAWTOOTHDOWN: //6
    		force = SawtoothDownForceCalculator (effect) * _gains.sawtoothdownGain  * angle_ratio;
      break;
    case USB_EFFECT_SAWTOOTHUP: //7
    		force = SawtoothUpForceCalculator (effect) * _gains.sawtoothupGain* angle_ratio;
      break;
    case USB_EFFECT_SPRING: //8
			force += ConditionForceCalculator (effect, NormalizeRange (_effect_params.springPosition, _effect_params.springMaxPosition), 0.40f,condition) * _gains.springGain;

			if (useForceDirectionForConditionEffect)
				{
					force *= angle_ratio;
				}
      break;
    case USB_EFFECT_DAMPER: //9

			force += ConditionForceCalculator ( effect, NormalizeRange (_effect_params.damperVelocity, _effect_params.damperMaxVelocity), 0.4f, condition) * _gains.damperGain;


			if (useForceDirectionForConditionEffect)
				{
					force *= angle_ratio;
				}
      break;
    case USB_EFFECT_INERTIA: //10

    			effect.conditions[axis].negativeSaturation = -INERTIA_SATURATION;
    			effect.conditions[axis].positiveSaturation = INERTIA_SATURATION;
		  	//if (_effect_params.inertiaAcceleration < 0 && _effect_params.frictionPositionChange < 0)
			//{
		  			force = ConditionForceCalculator (effect, abs( NormalizeRange (_effect_params.inertiaAcceleration,
												  _effect_params.inertiaMaxAcceleration)), 0.40f,condition) * _gains.inertiaGain;
			//}
			  //else if (_effect_params.inertiaAcceleration < 0 && _effect_params.frictionPositionChange > 0)
			//{
					  //force = -1 * ConditionForceCalculator ( effect, abs ( NormalizeRange (_effect_params.inertiaAcceleration,
					//							  _effect_params.inertiaMaxAcceleration)), 0.70f,condition) * _gains.inertiaGain;
			//}


			if (useForceDirectionForConditionEffect)
				{
					force *= angle_ratio;
				}
      break;
    case USB_EFFECT_FRICTION: //11

    			//effect.conditions[axis].negativeSaturation = -FRICTION_SATURATION;
    			//effect.conditions[axis].positiveSaturation = FRICTION_SATURATION;
    		force += ConditionForceCalculator(effect, NormalizeRange(_effect_params.frictionPositionChange,
    										_effect_params.frictionMaxPositionChange), 0.4f, condition) * _gains.frictionGain;


			if (useForceDirectionForConditionEffect)
			{
				force *= angle_ratio;
			}
      break;
    case USB_EFFECT_CUSTOM: //12
      break;
    }
  effect.elapsedTime = (uint64_t) HAL_GetTick () - effect.startTime;
  return force;
}

void forceCalculator (int32_t *forces)
{
  forces[0] = 0;
  forces[1] = 0;
  //int32_t force = 0;
  FFB_effect_activated = false;
  for (int id = 0; id < MAX_EFFECTS; id++)
    {
      volatile TEffectState &effect = pidReportHandler.g_EffectStates[id];
      if ((effect.state == MEFFECTSTATE_PLAYING) && ((effect.elapsedTime <= effect.duration)
	      || (effect.duration == USB_DURATION_INFINITE))  && !pidReportHandler.devicePaused)
    	  {
			  if (effect.enableAxis == DIRECTION_ENABLE || effect.enableAxis & X_AXIS_ENABLE)
				{
				  forces[0] += (int32_t) (getEffectForce (effect, m_gains[0], m_effect_params[0], 0));
				  FFB_effect_activated = true;
				}

			  if (effect.enableAxis == DIRECTION_ENABLE || effect.enableAxis & Y_AXIS_ENABLE)
				{
				  forces[1] += (int32_t) (getEffectForce (effect, m_gains[1], m_effect_params[1], 1));
				  FFB_effect_activated = true;
				}

    	  }

    }
  // each effect gain * total effect gain = 10000
  forces[0] = (int32_t) ((float) 1.00 * forces[0] * m_gains[0].totalGain / 10000);
  // each effect gain * total effect gain = 10000
  forces[1] = (int32_t) ((float) 1.00 * forces[1] * m_gains[1].totalGain / 10000);
  //forces[0] = constrain(forces[0], -255, 255);
  //forces[1] = constrain(forces[1], -255, 255);
  forces[0] = constrain(forces[0], -32767, 32767);		//16 bits
  forces[1] = constrain(forces[1], -32767, 32767);


}

int32_t ConstantForceCalculator (volatile TEffectState &effect)
{

  float tempforce = ((float) effect.magnitude * (int32_t)(1 + effect.gain))/255;
  //float tempforce = ((float) effect.magnitude * (int32_t)(1 + effect.gain))/0xFFFF;
  //tempforce = map(tempforce, -10000, 10000, -255, 255);			//DAC resulusion 16 bit - remove this
  tempforce = map(tempforce, -10000, 10000, -32767, 32767);
  return (int32_t) tempforce;

}

int32_t RampForceCalculator (volatile TEffectState &effect)
{
	uint32_t duration = effect.duration;
	duration = (duration == 0) ? 0x7FFF : duration; // prevent div zero
  int32_t rampForce = effect.startMagnitude + effect.elapsedTime * (effect.endMagnitude - effect.startMagnitude)/ duration;
  rampForce *= (int32_t)(1 + effect.gain) >> 8;

  return rampForce;
}

int32_t SquareForceCalculator (volatile TEffectState &effect)
{
  int32_t offset = effect.offset * 2;
  uint32_t magnitude = effect.magnitude;
  uint32_t elapsedTime = effect.elapsedTime;
  uint32_t phase = effect.phase;
  uint32_t period = effect.period;

  int32_t maxMagnitude = offset + magnitude;
  int32_t minMagnitude = offset - magnitude;
  uint32_t phasetime = (phase * period) / 255;
  //uint32_t phasetime = (phase * period) / 35999;
  uint32_t timeTemp = elapsedTime + phasetime;
  uint32_t reminder = timeTemp % period;
  int32_t tempforce;
  if (reminder > (period / 2))
    tempforce = minMagnitude;
  else
    tempforce = maxMagnitude;
  return ApplyEnvelope (effect, tempforce);
}

int32_t SinForceCalculator (volatile TEffectState &effect)
{
  float offset = effect.offset * 2;
  float magnitude = effect.magnitude;
  float phase = effect.phase;
  float timeTemp = effect.elapsedTime;
  float period = effect.period;
  float angle = ((timeTemp / period) + (phase / 255) * period) * 2 * PI;
  //float angle = ((timeTemp / period) * 2 * PI + (float) (phase / 36000));
  float sine = sin (angle);
  float tempforce = sine * magnitude;
  tempforce += offset;
  return ApplyEnvelope (effect, tempforce);
}

int32_t TriangleForceCalculator (volatile TEffectState &effect)
{
  float offset = effect.offset * 2;
  float magnitude = effect.magnitude;
  float elapsedTime = effect.elapsedTime;
  uint32_t phase = effect.phase;
  uint32_t period = effect.period;
  float periodF = effect.period;

  float maxMagnitude = offset + magnitude;
  float minMagnitude = offset - magnitude;
  uint32_t phasetime = (phase * period) / 255;
  //uint32_t phasetime = (phase * period) / 35999;
  uint32_t timeTemp = elapsedTime + phasetime;
  float reminder = timeTemp % period;
  float slope = ((maxMagnitude - minMagnitude) * 2) / periodF;
  float tempforce = 0;
  if (reminder > (periodF / 2))
	  tempforce = slope * (periodF - reminder);
  else
	  tempforce = slope * reminder;
  tempforce += minMagnitude;
  return ApplyEnvelope (effect, tempforce);
}

int32_t SawtoothDownForceCalculator (volatile TEffectState &effect)
{
  float offset = effect.offset * 2;
  float magnitude = effect.magnitude;
  float elapsedTime = effect.elapsedTime;
  float phase = effect.phase;
  uint32_t period = effect.period;
  float periodF = effect.period;

  float maxMagnitude = offset + magnitude;
  float minMagnitude = offset - magnitude;
  int32_t phasetime = (phase * period) / 255;
  //int32_t phasetime = (phase * period) / 35999;
  uint32_t timeTemp = elapsedTime + phasetime;
  float reminder = timeTemp % period;
  float slope = (maxMagnitude - minMagnitude) / periodF;
  float tempforce = 0;
  tempforce = slope * (period - reminder);
  tempforce += minMagnitude;
  return ApplyEnvelope (effect, tempforce);
}

int32_t SawtoothUpForceCalculator (volatile TEffectState &effect)
{
  float offset = effect.offset * 2;
  float magnitude = effect.magnitude;
  float elapsedTime = effect.elapsedTime;
  uint32_t phase = effect.phase;
  uint32_t period = effect.period;
  float periodF = effect.period;

  float maxMagnitude = offset + magnitude;
  float minMagnitude = offset - magnitude;
  int32_t phasetime = (phase * period) / 255;
  //int32_t phasetime = (phase * period) / 35999;
  uint32_t timeTemp = elapsedTime + phasetime;
  float reminder = timeTemp % period;
  float slope = (maxMagnitude - minMagnitude) / periodF;
  float tempforce = 0;
  tempforce = slope * reminder;
  tempforce += minMagnitude;
  return ApplyEnvelope (effect, tempforce);
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

      tempForce = (metric - (cpOffset - deadBand)) * negativeCoefficient * scale;
      //tempForce = (metric -  scale * (cpOffset - deadBand) / 10000) * negativeCoefficient;


      tempForce = (tempForce < -negativeSaturation ? -negativeSaturation : tempForce);

    }
  else if (metric > (cpOffset + deadBand))
    {

      tempForce = (metric -  (cpOffset + deadBand)) * positiveCoefficient * scale;
      //tempForce = (metric -  scale * (cpOffset + deadBand) / 10000) * positiveCoefficient;
      tempForce = (tempForce > positiveSaturation ? positiveSaturation : tempForce);
    }
	  else return 0;

	  tempForce = -tempForce * effect.gain / 255;
	  //tempForce = -tempForce * effect.gain / 0x7FFF;
  switch (effect.effectType)
    {
    case USB_EFFECT_DAMPER:
    	setFilterParameter();
      tempForce = damperFilter.filterIn (tempForce);
      break;
    case USB_EFFECT_INERTIA:
    	setFilterParameter();
    	tempForce = interiaFilter.filterIn (tempForce);
      break;
    case USB_EFFECT_FRICTION:
    	setFilterParameter();
    	tempForce = frictionFilter.filterIn (tempForce);
      break;
    default:
      break;
    }
  //tempForce = map(tempForce, -10000, 10000, -255, 255);
  tempForce = map (tempForce, -10000, 10000, -32767, 32767);	//16 bits
  return (int32_t) tempForce;
}

float NormalizeRange (int32_t x, int32_t maxValue)
{
  return (float) x * 1.00 / maxValue;
}

int32_t ApplyGain (uint8_t value, uint8_t gain)
{
  int32_t value_32 = (int16_t) value;
  return ((value_32 * gain) / 255);
  //return ((value_32 * gain) / 0xFFFF);
}

int32_t ApplyEnvelope (volatile TEffectState &effect, int32_t value)
{
  int32_t magnitude = ApplyGain (effect.magnitude, effect.gain);
  int32_t attackLevel = ApplyGain (effect.attackLevel, effect.gain);
  int32_t fadeLevel = ApplyGain (effect.fadeLevel, effect.gain);
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
  if (elapsedTime > (duration - fadeTime))
    {
      newValue = (magnitude - fadeLevel) * (duration - elapsedTime);
      newValue /= fadeTime;
      newValue += fadeLevel;
    }

  newValue *= value;
  newValue /= 255;
  //newValue /= 0xFFFF; //16 bits
  return newValue;
}

void
end ()
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
