#include <usbd_joystick_hid_if.h>
#include "PIDReportHandler.h"
#include "main.h"
#include "FFBMain.h"

PIDReportHandler::PIDReportHandler ()
{
  nextEID = 1;
  devicePaused = 0;
}

PIDReportHandler::~PIDReportHandler ()
{
  FreeAllEffects ();
}

uint8_t PIDReportHandler::GetNextFreeEffect (void)
{
  if (nextEID == MAX_EFFECTS)
    return 0;

  uint8_t id = nextEID++;

  while (g_EffectStates[nextEID].state != 0)
    {
      if (nextEID >= MAX_EFFECTS)
	break;  // the last spot was taken
      nextEID++;
    }

  g_EffectStates[id].state = MEFFECTSTATE_ALLOCATED;

  return id;
}

void PIDReportHandler::sendStatusReport ()
{

  uint8_t rp[sizeof(USB_FFBReport_PIDStatus_Input_Data_t)];
  rp[0] = pidState.reportId;
  rp[1] = pidState.status;
  rp[2] = pidState.effectBlockIndex;

  USBD_JOYSTICK_HID_SendReport_FS (
      rp, sizeof(USB_FFBReport_PIDStatus_Input_Data_t));
}

void PIDReportHandler::StopAllEffects (void)
{
  for (uint8_t id = 0; id <= MAX_EFFECTS; id++)
    StopEffect (id);
}

void PIDReportHandler::StartEffect (uint8_t id)
{
  if (id > MAX_EFFECTS)
    return;
  g_EffectStates[id].state = MEFFECTSTATE_PLAYING;
  g_EffectStates[id].elapsedTime = 0;
  g_EffectStates[id].startTime = (uint64_t) HAL_GetTick ();
}

void PIDReportHandler::StopEffect (uint8_t id)
{
  if (id > MAX_EFFECTS)
    return;
  g_EffectStates[id].state &= ~MEFFECTSTATE_PLAYING;
  pidBlockLoad.ramPoolAvailable += SIZE_EFFECT;
}

void PIDReportHandler::FreeEffect (uint8_t id)
{
  if (id > MAX_EFFECTS)
    return;
  g_EffectStates[id].state = 0;
  if (id < nextEID)
    nextEID = id;
}

void PIDReportHandler::FreeAllEffects (void)
{
  nextEID = 1;
  memset ((void*) &g_EffectStates, 0, sizeof(g_EffectStates));
  pidBlockLoad.ramPoolAvailable = MEMORY_SIZE;
}

void PIDReportHandler::EffectOperation (USB_FFBReport_EffectOperation_Output_Data_t *data)
{
  if (data->operation == 1)
    { // Start
      if (data->loopCount > 0)
	g_EffectStates[data->effectBlockIndex].duration *= data->loopCount;
      if (data->loopCount == 0xFF)
    	  g_EffectStates[data->effectBlockIndex].duration = USB_DURATION_INFINITE;

      StartEffect (data->effectBlockIndex);
    }
  else if (data->operation == 2)
    {
      StopAllEffects ();
      StartEffect (data->effectBlockIndex);
    }
  else if (data->operation == 3)
    {
      StopEffect (data->effectBlockIndex);
    }
  else
    {
    }
}

void PIDReportHandler::BlockFree (USB_FFBReport_BlockFree_Output_Data_t *data)
{
  uint8_t eid = data->effectBlockIndex;

  if (eid == 0xFF)
    { // all effects
      FreeAllEffects ();
    }
  else
    {
      FreeEffect (eid);
    }
}

void PIDReportHandler::DeviceControl (
    USB_FFBReport_DeviceControl_Output_Data_t *data)
{

  pidState.reportId = 2;
  Bset(pidState.status, SAFETY_SWITCH);
  Bset(pidState.status, ACTUATOR_POWER);
  pidState.effectBlockIndex = 0;

  switch (data->control)
    {
    case 0x01:
      // enable  actuators
      Bclr(pidState.status, ACTUATORS_ENABLED);
      //printf("Effects: Enabled.\n");
      break;
    case 0x02:
      // disable actuators
      Bset(pidState.status, ACTUATORS_ENABLED);
      //printf("Effects: Disabled.\n");
      break;
    case 0x03:
      // Disable auto-center spring and stop all effects
      //SetAutoCenter(0);
      StopAllEffects ();
      //printf("Effects: Stop All.\n");
      break;
    case 0x04:
      // Reset (e.g. FFB-application out of focus)
      FreeAllEffects ();
      //printf("Effects: Free All.\n");
      break;
    case 0x05:
      // pause
      Bset(pidState.status, DEVICE_PAUSED);
      devicePaused = 1;
      //printf("Effects: Paused.\n");
      break;
    case 0x06:
      // continue
      Bclr(pidState.status, DEVICE_PAUSED);
      devicePaused = 0;
      //printf("Effects: Continue.\n");
      break;
    default:
      return;
    }

}

void PIDReportHandler::DeviceGain (USB_FFBReport_DeviceGain_Output_Data_t *data)
{
  deviceGain.gain = data->gain;
}

void PIDReportHandler::SetCustomForce (USB_FFBReport_SetCustomForce_Output_Data_t *data)
{
}

void PIDReportHandler::SetCustomForceData (USB_FFBReport_SetCustomForceData_Output_Data_t *data)
{
}

void PIDReportHandler::SetDownloadForceSample (USB_FFBReport_SetDownloadForceSample_Output_Data_t *data)
{
}

void PIDReportHandler::SetEffect (USB_FFBReport_SetEffect_Output_Data_t *data)
{
	uint8_t index = data->effectBlockIndex;
		if(index > MAX_EFFECTS || index == 0)
			return;

  volatile TEffectState *effect = &g_EffectStates[data->effectBlockIndex];

  effect->duration = data->duration;
  effect->directionX = data->directionX;
  effect->directionY = data->directionY;
  effect->effectType = data->effectType;
  effect->gain = data->gain;
  effect->enableAxis = data->enableAxis;

}

void PIDReportHandler::SetEnvelope (USB_FFBReport_SetEnvelope_Output_Data_t *data, volatile TEffectState *effect)
{
  effect->attackLevel = data->attackLevel;
  effect->fadeLevel = data->fadeLevel;
  effect->attackTime = data->attackTime;
  effect->fadeTime = data->fadeTime;
  effect->useEnvelope = true;
}

void PIDReportHandler::SetCondition (USB_FFBReport_SetCondition_Output_Data_t *data, volatile TEffectState *effect)
{
  uint8_t axis = data->parameterBlockOffset;
  effect->conditions[axis].cpOffset = data->cpOffset;
  effect->conditions[axis].positiveCoefficient = data->positiveCoefficient;
  effect->conditions[axis].negativeCoefficient = data->negativeCoefficient;
  effect->conditions[axis].positiveSaturation = data->positiveSaturation;
  effect->conditions[axis].negativeSaturation = data->negativeSaturation;
  effect->conditions[axis].deadBand = data->deadBand;
  effect->axesIdx = (axis + 1 > effect->axesIdx) ? (axis + 1) : (effect->axesIdx);
  effect->conditionBlocksCount++;
}

void PIDReportHandler::SetPeriodic (USB_FFBReport_SetPeriodic_Output_Data_t *data, volatile TEffectState *effect)
{
  effect->magnitude = data->magnitude;
  effect->offset = data->offset;
  effect->phase = data->phase;
  effect->period = constrain(data->period,1,32767); //prevent priod = 0
}

void PIDReportHandler::SetConstantForce (USB_FFBReport_SetConstantForce_Output_Data_t *data,volatile TEffectState *effect)
{
  //  ReportPrint(*effect);
  effect->magnitude = data->magnitude;
}

void PIDReportHandler::SetRampForce (USB_FFBReport_SetRampForce_Output_Data_t *data,
				volatile TEffectState *effect)
{
  effect->startMagnitude = data->startMagnitude;
  effect->endMagnitude = data->endMagnitude;
}

void PIDReportHandler::CreateNewEffect (USB_FFBReport_CreateNewEffect_Feature_Data_t *inData)
{
  pidBlockLoad.reportId = 6;
  pidBlockLoad.effectBlockIndex = GetNextFreeEffect ();

  if (pidBlockLoad.effectBlockIndex == 0)
    {
      pidBlockLoad.loadStatus = 2;    // 1=Success,2=Full,3=Error
    }
  else
    {
      pidBlockLoad.loadStatus = 1;    // 1=Success,2=Full,3=Error

      volatile TEffectState *effect = &g_EffectStates[pidBlockLoad.effectBlockIndex];

      memset ((void*) effect, 0, sizeof(TEffectState));
      effect->state = MEFFECTSTATE_ALLOCATED;
      pidBlockLoad.ramPoolAvailable -= SIZE_EFFECT;
    }
  //printf("Effects: CreateNewEffect().\n");
}

void PIDReportHandler::UppackUsbData (uint8_t *data, uint16_t len)
{
	 //printf("UppackUsbData: ReportID(%d).\n", data[0]);
  uint8_t effectId = data[1]; // effectBlockIndex is always the second byte.
  switch (data[0]) // reportID
    {
    case 1:
    	 //printf("Effects: SetEffect().\n");
      SetEffect ((USB_FFBReport_SetEffect_Output_Data_t*) data);

      break;
    case 2:
    	//printf("Effects: SetEnvelope(%d).\n" ,effectId);
      SetEnvelope ((USB_FFBReport_SetEnvelope_Output_Data_t*) data, &g_EffectStates[effectId]);
      break;
    case 3:
    	//printf("Effects: SetCondition(%d).\n",effectId);
      SetCondition ((USB_FFBReport_SetCondition_Output_Data_t*) data, &g_EffectStates[effectId]);
      break;
    case 4:
    	//printf("Effects: SetPeriodic(%d).\n",effectId);
      SetPeriodic ((USB_FFBReport_SetPeriodic_Output_Data_t*) data, &g_EffectStates[effectId]);
      break;
    case 5:
    	//printf("Effects: SetConstantForce(%d).\n",effectId);
      SetConstantForce ((USB_FFBReport_SetConstantForce_Output_Data_t*) data, &g_EffectStates[effectId]);
      break;
    case 6:
    	//printf("Effects: SetRampForce(%d).\n",effectId);
      SetRampForce ((USB_FFBReport_SetRampForce_Output_Data_t*) data, &g_EffectStates[effectId]);
      break;
    case 7:
    	//printf("Effects: SetCustomForceData().\n");
      SetCustomForceData ((USB_FFBReport_SetCustomForceData_Output_Data_t*) data);
      break;
    case 8:
    	//printf("Effects: SetDownloadForceSample().\n");
      SetDownloadForceSample ((USB_FFBReport_SetDownloadForceSample_Output_Data_t*) data);
      break;
    case 9:
      break;
    case 10:
    	//printf("Effects: EffectOperation().\n");
      EffectOperation ((USB_FFBReport_EffectOperation_Output_Data_t*) data);
      sendStatusReport ();
      break;
    case 11:
    	//printf("Effects: BlockFree().\n");
      BlockFree ((USB_FFBReport_BlockFree_Output_Data_t*) data);
      break;
    case 12:
    	//printf("Effects: DeviceControl().\n");
      DeviceControl ((USB_FFBReport_DeviceControl_Output_Data_t*) data);
      sendStatusReport ();
      break;
    case 13:
    	//printf("Effects: DeviceGain().\n");
      DeviceGain ((USB_FFBReport_DeviceGain_Output_Data_t*) data);
      break;
    case 14:
    	//printf("Effects: SetCustomForce().\n");
      SetCustomForce ((USB_FFBReport_SetCustomForce_Output_Data_t*) data);
      break;
    case 0x11:
    	//printf("Effects: CreateNewEffect().\n");
      CreateNewEffect ((USB_FFBReport_CreateNewEffect_Feature_Data_t*) data);
      break;
    default:
      break;
    }
}

uint8_t* PIDReportHandler::getPIDPool ()
{
  FreeAllEffects ();

  pidPoolReport.reportId = 7;
  pidPoolReport.ramPoolSize = MEMORY_SIZE;
  pidPoolReport.maxSimultaneousEffects = MAX_EFFECTS;
  pidPoolReport.memoryManagement = 3;
  return (uint8_t*) &pidPoolReport;
}

uint8_t* PIDReportHandler::getPIDBlockLoad ()
{
  return (uint8_t*) &pidBlockLoad;
}

uint8_t* PIDReportHandler::getPIDStatus ()
{
  return (uint8_t*) &pidState;
}

