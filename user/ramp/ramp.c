#include "ramp.h"
#include "stm32f4xx_hal.h"

void ramp_init(ramp_t *ramp, int32_t scale)
{
  ramp->count = 0;
  ramp->scale = scale;
}

float ramp_calc(ramp_t *ramp)
{
  if (ramp->scale <= 0)
    return 0;
  
  if (ramp->count++ >= ramp->scale)
    ramp->count = ramp->scale;
  
  ramp->out = ramp->count * 1.0f / ((float)ramp->scale);
  return ramp->out;
}
