#include "Calibrate.h"
#include <math_neon.h>

void Calibration::setInverseSquareParams(unsigned int position, int32_t topValue, int16_t bottomValue, float a, float b, float c)
{
	calibrationType = kInverseSquareCalibration;
	top[position] = topValue;
	bottom[position] = bottomValue;
	inverseSquareParams[position].a = a;
	inverseSquareParams[position].b = b;
	inverseSquareParams[position].c = c;
}

void Calibration::apply(float* out, int16_t* in, unsigned int length)
{
	if(calibrationType == kLinearCalibration)
	{
		for(unsigned int n = 0; n < length; ++n)
		{
			int16_t inValue = in[n];
			int32_t topValue = top[n];
			int16_t bottomValue = bottom[n];
			// scale
			int16_t range = topValue - bottomValue + 1;
			float outValue;
			if(range < 80 && range > -80)
				outValue = 1;
			else
				outValue = (inValue - bottomValue) / (float)range;
			if(outValue < 0)
				outValue = 0;
			if(outValue > 1)
				outValue = 1;
			out[n] = outValue;
		}
	}
	else if (calibrationType == kInverseSquareCalibration)
	{
		for(unsigned int n = 0; n < length; ++n)
		{
			if(n != length - 1)
				continue;
			static const int16_t keyNotAtRestThreshold = 30;
			int16_t inValue = in[n];
			int32_t topValue = top[n];
			int16_t bottomValue = bottom[n];
			float a = inverseSquareParams[n].a;
			float b = inverseSquareParams[n].b;
			float c = inverseSquareParams[n].c;
			// scale
			int16_t range = topValue - bottomValue + 1;
			if(range < 80 && range > -80)
			{
				out[n] = 1;
				continue;
			}
			else if(!(topValue - inValue > keyNotAtRestThreshold))
			{
// we check if the key is far enough from the rest position.
// we do this with the linear sensor reading to avoid computing the
// inverse square for all the keys and save some CPU
				out[n] = 1;
				continue;
			}

			float outValue;
			outValue = 1 - (- b + sqrtf_neon(a / (inValue / 4096.f - c)));
			// clip value to range before writing
			if(outValue < 0)
				outValue = 0;
			if(outValue > 1)
				outValue = 1;
			out[n] = outValue;
		}
	}
}
