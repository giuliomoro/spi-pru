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
			out[n] = outValue;
		}
	}
	else if (calibrationType == kInverseSquareCalibration)
	{
		for(unsigned int n = 0; n < length; ++n)
		{
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

			float outValue;
			outValue = 1 - (- b + sqrtf_neon(a / (inValue / 4096.f - c)));
			out[n] = outValue;
		}
	}
	if(hardClip)
	{
		for(unsigned int n = 0; n < length; ++n)
		{
			// clip value to range [0, 1]
			if(out[n] < 0)
				out[n] = 0;
			if(out[n] > 1)
				out[n] = 1;
		}
	}
}

void Calibration::calibrateTop(int16_t* buffer, int length)
{
	if(topCalibrationCount == 0)
	{
		initTopCalibration();
	}
	if(topCalibrationCount >= topCalibrateMax)
	{
		// we are done with calibration, why do you keep asking?
		return;
	}

	++topCalibrationCount;
	if(topCalibrationCount <= topCalibrateMin)
	{
		// we ignore the first few readings as they may be a bit wobbly
		return;
	}
	for(int n = 0; n < length; ++n)
	{
		top[n] += buffer[n];
	}

	if(topCalibrationCount == topCalibrateMax)
	{
		for(int n = 0; n < length; ++n)
		{
			top[n] /= topCalibrationCount - topCalibrateMin;
		}
	}
}

void Calibration::setHardClip(bool shouldHardClip)
{
	hardClip = shouldHardClip;
}
