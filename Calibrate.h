#ifndef CALIBRATE_H_INCLUDED
#define CALIBRATE_H_INCLUDED
#include <vector>
#include <limits>
#include <iostream>
#include <string.h>

class Calibration
{
public:
	Calibration(int numKeys) :
		topCalibrationCount(0),
		calibrationType(0),
		top(numKeys),
		bottom(numKeys)
		,
		inverseSquareParams(numKeys)
	{
		std::cout << top.data() << " " << bottom.data() << " " 
			<< inverseSquareParams.data() 
			<< "\n";
	}

	~Calibration()
	{
		std::cout << top.data() << " " << bottom.data() << " " 
			<< inverseSquareParams.data() 
			<< "\n";
	}

	void calibrateTop(int16_t* buffer, int length)
	{
		if(topCalibrationCount == 0)
		{
			initTopCalibration();
		}

		for(int n = 0; n < length; ++n)
		{
			top[n] += buffer[n];
		}

		++topCalibrationCount;
		if(topCalibrationCount == topCalibrateMax)
		{
			for(int n = 0; n < length; ++n)
			{
				top[n] /= topCalibrationCount;
			}
		}
	}

	void setInverseSquareParams(unsigned int position, int32_t topValue, int16_t bottomValue, float a, float b, float c);

	void set(unsigned int position, int32_t topValue, int16_t bottomValue)
	{
		calibrationType = kLinearCalibration;
		top[position] = topValue;
		bottom[position] = bottomValue;
	}

	bool isTopCalibrationDone()
	{
		return topCalibrationCount == topCalibrateMax;
	}

	void calibrateBottom(int16_t* buffer, unsigned int length)
	{
		for(unsigned int n = 0; n < length; ++n)
		{
			bottom[n] = buffer[n] < bottom[n] ? buffer[n] : bottom[n];
		}
		++bottomCalibrationCount;
	}

	void dumpTopCalibration(int offset = 0)
	{
		unsigned int line = 12;
		for(unsigned int n = 0; n < top.size(); ++n)
		{
			printf("[%3d]%5d ", n + offset, top[n]);
			if(n % line == line - 1)
				printf("\n");
		}
		printf("\n");
	}

	void dumpBottomCalibration(int offset = 0)
	{
		unsigned int line = 12;
		for(unsigned int n = 0; n < bottom.size(); ++n)
		{
			printf("[%3d]%5d ", n + offset, bottom[n]);
			if(n % line == line - 1)
				printf("\n");
		}
		printf("\n");
	}

	void startBottomCalibration()
	{
		initBottomCalibration();
		bottomCalibrationCount = 0;
	}

	std::vector<int16_t>& getBottom()
	{
		return bottom;
	}

	std::vector<int32_t>& getTop()
	{
		return top;
	}

	void apply(float* out, int16_t* in, unsigned int length)
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
				float x = inValue / 4096.f;
				outValue =  a / (x * x + b) + c;
				// clip value to range before writing
				if(outValue < 0)
					outValue = 0;
				if(outValue > 1)
					outValue = 1;
				out[n] = outValue;
			}
		}
	}

private:
	int topCalibrationCount;
	int bottomCalibrationCount;
	int calibrationType;
	std::vector<int32_t> top;
	std::vector<int16_t> bottom;
	typedef struct _InverseSquareCalibParams
	{
		float a;
		float b;
		float c;
	} InverseSquareParams;
	std::vector<InverseSquareParams> inverseSquareParams;
	const int topCalibrateMax = 30;

	void initBottomCalibration()
	{
		for(auto& it : bottom)
			it = std::numeric_limits<int16_t>::max();
	}

	void initTopCalibration()
	{
		memset(top.data(), 0, sizeof(top[0])*top.size());
	}

	enum
	{
		kLinearCalibration,
		kInverseSquareCalibration,
	};
};

#endif /* CALIBRATE_H_INCLUDED */
