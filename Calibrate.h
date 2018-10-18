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
		bottom(numKeys),
		inverseSquareParams(numKeys)
	{}

	void calibrateTop(int16_t* buffer, int length);

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

	void apply(float* out, int16_t* in, unsigned int length);

	void setHardClip(bool shouldHardClip);

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
	const int topCalibrateMin = 10;
	const int topCalibrateMax = 30;
	bool hardClip = false;

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
