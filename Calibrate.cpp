#include "Calibrate.h"

void Calibration::setInverseSquareParams(unsigned int position, int32_t topValue, int16_t bottomValue, float a, float b, float c)
{
#if inv
	calibrationType = kInverseSquareCalibration;
	inverseSquareParams[position].a = a;
	inverseSquareParams[position].b = b;
	inverseSquareParams[position].c = c;
	std::cout << "Position: " << position << "(" << inverseSquareParams.size() << ")\n";
#endif
}

