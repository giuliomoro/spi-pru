#include "Keys_c.h"
#include "Keys.h"

Keys* Keys_new()
{
	return new Keys;
}

void Keys_delete(Keys* that)
{
	delete that;
}

int Keys_start(Keys* that, BoardsTopology* bt, volatile int* shouldStop)
{
	return that->start(bt, shouldStop);
}

void Keys_stopAndWait(Keys* that)
{
	that->stopAndWait();
}

void Keys_stop(Keys* that)
{
	that->stop();
}

float Keys_getNoteValue(Keys* that, int note)
{
	return that->getNoteValue(note);
}

int Keys_loadInverseSquareCalibrationFile(Keys* that, const char* path, int offset)
{
	return that->loadInverseSquareCalibrationFile(path, offset);
}

int Keys_loadLinearCalibrationFile(Keys* that, const char* path)
{
	return that->loadLinearCalibrationFile(path);
}

int Keys_saveLinearCalibrationFile(Keys* that, const char* path)
{
	return that->saveLinearCalibrationFile(path);
}

void Keys_startTopCalibration(Keys* that)
{
	that->startTopCalibration();
}

int Keys_isTopCalibrationDone(Keys* that)
{
	return that->isTopCalibrationDone();
}

void Keys_stopTopCalibration(Keys* that)
{
	that->stopTopCalibration();
}


void Keys_dumpTopCalibration(Keys* that)
{
	that->dumpTopCalibration();
}

void Keys_startBottomCalibration(Keys* that)
{
	that->startBottomCalibration();
}

void Keys_stopBottomCalibration(Keys* that)
{
	that->stopBottomCalibration();
}

void Keys_dumpBottomCalibration(Keys* that)
{
	that->dumpBottomCalibration();
}

void Keys_useCalibration(Keys* that, int shouldUse)
{
	that->useCalibration(shouldUse);
}


void Keys_setHardClip(Keys* that, int shouldClip)
{
	that->setHardClip(shouldClip);
}

void Keys_setPostCallback(Keys* that, void(*postCallback)(void*, float*, unsigned int), void* arg)
{
	that->setPostCallback(postCallback, arg);
}

