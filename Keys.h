#ifndef KEYS_H_INCLUDED
#define KEYS_H_INCLUDED

#include <vector>
#include <array>
#include <inttypes.h>
#include "Boards.h"
#include "PruSpiKeysDriver.h"
#include "Calibrate.h"

#include <stdio.h>
class Keys
{
public:
	Keys() :
		_buffers()
		, _activeBuffer(false)
		, _shouldUseCalibration(false)
		, _postCallback(NULL)
	{};

	~Keys()
	{};
	
	void stop();
	void stopAndWait();

	int start(BoardsTopology* bt, volatile int* shouldStop = NULL);

	void setPostCallback(void(*postCallback)(void* arg, float* buffer, unsigned int length), void* arg)
	{
		_postCallback = postCallback;
		_postCallbackArg = arg;
	}
	/** 
	 * Receives a new buffer of raw data, converts it 
	 * and stores it in the inactive internal buffer.
	 * Once done, it switches over the active buffer.
	 */
	static void callback(void* obj);

	float getNoteValue(int note)
	{
		int lowestNote = _bt->getLowestNote();
		int highestNote = _bt->getHighestNote();
		if(note < lowestNote || note > highestNote){
			return -1;
		}
		else 
			return _buffers[_activeBuffer][note - lowestNote];
	}

	void setPriority(unsigned int priority);

	void setTimeResolution(unsigned int milliseconds);

	void startTopCalibration()
	{
		for(auto cal : calibration)
			delete cal;
		for(unsigned int n = 0; n < _bt->getNumBoards(); ++n)
		{
			int numKeys = _bt->getLastActiveKey(n) - _bt->getFirstActiveKey(n) + 1;
			calibration[n] = new Calibration(numKeys);
			_calibratingTop[n] = true;
		}
	}

	bool isTopCalibrationDone()
	{
		for(unsigned int n = 0; n < _calibratingTop.size(); ++n)
		{
			if(_calibratingTop[n])
				return false;
		}
		return true;
	}

	void stopTopCalibration()
	{
		for(unsigned int n = 0; n < _calibratingTop.size(); ++n)
		{
			_calibratingTop[n] = false;
		}
	}

	void dumpTopCalibration()
	{
		for(int n = _bt->getNumBoards() - 1; n >= 0; --n)
		{
			calibration[n]->dumpTopCalibration(_bt->getLowestNote(n));
		}
	}

	void dumpBottomCalibration()
	{
		for(int n = _bt->getNumBoards() - 1; n >= 0; n--)
		{
			calibration[n]->dumpBottomCalibration(_bt->getLowestNote(n));
		}
	}

	void startBottomCalibration()
	{
		unsigned int numBoards = _bt->getNumBoards();
		for(unsigned int n = 0; n < numBoards; ++n)
		{
			calibration[n]->startBottomCalibration();
			_calibratingBottom[n] = true;
		}
	}

	void stopBottomCalibration()
	{
		for(unsigned int n = 0; n < _calibratingBottom.size(); ++n)
		{
			_calibratingBottom[n] = false;
		}
		useCalibration(true);
	}

	bool loadCalibrationFile(const char* path);

	bool saveCalibrationFile(const char* path);

	void useCalibration(bool shouldUse)
	{
		_shouldUseCalibration = shouldUse;
	}

	void setDebug(bool should)
	{
		_debug = should;
		_driver._debug = should;
	}

private:
	PruSpiKeysDriver _driver;
	std::array<std::vector<float>, 2> _buffers;
	bool _activeBuffer;
	BoardsTopology* _bt;
	std::vector<bool> _calibratingTop;
	std::vector<bool> _calibratingBottom;
	std::vector<Calibration*> calibration;
	bool _shouldUseCalibration;
	void(*_postCallback)(void* arg, float* buffer, unsigned int length);
	void* _postCallbackArg;
	bool _debug = false;
};

// notes: 
// orders of boards should be reverse in updateBoards
// the driver should validate CRC
// the driver should allow to send and receive arbitrary SPI packets
// the driver shoudl allow to set priority
// the boards topology should be copied by Keys
// the driver should add waitForCurrentSpiTransactionToFinish() for synchronous transactions
//
#endif /* KEYS_H_INCLUDED */

