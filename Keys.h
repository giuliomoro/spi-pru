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
	{
		for(auto cal : calibration)
		{
			delete cal;
		}
	};
	
	void stop();
	void stopAndWait();

	int start(BoardsTopology* bt, volatile int* shouldStop = NULL);

	void setPostCallback(void(*postCallback)(void* arg, float* buffer, unsigned int length), void* arg);

	/** 
	 * Receives a new buffer of raw data, converts it 
	 * and stores it in the inactive internal buffer.
	 * Once done, it switches over the active buffer.
	 */
	static void callback(void* obj);

	// inlined for speed
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

	void startTopCalibration();

	bool isTopCalibrationDone();

	void stopTopCalibration();

	void dumpTopCalibration();

	void dumpBottomCalibration();

	void startBottomCalibration();

	void stopBottomCalibration();

	bool loadInverseSquareCalibrationFile(const char* path, int offset);

	bool loadLinearCalibrationFile(const char* path);

	bool saveLinearCalibrationFile(const char* path);

	void useCalibration(bool shouldUse);

	void setHardClip(bool shouldClip);

	void setDebug(bool should);

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

