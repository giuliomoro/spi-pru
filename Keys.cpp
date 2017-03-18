#include <vector>
#include <array>
#include "Keys.h"
#include <iostream>
#include <fstream>

void Keys::stop()
{
	_driver.stop();
}

int Keys::start(BoardsTopology* bt, volatile int* shouldStop /* = NULL */)
{
	stop();
	// TODO: wait for stop

	_bt = bt;
	for(auto &buffer : _buffers)
	{
		buffer.resize(_bt->getNumNotes());
		for(auto &val : buffer)
			val = 0;
	}

	int ret;
	ret = _driver.init(bt->getNumBoards());
	if(ret < 0)
		return ret;
		
	ret = _driver.start(shouldStop, Keys::callback, (void*)this);
	if(ret < 0)
		return ret;

	//TODO: scan boards and return number of boards found
	//
	return 1;
}

void Keys::callback(void* obj)
{
	Keys* that = (Keys*)obj;
	PruSpiKeysDriver* driver = &that->_driver;
	BoardsTopology* bt = that->_bt;
	float* noteBuffer = that->_buffers[!that->_activeBuffer].data();
	unsigned int numBoards = bt->getNumBoards();
	for(int board = numBoards - 1; board >= 0; --board)
	{
		int16_t* boardBuffer = driver->getKeysData(board);
		int firstActiveKey = bt->getFirstActiveKey(board);
		int lastActiveKey = bt->getLastActiveKey(board);
		if(boardBuffer == NULL)
		{ 
			// new data not available at the moment, copy over the old data
			int currentNote = bt->getLowestNote(board) - bt->getLowestNote();
			float* oldNoteBuffer = that->_buffers[that->_activeBuffer].data();
			float* i = oldNoteBuffer + currentNote;
			float* o = noteBuffer + currentNote;
			float* iEnd = i + lastActiveKey - firstActiveKey + 1;
			for(; i < iEnd; ++i, ++o)
			{
				*o = *i;
			}
			continue;
		}
		// new data available
		unsigned int length = lastActiveKey - firstActiveKey + 1;
		int16_t* boardBufferStart = boardBuffer + firstActiveKey;
		int currentNote = bt->getLowestNote(board) - bt->getLowestNote();
		if(that->_calibratingTop[board])
		{
			that->calibration[board]->calibrateTop(boardBufferStart, length);
			that->_calibratingTop[board] = !that->calibration[board]->isTopCalibrationDone();
		}
		else if(that->_calibratingBottom[board])
		{
			that->calibration[board]->calibrateBottom(boardBufferStart, length);
		}

		if(that->_shouldUseCalibration)
		{
			float* o = noteBuffer + currentNote;
			that->calibration[board]->apply(o, boardBufferStart, length);
		}
		else
		{
			//convert them to float
			float* o = noteBuffer + currentNote;
			int16_t* i = boardBuffer + firstActiveKey;
			int16_t* iEnd = boardBuffer + lastActiveKey + 1;
			for(; i < iEnd; ++i, ++o)
			{
				float out = *i / 4096.f;
				if(out < 0)
					out = 0;
				*o = out;
			}
		}


	}
	// when we are done with updating the new buffer, we 
	// change the buffer in use.
	that->_activeBuffer = !that->_activeBuffer;
	if(that->_postCallback)
		that->_postCallback(that->_postCallbackArg, that->_buffers[that->_activeBuffer].data(), that->_buffers[that->_activeBuffer].size());
	return;
}

bool Keys::saveCalibrationFile(const char* path)
{
	// format: [note#] [board] [top] [bottom]
	std::ofstream file;
	file.open(path);
	for(int board = _bt->getNumBoards() - 1; board >= 0; --board)
	{
		//file << "# Board: " << board << "\n";
		std::vector<int32_t> top = calibration[board]->getTop();
		std::vector<int16_t> bottom = calibration[board]->getBottom();
		unsigned int length = _bt->getHighestNote(board) - _bt->getLowestNote(board) + 1;
		for(unsigned int n = 0; n < length; ++n)
			file << n << " " << board << " " <<  top[n] << " " << bottom[n] << "\n";
	}
	file.close();
	return true;
}

bool Keys::loadCalibrationFile(const char* path)
{
	// format: [note#] [board] [top] [bottom]
	stopTopCalibration();
	stopBottomCalibration();
	std::ifstream inputFile;
	try
	{
		inputFile.open(path);
		if(inputFile.fail())
		{
			return false;
		}
		while(!inputFile.eof())
		{
			unsigned int key;
			int32_t top;
			int16_t bottom;
			unsigned int board;
			inputFile >> key;
			inputFile >> board;
			inputFile >> top;
			inputFile >> bottom;
			calibration[board]->set(key, top, bottom);
		}
	}
	catch(...)
	{
		return false;
	}
	useCalibration(true);
	return true;
}
