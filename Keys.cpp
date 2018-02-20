#include <vector>
#include <array>
#include "Keys.h"
#include <iostream>
#include <fstream>
#include <Gpio.h>

#define GPIO_DEBUG

#ifdef PRINT_ONE_KEY
extern "C"
{
	void rt_print_auto_init(int);
	int rt_printf(const char *format, ...);
};
#endif /* PRINT_ONE_KEY */

void Keys::stopAndWait()
{
	_driver.stopAndWait();
}

void Keys::stop()
{
	_driver.stop();
}

int Keys::start(BoardsTopology* bt, volatile int* shouldStop /* = NULL */)
{
#ifdef PRINT_ONE_KEY
	rt_print_auto_init(1);
#endif /* PRINT_ONE_KEY */
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
#ifdef GPIO_DEBUG
	static Gpio gpio0;
	static Gpio gpio1;
	static Gpio gpio2;
	static bool started = false;
	if(!started)
	{
		gpio0.open(86, 1);
		gpio1.open(88, 1);
		gpio2.open(87, 1);
		started = 1;
	}
	gpio0.set();
#endif /* GPIO_DEBUG */
	Keys* that = (Keys*)obj;
	PruSpiKeysDriver* driver = &that->_driver;
	BoardsTopology* bt = that->_bt;
	float* noteBuffer = that->_buffers[!that->_activeBuffer].data();
	unsigned int numBoards = bt->getNumBoards();
#ifdef GPIO_DEBUG
	gpio1.set();
#endif /* GPIO_DEBUG */
	for(int board = numBoards - 1; board >= 0; --board)
	{
		int16_t* boardBuffer = driver->getKeysData(board);
		int firstActiveKey = bt->getFirstActiveKey(board);
		int lastActiveKey = bt->getLastActiveKey(board);
		if(boardBuffer == NULL)
		{ 
#ifdef PRINT_ONE_KEY
			if(board == 0)
				rt_printf("n ");
#endif /* PRINT_ONE_KEY */
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
		if(that->_debug)
			fprintf(stderr, "Data available for board: %d\n", board);
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
#ifdef PRINT_ONE_KEY
				int note = i - (boardBuffer + firstActiveKey);
				if(board == 0 && note == 23)
				{
					rt_printf("%d ", *i);
					static int count = 0;
					count++;
					if(count == 20)
					{
						count = 0;
						rt_printf("\n");
					}
				}
#endif /* PRINT_ONE_KEY */
				float out = *i / 4096.f;
				if(out < 0)
					out = 0;
				*o = out;
			}
		}
	}
#ifdef GPIO_DEBUG
	gpio1.clear();
	gpio2.set();
#endif /* GPIO_DEBUG */
	// when we are done with updating the new buffer, we 
	// change the buffer in use.
	that->_activeBuffer = !that->_activeBuffer;
	if(that->_postCallback)
		that->_postCallback(that->_postCallbackArg, that->_buffers[that->_activeBuffer].data(), that->_buffers[that->_activeBuffer].size());
#ifdef GPIO_DEBUG
	gpio2.clear();
	gpio0.clear();
#endif /* GPIO_DEBUG */
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
