#include <vector>
#include <array>
#include "Keys.h"

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
		// new data available, we convert them
		int currentNote = bt->getLowestNote(board) - bt->getLowestNote();
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
	// when we are done with updating the new buffer, we 
	// change the buffer in use.
	that->_activeBuffer = !that->_activeBuffer;

	return;
}
