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
		if(boardBuffer == NULL)
		{ 
			// new data not available at the moment, copy over the old data
			printf("TODO: copy over the old data %d\n", board);
			//TODO
			continue;
		}
		// new data available, we convert them
		int firstActiveKey = bt->getFirstActiveKey(board);
		int lastActiveKey = bt->getLastActiveKey(board);
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
	printf("noteBuffer: %p (%d)|||", noteBuffer, that->_activeBuffer);
	for(int n = bt->getLowestNote(); n <= bt->getHighestNote(); ++n)
		printf("%1d ", (int)(that->getNoteValue(n)*10));
	printf("\n");
}
