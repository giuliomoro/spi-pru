#ifndef KEYS_H_INCLUDED
#define KEYS_H_INCLUDED

#include <vector>
#include <array>
#include <inttypes.h>
#include <stdio.h>
#include <Gpio.h>

class Board
{
public:
	Board() {};

	~Board() {};
	
	void setKeys(unsigned int firstActiveKey, unsigned int lastActiveKey, unsigned int lowestNote);

	int getNote(unsigned int key);

	unsigned int getLastActiveKey(){
		return _lastActiveKey;
	}

	unsigned int getFirstActiveKey(){
		return _firstActiveKey;
	}

	unsigned int getNumActiveKeys(){
		int ret = _lastActiveKey - _firstActiveKey + 1;
		if(ret < 0)
			return 0;
		else
			return ret;
	}

private:
	unsigned int _firstActiveKey;
	unsigned int _lastActiveKey;
	unsigned int _lowestNote;
};

class BoardsTopology
{
public:
	BoardsTopology():
		_lowestNote(-1)
		, _highestNote(-1)
	{};

	~BoardsTopology()
	{
		deallocBoards();
	}

	void setLowestNote(unsigned int lowestNote)
	{
		_lowestNote = lowestNote;
		updateBoards();
	}

	void setBoard(unsigned int boardPosition, unsigned int firstActiveKey, unsigned int lastActiveKey);

	int getNote(unsigned int board, unsigned int key)
	{
		return boards[board]->getNote(key);
	}

	unsigned int getNumBoards()
	{
		return boards.size();
	}

	unsigned int getNumNotes();

	unsigned int getFirstActiveKey(unsigned int board)
	{
		return indexes[board * numIndexesPerBoard];
	}
	
	unsigned int getLastActiveKey(unsigned int board)
	{
		return indexes[board * numIndexesPerBoard + 1];
	}
	int getLowestNote()
	{
		return _lowestNote;
	}

	int getHighestNote()
	{
		return _highestNote;
	}

	int getLowestNote(unsigned int board)
	{
		return boards[board]->getNote(getFirstActiveKey(board));
	}

	int getHighestNote(unsigned int board)
	{
		return boards[board]->getNote(getLastActiveKey(board));
	}
private:
	void deallocBoard(unsigned int boardPosition);
	void deallocBoards();
	void updateBoards();
	void resizeBoards(unsigned int numBoards);

	unsigned int _lowestNote;
	unsigned int _highestNote;
	std::vector<Board*> boards;
	std::vector<unsigned int> indexes;
	static const unsigned int numIndexesPerBoard = 2;
};

// Driver header file
#include "prussdrv.h"
#include "pruss_intc_mapping.h"
#include <string.h>
#include <native/task.h>
#define PRU_NUM 1
#define PRU_DATA_BUFFER_SIZE 0x400
#define PRU_BOARD_BUFFER_SIZE 0x100
#define PRU_BOARD_KEYS_DATA_OFFSET 8

typedef struct {
	uint8_t buffers[PRU_DATA_BUFFER_SIZE * 2];
	uint32_t buffer;
	uint32_t activeBoards;
	uint32_t ticks;
	uint32_t ticksPerCallback;
	uint32_t requestMode;
	uint32_t actualMode;
} PruSpiKeysDriverContext;

class PruSpiKeysDriver
{
public:
	PruSpiKeysDriver() :
		_pruInited(false)
		, _pruEnabled(false)
		, _isPruRunning(false)
		, _isLoopRunning(false)
		, _validData(0)
		, context(NULL)
	{ }

	~PruSpiKeysDriver()
	{
		cleanup();
	}


	/**
	 * Enables the PRU, runs device discovery to detect which boards are
	 * connected.
	 *
	 * @return the boards discovered in case of success, a negative
	 * value otherwise.
	 */
	int init(int numBoards)
	{
		testGpio.open(44, 1);
		/* Initialize the PRU */
		int ret;
		if(!_pruInited)
		{
			ret = prussdrv_init();
			if (ret)
			{
				fprintf(stderr, "prussdrv_init failed\n");
				return (ret);
			}
			else
			{
				_pruInited = true;
			}
		}

		/* Open PRU Interrupt */
		ret = prussdrv_open(PRU_EVTOUT_0);
		if (ret)
		{
			fprintf(stderr, "prussdrv_open open failed\n");
			return (ret);
		}

		/* Map PRU's INTC */
		tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
		prussdrv_pruintc_init(&pruss_intc_initdata);

		prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);

		prussdrv_map_prumem (PRU_NUM == 0 ? PRUSS0_PRU0_DATARAM : PRUSS0_PRU1_DATARAM, (void **)&_pruMem);
		if(_pruMem == NULL){
			fprintf(stderr, "prussdrv_map_prumem failed\n");
			return -1;
		} 
		// construct the context object in the pru memory
		context = (PruSpiKeysDriverContext*) _pruMem;
		memset(context, 0, sizeof(PruSpiKeysDriverContext));

		if(!_pruEnabled)
		{
			if(prussdrv_exec_program (PRU_NUM, "/root/spi-pru/spi-pru.bin"))
			{
				fprintf(stderr, "Failed loading spi-pru program\n");
				return -1;
			}
			else 
			{
				_pruEnabled = true;
			}
		}
		return 1;
	}

	/**
	 * Starts the PRU loop in continuous scan mode:
	 * it will periodically request frames from the connected devices.
	 */
	int start(volatile int* shouldStop, void(*callback)(void*), void* arg)
	{
		if(shouldStop)
			_externalShouldStop = shouldStop;
		else
			_externalShouldStop = &_shouldStop;

		if(callback)
			_callback = callback;
		else 
			_callback = NULL;

		int ret = rt_task_create(&_loopTask, _loopTaskName, 0, _loopTaskPriority, T_FPU | T_JOINABLE | T_SUSP);
		if(ret)
			return ret;

		_callbackArg = arg;
		printf("TODO: set flag on PRU so that it switches to MASTER_MODE\n");

		ret = rt_task_start(&_loopTask, loop, this);
		if(ret){
			return ret;
		}

		_shouldStop = 0;

		ret = rt_task_resume(&_loopTask);
		if(ret){
			return ret;
		}
		return 1;
	}

	/**
	 * Stops the PRU loop.
	 */
	void stop()
	{
		_externalShouldStop = &_shouldStop;
		_shouldStop = true;
		_callback = NULL;
	}

	/**
	 * Checks whether the thread should stop.
	 */
	bool shouldStop()
	{
		return _shouldStop || *_externalShouldStop;
	}

	/**
	 * Disables the PRU and resets all the internal states.
	 */
	void cleanup()
	{
		_pruMem = NULL;
		_callback = NULL;
		if(_pruEnabled)
		{
			prussdrv_pru_disable(PRU_NUM);
			_pruEnabled = false;
		}
		if(_pruInited)
		{
			prussdrv_exit();
			_pruInited = false;
		}
	}

	int getBuffer()
	{
		return context->buffer;
	}

	uint8_t* getData()
	{
		return &context->buffers[PRU_DATA_BUFFER_SIZE * context->buffer];
	}

	static void loop(void* arg)
	{
		PruSpiKeysDriver* that = (PruSpiKeysDriver*)arg;
		int lastBuffer = that->getBuffer();
		while(!that->shouldStop()){
			int buffer = that->getBuffer();
			if(lastBuffer == buffer){
				rt_task_sleep(300000);
				continue;
			}
			that->testGpio.set();

			lastBuffer = buffer;
			// TODO: validate CRC and set valid data appropriately
			that->_validData = 0xff;

			that->_callback(that->_callbackArg);
			that->testGpio.clear();
		}
	}

	void setTimeResolution(uint32_t milliseconds)
	{
		context->ticksPerCallback = milliseconds;
	}

	uint32_t getTicks()
	{
		return context->ticks;
	}

	int16_t* getKeysData(unsigned int board)
	{
		if(wasDataValid(board))
			return (int16_t*)(getData() + board * PRU_BOARD_BUFFER_SIZE + PRU_BOARD_KEYS_DATA_OFFSET);
		else
			return NULL;
	}

private:
	bool wasDataValid(unsigned int board)
	{
		return (1 << board) & _validData;	
	}

	bool _pruInited;
	bool _pruEnabled;
	bool _isPruRunning;
	bool _isLoopRunning;
	RT_TASK _loopTask;
	uint32_t _validData;
	volatile int _shouldStop;
	volatile int* _externalShouldStop;
	uint8_t* _pruMem;
	void(*_callback)(void*);
	void* _callbackArg;
	PruSpiKeysDriverContext* volatile context;
	uint8_t* buffers[2];
	const unsigned int _loopTaskPriority = 90;
	const char* _loopTaskName = "SpiPruKeysDriverTask";
	Gpio testGpio;
};

class Keys
{
public:
	Keys() :
		_buffers()
		, _activeBuffer(false)
	{};

	~Keys()
	{};
	
	void stop()
	{
		_driver.stop();
	}

	int start(BoardsTopology* bt, volatile int* shouldStop = NULL)
	{
		testGpio.open(45, 1);
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

	/** 
	 * Receives a new buffer of raw data, converts it 
	 * and stores it in the inactive internal buffer.
	 * Once done, it switches over the active buffer.
	 */
	static void callback(void* obj)
	{
		Keys* that = (Keys*)obj;
		that->testGpio.set();
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

		that->testGpio.clear();
        return;
		printf("noteBuffer: %p (%d)|||", noteBuffer, that->_activeBuffer);
		for(int n = bt->getLowestNote(); n <= bt->getHighestNote(); ++n)
			printf("%1d ", (int)(that->getNoteValue(n)*10));
		printf("\n");
	}

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

private:
	PruSpiKeysDriver _driver;
	std::array<std::vector<float>, 2> _buffers;
	bool _activeBuffer;
	BoardsTopology* _bt;
	Gpio testGpio;
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

