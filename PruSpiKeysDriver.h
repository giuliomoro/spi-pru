#ifndef PRUSPIKEYDRIVER_H_INCLUDED
#define PRUSPIKEYDRIVER_H_INCLUDED

#include <Gpio.h>
#include <string.h>
#include <pthread.h>

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
		,_loopTask(0)
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
	int init(unsigned int numBoards);

	/**
	 * Starts the PRU loop in continuous scan mode:
	 * it will periodically request frames from the connected devices.
	 */
	int start(volatile int* shouldStop, void(*callback)(void*), void* arg);

	/**
	 * Stops the PRU loop.
	 */
	void stop();
	void stopAndWait();

	/**
	 * Checks whether the thread should stop.
	 */
	bool shouldStop()
	// inlined for speed
	{
		return _shouldStop || *_externalShouldStop;
	}

	/**
	 * Disables the PRU and resets all the internal states.
	 */
	void cleanup();

	int getActiveBuffer()
	// inlined for speed
	{
		return pruContext->buffer;
	}

	uint8_t* getData()
	// inlined for speed
	{
		return &context->buffers[PRU_DATA_BUFFER_SIZE * context->buffer];
	}

	static void loop(void* arg);

	void setTimeResolution(uint32_t milliseconds)
	{
		context->ticksPerCallback = milliseconds;
	}

	uint32_t getTicks()
	{
		return context->ticks;
	}

	uint8_t* getBoardData(unsigned int board)
	// inlined for speed
	{
		return getData() + board * PRU_BOARD_BUFFER_SIZE;
	}

	int16_t* getKeysData(unsigned int board)
	// inlined for speed
	{
		if(wasDataValid(board))
			return (int16_t*)(getBoardData(board) + PRU_BOARD_KEYS_DATA_OFFSET);
		else
			return NULL;
	}

	bool _debug = false;

private:
	bool wasDataValid(unsigned int board)
	{
		return (1 << board) & _validData;	
	}

	static uint32_t markBoardDisabled(unsigned int board, uint32_t activeBoards)
	{
		return activeBoards &= ~(1 << board);
	}

	bool _pruInited;
	bool _pruEnabled;
	bool _isPruRunning;
	bool _isLoopRunning;
	pthread_t _loopTask;
	uint32_t _validData;
	volatile int _shouldStop;
	volatile int* _externalShouldStop;
	uint8_t* _pruMem;
	unsigned int _numBoards;
	void(*_callback)(void*);
	void* _callbackArg;
	PruSpiKeysDriverContext* volatile pruContext;
	PruSpiKeysDriverContext* context = NULL;
	uint8_t* buffers[2];
	const unsigned int _loopTaskPriority = 90;
	const char* _loopTaskName = "SpiPruKeysDriverTask";
	static Gpio gpios[4];
	static const unsigned int numGpios = 4;
	static constexpr unsigned int gpioPins[numGpios] = {
		68, // P8_10, R/W
		66, // P8_07, CS0
		67, // P8_08, CS1
		69  // P8_09, CS2
	};
	bool _hasStopped = true;
};
#endif /* PRUSPIKEYDRIVER_H_INCLUDED */
