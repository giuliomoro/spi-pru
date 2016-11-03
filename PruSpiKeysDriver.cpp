#include <prussdrv.h>
#include <pruss_intc_mapping.h>
#include "PruSpiKeysDriver.h"
#include <inttypes.h>
#include </root/Bela/include/xenomai_wraps.h>
// Xenomai-specific includes
#if XENOMAI_MAJOR == 3
#include <xenomai/init.h>
#endif
extern int gXenomaiInited;

#define GPIO_DEBUG

static const uint32_t Polynomial = 0x04C11DB7;

static uint32_t crc32_bitwise(const void* data, size_t length)
{
  uint32_t previousCrc32 = 0;
  uint32_t crc = ~previousCrc32;
  unsigned char* current = (unsigned char*) data;
  while (length--)
  {
	unsigned int data = *current++;
    crc ^= data;
    for (unsigned int j = 0; j < 8; j++)
      crc = (crc >> 1) ^ (-(int)(crc & 1) & Polynomial);
  }
  return ~crc; // same as crc ^ 0xFFFFFFFF
}

Gpio PruSpiKeysDriver::gpios[4];
constexpr unsigned int PruSpiKeysDriver::gpioPins[4];
const unsigned int PruSpiKeysDriver::numGpios;

int PruSpiKeysDriver::init(unsigned int numBoards)
{
	_numBoards = numBoards;

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
	pruContext = (PruSpiKeysDriverContext*) _pruMem;
	memset(pruContext, 0, sizeof(PruSpiKeysDriverContext));
	context = new PruSpiKeysDriverContext;
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

	// Initialize the GPIO pins in use for CS and R/W lines
	for(unsigned int n = 0; n < numGpios; ++n)
	{
		gpios[n].open(gpioPins[n], 1);
	}
	return 1;
}

int PruSpiKeysDriver::start(volatile int* shouldStop, void(*callback)(void*), void* arg)
{
	if(shouldStop)
		_externalShouldStop = shouldStop;
	else
		_externalShouldStop = &_shouldStop;

	if(callback)
		_callback = callback;
	else 
		_callback = NULL;

	_callbackArg = arg;
	//printf("TODO: set flag on PRU so that it switches to MASTER_MODE\n");

	_shouldStop = 0;
	if(!gXenomaiInited)
	{
		int argc = 0;
		char *const *argv;
		xenomai_init(&argc, &argv);
		gXenomaiInited = 1;
	}
	int ret = create_and_start_thread(&_loopTask, _loopTaskName, _loopTaskPriority, 0, (pthread_callback_t*)loop, (void*)this);
	if(ret)
	{
		fprintf(stderr, "Failed to create thread: %d\n", ret);
		return 0;
	}

	return 1;
}

void PruSpiKeysDriver::stopAndWait()
{
	stop();
	__wrap_pthread_join(_loopTask, NULL);
}
void PruSpiKeysDriver::stop()
{
	_externalShouldStop = &_shouldStop;
	_shouldStop = true;
	_callback = NULL;
}

void PruSpiKeysDriver::cleanup()
{
	delete context;
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

#define LOCAL_COPY
void PruSpiKeysDriver::loop(void* arg)
{
#ifdef GPIO_DEBUG
	static bool init = false;
	static Gpio gpio3;
	if(!init)
	{
		init = true;
		gpio3.open(89, 1);
	}
#endif /* GPIO_DEBUG */
	PruSpiKeysDriver* that = (PruSpiKeysDriver*)arg;
	int lastBuffer = that->getActiveBuffer();
	that->_hasStopped = false;
	while(!that->shouldStop()){
		int buffer = that->getActiveBuffer();
		if(lastBuffer == buffer){
			task_sleep_ns(300000);
			continue;
		}

#ifdef GPIO_DEBUG
		gpio3.set();
#endif /* GPIO_DEBUG */
		lastBuffer = buffer;
		that->_validData = 0;

#ifdef LOCAL_COPY
		that->context->buffer = buffer;
		int baseOffset = PRU_DATA_BUFFER_SIZE * that->context->buffer;
		for(unsigned int n = 0; n < that->_numBoards; ++n)
		{
			int offset = baseOffset + n * PRU_BOARD_BUFFER_SIZE;
			memcpy(that->context->buffers + offset,
				that->pruContext->buffers + offset,
				sizeof(int16_t) * 36);
		}
#else /* LOCAL_COPY */
		that->context = that->pruContext;
#endif /* LOCAL_COPY */

		int activeBoards = that->pruContext->activeBoards;
		// int numBoards = __builtin_popcount(activeBoards); // __builtin_popcount returns the number of bits set

		// For each frame of data received, validate CRC and set _validData appropriately
		for(unsigned int n = 0; n < that->_numBoards; ++n)
		{
			uint8_t* data = that->getBoardData(n);
			int receivedLength = data[0];
			int moreData = data[1];
			int frameType = data[2];
			int zero = data[3];
			uint32_t timestamp;
			memcpy((void*)&timestamp, (void*)&data[4], 4);
			uint32_t paddedLength = (receivedLength + 3) & ~3;
			uint32_t receivedCrc;
			memcpy((void*)&receivedCrc, (void*)&data[paddedLength], 4);
			uint32_t computedCrc = crc32_bitwise((void*)data, paddedLength >> 2);
			if(that->_debug)
				fprintf(stderr, "Board%d (%d bytes):", n, paddedLength); 
			if(computedCrc != receivedCrc)
			{
				// invalid crc
				activeBoards = markBoardDisabled(n, activeBoards);
				if(that->_debug)
					fprintf(stderr, "invalid crc\n");
				continue;
			}
			if(frameType == 20)
			{
				// empty frame
				activeBoards = markBoardDisabled(n, activeBoards);
				if(that->_debug)
					fprintf(stderr, "empty frame\n");
				continue;
			}
			if(frameType != 19)
			{
				// unknwon frame
				activeBoards = markBoardDisabled(n, activeBoards);
				if(that->_debug)
				{
					fprintf(stderr, "unknown frame: %d\n", frameType);
					fprintf(stderr, "unknown frame: %d%d%d%d\n", data[0], data[1], data[2], data[3]);
				}
				continue;
			}
		}

		that->_validData = activeBoards;
		that->_callback(that->_callbackArg);
#ifdef GPIO_DEBUG
		gpio3.clear();
#endif /* GPIO_DEBUG */
	}
	that->_hasStopped = true;
}
