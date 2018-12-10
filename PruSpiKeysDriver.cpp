#include <prussdrv.h>
#include <pruss_intc_mapping.h>
#include "PruSpiKeysDriver.h"
#include <inttypes.h>
#include </root/Bela/include/xenomai_wraps.h>
#include </opt/rtdm_pruss_irq/rtdm_pruss_irq.h>

#define PRU_SYSTEM_EVENT 21
#define PRU_INTC_CHANNEL 5
#define PRU_INTC_HOST PRU_INTC_CHANNEL
// Xenomai-specific includes
#if XENOMAI_MAJOR == 3
#include <xenomai/init.h>
#endif
extern int gXenomaiInited;

#define LOCAL_COPY
#define GPIO_DEBUG
#define PRU_USE_RTDM
#define PRU_SPI_XENOMAI

#ifndef PRU_SPI_XENOMAI
#undef PRU_USE_RTDM // can't use RTDM if not Xenomai!
inline int create_and_start_thread_nonrt(pthread_t* task, const char* taskName, int priority, int stackSize, pthread_callback_t* callback, void* arg)
{
	pthread_attr_t attr;
	if(pthread_attr_init(&attr))
	{
		fprintf(stderr, "Error: unable to init thread attributes\n");
		return -1;
	}
	if(int ret = set_thread_stack_and_priority(&attr, stackSize, priority)) // nothing Xenomai-specific in this call
	{
		return ret;
	}
	if(int ret = pthread_create(task, &attr, callback, arg))
	{
		return ret;
	}
	int ret = pthread_setname_np(*task, taskName);
	if(ret)
	{
		fprintf(stderr, "Error setting thread name: %d %s\n", ret, strerror(ret));
	}
	// check that effective parameters match the ones we requested
	//pthread_attr_t actualAttr;
	//pthread_getattr_np(*task, &actualAttr);
	//size_t stk;
	//pthread_attr_getstacksize(&actualAttr, &stk);
	//printf("measured stack: %d, requested stack: %d\n", stk, stackSize);

	pthread_attr_destroy(&attr);
	return 0;
}
#endif /* not PRU_SPI_XENOMAI */

#ifdef PRU_USE_RTDM
static char rtdm_driver[] = "/dev/rtdm/rtdm_pruss_irq_0";
static int rtdm_fd;
#endif /* PRU_USE_RTDM */

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

	_callback = callback;

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
	int ret;
#ifdef PRU_USE_RTDM
	// Open RTDM driver
	if ((rtdm_fd = __wrap_open(rtdm_driver, O_RDWR)) < 0) {
		fprintf(stderr, "Failed to open the kernel driver: (%d) %s.\n", errno, strerror(errno));
		if(errno == EBUSY) // Device or resource busy
		{
			fprintf(stderr, "Another program is already running?\n");
		}
		if(errno == ENOENT) // No such file or directory
		{
			fprintf(stderr, "Maybe try\n  modprobe rtdm_pruss_irq\n?\n");
		}
		gShouldStop = 1;
		return -1;
	}
	ret = __wrap_ioctl(rtdm_fd, RTDM_PRUSS_IRQ_REGISTER, PRU_SYSTEM_EVENT);
	if(ret)
	{
		fprintf(stderr, "ioctl failed: %d %s\n", -ret, strerror(-ret));
		return -1;
	}
#endif /* PRU_USE_RTDM */
#ifdef PRU_SPI_XENOMAI
	ret = create_and_start_thread(&_loopTask, _loopTaskName, _loopTaskPriority, 1 << 17, (pthread_callback_t*)loop, (void*)this);
#else /* PRU_SPI_XENOMAI */
	ret = create_and_start_thread_nonrt(&_loopTask, "PruSpiDriver", _loopTaskPriority, 1 << 17, (pthread_callback_t*)loop, (void*)this);
#endif /* PRU_SPI_XENOMAI */
	if(ret)
	{
		fprintf(stderr, "Failed to create thread: %d\n", ret);
		return -1;
	}

	if(!_pruInited)
	{
		fprintf(stderr, "PRU not inited\n");
		return -1;
	}
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

void PruSpiKeysDriver::stopAndWait()
{
	stop();
	if(_loopTask)
		__wrap_pthread_join(_loopTask, NULL);
	_loopTask = 0;
}
void PruSpiKeysDriver::stop()
{
	_externalShouldStop = &_shouldStop;
	_shouldStop = true;
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

void PruSpiKeysDriver::loop(void* arg)
{
#ifdef GPIO_DEBUG
	static bool init = false;
	int ret;
	static Gpio gpio3;
	static Gpio gpio4;
	if(!init)
	{
		init = true;
		gpio3.open(87, 1); // P8_29
		gpio4.open(89, 0); // P8_30
	}
#endif /* GPIO_DEBUG */
	PruSpiKeysDriver* that = (PruSpiKeysDriver*)arg;
	int lastBuffer = that->getActiveBuffer();
	that->_hasStopped = false;
	while(!that->shouldStop()){
#ifdef PRU_USE_RTDM
		ret = __wrap_read(rtdm_fd, NULL, 0);
#ifdef GPIO_DEBUG
		// terminate program if gpio4 is high
		while(!gShouldStop && gpio4.read())
		{
			usleep(100000);
		}
#endif /* GPIO_DEBUG */
		if(ret < 0)
		{
			static int interruptTimeoutCount = 0;
			++interruptTimeoutCount;
			fprintf(stderr, "SPI PRU interrupt timeout, %d %d %s\n", ret, errno, strerror(errno));
			if(interruptTimeoutCount >= 5)
			{
				fprintf(stderr, "The SPI PRU stopped responding. Quitting. If you are running a Bela program on the side, make sure to run it with: `--pru-number 0 -G 0`\n");
				gShouldStop = 1;
				break;
			}
			task_sleep_ns(100000000);
		}
#endif /* PRU_USE_RTDM */
		int buffer = that->getActiveBuffer();
#ifndef PRU_USE_RTDM
		if(lastBuffer == buffer){
#ifdef PRU_SPI_XENOMAI
			task_sleep_ns(300000);
#else /* PRU_SPI_XENOMAI */
			usleep(300);
#endif /* PRU_SPI_XENOMAI */
			continue;
		}
#endif /* PRU_USE_RTDM */

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
		if(that->_callback)
			that->_callback(that->_callbackArg);
#ifdef GPIO_DEBUG
		gpio3.clear();
#endif /* GPIO_DEBUG */
	}
	that->_hasStopped = true;
}
