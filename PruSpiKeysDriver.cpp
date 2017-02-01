#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#include "PruSpiKeysDriver.h"
int PruSpiKeysDriver::init(int numBoards)
{
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

void PruSpiKeysDriver::stop()
{
	_externalShouldStop = &_shouldStop;
	_shouldStop = true;
	_callback = NULL;
}

void PruSpiKeysDriver::cleanup()
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

void PruSpiKeysDriver::loop(void* arg)
{
	PruSpiKeysDriver* that = (PruSpiKeysDriver*)arg;
	int lastBuffer = that->getBuffer();
	while(!that->shouldStop()){
		int buffer = that->getBuffer();
		if(lastBuffer == buffer){
			rt_task_sleep(300000);
			continue;
		}

		lastBuffer = buffer;
		// TODO: validate CRC and set valid data appropriately
		that->_validData = 0xff;

		that->_callback(that->_callbackArg);
	}
}
