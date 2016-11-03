#include "Keys.h"
#include <unistd.h>
#include <signal.h>
#include <Gpio.h>
#include <WriteFile.h>
int gXenomaiInited = 0; // required by WriteFile's AuxiliaryTask
unsigned int gAuxiliaryTaskStackSize  = 1 << 17; // required by WriteFile's AuxiliaryTask
volatile int gShouldStop = 0;

void catch_function(int signo){
	gShouldStop = 1;
}

BoardsTopology bt;
WriteFile file;

void postCallback(void* arg, float* buffer, unsigned int length)
{
	Keys* keys = (Keys*)arg;
	unsigned int numKeys = bt.getHighestNote() - bt.getLowestNote() + 1;
	float values[numKeys];
	for(int n = bt.getLowestNote(); n <= bt.getHighestNote(); ++n)
	{
		values[n-bt.getLowestNote()] = keys->getNoteValue(n);
	}
	file.log(values, numKeys);
}

int main(int argc, char** argv)
{
	const int noinout = 0;
	const int in = 1;
	const int out = 2;
	bool lograw = false;
	int inout = noinout;
	char* path = NULL;
	while(++argv, --argc)
	{
		printf("arg: %s\n", *argv);
		if(strcmp(*argv, "in") == 0)
			inout = in;
		if(strcmp(*argv, "out") == 0)
			inout = out;
		if(strcmp(*argv, "raw") == 0)
		{
			lograw = true;
			inout = noinout;
			continue;
		}
		if(inout != noinout)
		{
			--argc;
			++argv;
			printf("subarg: %s\n", *argv);
			if(argc < 0)
			{
				fprintf(stderr, "You need to pass a filename after \"in\" or \"out\"");
				exit(1);
			}
			path = *argv;
		}
	}
	if(lograw)
	{
		file.init("rawsensors.bin"); //set the file name to write to
		file.setFileType(kBinary);
	}
	//Gpio testGpio;
	//testGpio.open(66, 1);
	//Gpio testGpio2;
	//testGpio2.open(67, 1);
	//Gpio testGpio3;
	//testGpio3.open(69, 1);
	signal(SIGINT, catch_function);
	Keys keys;
	keys.setDebug(false);
	// Let's build a topology for just above 5 octaves, A to C
	bt.setLowestNote(33);
	bt.setBoard(0, 0, 24);
	bt.setBoard(1, 0, 23);
	bt.setBoard(2, 0, 23);
	//bt.setBoard(0, 0, 23);
	//bt.setBoard(1, 0, 23);
	if(lograw)
	{
		keys.setPostCallback(postCallback, &keys);
	}
	int ret = keys.start(&bt, NULL);
	if(ret < 0)
	{
		fprintf(stderr, "Error while starting the scan of the keys: %d %s\n", ret, strerror(-ret));
		return 1;
	}

	if(inout == out)
	{
		keys.startTopCalibration();
		printf("Top calibration...");
		fflush(stdout);
		while(!gShouldStop && !keys.isTopCalibrationDone())
		{
			for(int n = bt.getLowestNote(); n <= bt.getHighestNote(); ++n)
				printf("%2d ", (int)(100 * keys.getNoteValue(n)));
			printf("\n");
			usleep(10000);
		}
		gShouldStop = 0;
		printf("done\n");
		keys.dumpTopCalibration();
	
		int count = 30000;
		printf("Bottom calibration...");
		fflush(stdout);
		keys.startBottomCalibration();
		while(!gShouldStop && count--)
		{
			printf("bot calib ");
			for(int n = bt.getLowestNote(); n <= bt.getHighestNote(); ++n)
				printf("%2d ", (int)(100 * keys.getNoteValue(n)));
			printf("\n");
			usleep(100000);	
		}
		gShouldStop = 0;
		printf("done\n");
		keys.stopBottomCalibration();
		keys.dumpBottomCalibration();
	
		keys.saveCalibrationFile(path);
	}
	if(inout == in)
	{
		keys.startTopCalibration();
		printf("Loading calibration file %s\n", path);
		keys.loadCalibrationFile(path);
	}
	if(inout == noinout)
	{
		keys.useCalibration(false);
	} else {
		keys.useCalibration(true);
	}
	while(!gShouldStop)
	{
		for(int n = bt.getLowestNote(); n <= bt.getHighestNote(); ++n)
			printf("%X ", (int)(10 * keys.getNoteValue(n)));
		printf("\n");
		usleep(100000);	
	}
	keys.stop();
	
	return 0;
}

