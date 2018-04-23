#include "Keys.h"
#include <unistd.h>
#include <signal.h>
#include <Gpio.h>
#include <WriteFile.h>
#include <xenomai/init.h>
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

void usage()
{
	printf(
			"single <note> : only display full-precision value for the given key"
			"raw         : log raw values as they come in from the sensor to rawsensors.bin"
			"in <path>   : load inverse square calibration file from <path> and display calibrated values "
			"out <path>  : generates file  <path> with key top and key bottom values. "
			"              make sure no key is been pressed. Start, wait for key values for be displayed"
                        "              and then start pressing all keys. Hit ctrl-C when done. Once done, will display"
			"              normalized readings.");
}

int main(int argc, char** argv)
{
	const int noinout = 0;
	const int in = 1;
	const int out = 2;
	int singleKey = -1;
	bool lograw = false;
	int inout = noinout;
	char* path = NULL;
	while(++argv, --argc)
	{
		printf("arg: %s\n", *argv);
		if(strcmp(*argv, "--help") == 0)
		{
			usage();
			return 0;
		}
		if(strcmp(*argv, "single") == 0)
		{
			--argc;
			++argv;
			singleKey = atoi(*argv);
			printf("singleKey: %d\n", singleKey);
			continue;
		}
		if(strcmp(*argv, "raw") == 0)
		{
			lograw = true;
			inout = noinout;
			continue;
		}
		if(strcmp(*argv, "in") == 0)
			inout = in;
		if(strcmp(*argv, "out") == 0)
			inout = out;
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
	if(!gXenomaiInited)
	{ //initing xenomai
		int argc = 0;
		char *const *argv;
		xenomai_init(&argc, &argv);
		gXenomaiInited = 1;
	}
	if(lograw)
	{
		file.init("rawsensors.bin"); //set the file name to write to
		file.setFileType(kBinary);
	}
	signal(SIGINT, catch_function);
	Keys keys;
	keys.setDebug(false);
	// Let's build a topology for 6 octaves, C to C
	bt.setLowestNote(24);
	bt.setBoard(0, 0, 24);
	bt.setBoard(1, 0, 23);
	bt.setBoard(2, 0, 23);
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
	
		printf("Bottom calibration...");
		fflush(stdout);
		keys.startBottomCalibration();
		while(!gShouldStop)
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
	
		keys.saveLinearCalibrationFile(path);
	}
	if(inout == in)
	{
		keys.startTopCalibration();
		printf("Loading calibration file %s\n", path);
		keys.loadInverseSquareCalibrationFile(path);
	}
	if(inout == noinout)
	{
		keys.useCalibration(false);
	} else {
		keys.useCalibration(true);
	}
	while(!gShouldStop)
	{
		if(singleKey >= 0)
		{
			printf("%.4f\n", keys.getNoteValue(singleKey));
			usleep(30000);	
		}
		else
		{
			for(int n = bt.getLowestNote(); n <= bt.getHighestNote(); ++n)
				printf("%X ", (int)(10 * keys.getNoteValue(n)));
			printf("\n");
			usleep(100000);	
		}
	}
	keys.stop();
	
	return 0;
}

