#include "Keys.h"
#include <unistd.h>
#include <signal.h>
#include <Gpio.h>
#include <WriteFile.h>
#include <xenomai/init.h>
#include <vector>
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
			"single <note> : only display high-precision value for the given key"
			"range <note> <note> : only display high-precision values for the given keys"
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
	std::vector<int> displayKeys;
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
			int singleKey = atoi(*argv);
			displayKeys.resize(1);
			displayKeys[0] = singleKey;
			printf("singleKey: %d\n", singleKey);
			continue;
		}
		if(strcmp(*argv, "range") == 0)
		{
			--argc;
			++argv;
			int start = atoi(*argv);
			--argc;
			++argv;
			int stop = atoi(*argv);
			if(start > stop)
			{
				int oldStop = stop;
				stop = start;
				start = oldStop;
			}
			int numKeys = stop - start;
			displayKeys.resize(numKeys);
			for(int n = 0; n < numKeys; ++n)
			{
				displayKeys[n] = start + n;
			}
			printf("key range: %d to %d\n", start, stop);
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
	bt.setLowestNote(0);
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
	if(displayKeys.size() > 0)
	{
		for(unsigned int n = 0; n < displayKeys.size(); ++n)
		{
			printf("%5d |", displayKeys[n]);
		}
		printf("\n");
	}
	while(!gShouldStop)
	{
		if(displayKeys.size() > 0)
		{
			for(unsigned int n = 0; n < displayKeys.size(); ++n)
			{
				// print fixed-point omitting all leading zeros (but not the . )
				// e.g.: 0.0123 becomes " . 123", and 1.0123 stays "1.0123"
				float value = keys.getNoteValue(displayKeys[n]);
				int integer = (int)value;
				int frac = (value - integer) * 10000;
				int len = snprintf(NULL, 0, "%d", integer);
				char str[len + 1];
				if(integer == 0)
					sprintf(str, " ");
				else
					sprintf(str, "%d", integer);
				printf("%s.%4d ", str, frac);
			}
			printf("\n");
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

