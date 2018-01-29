#include "Keys.h"
#include <unistd.h>
#include <signal.h>
#include <Gpio.h>


volatile int gShouldStop = 0;
void catch_function(int signo){
	gShouldStop = 1;
}

int main(int argc, char** argv)
{
	const int noinout = 0;
	const int in = 1;
	const int out = 2;
	int inout = noinout;
	char* path;
	while(++argv, --argc)
	{
		printf("arg: %s\n", *argv);
		if(strcmp(*argv, "in") == 0)
			inout = in;
		if(strcmp(*argv, "out") == 0)
			inout = out;
		if(inout != noinout);
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
	//Gpio testGpio;
	//testGpio.open(66, 1);
	//Gpio testGpio2;
	//testGpio2.open(67, 1);
	//Gpio testGpio3;
	//testGpio3.open(69, 1);
	signal(SIGINT, catch_function);
	Keys keys;
	keys.setDebug(true);
	BoardsTopology bt;
	// Let's build a topology for just above 5 octaves, A to C
	bt.setLowestNote(33);
	bt.setBoard(0, 0, 24);
	bt.setBoard(1, 0, 23);
	bt.setBoard(2, 0, 23);
	//bt.setBoard(0, 0, 23);
	//bt.setBoard(1, 0, 23);
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

