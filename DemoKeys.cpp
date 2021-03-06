#include <unistd.h>
#include <signal.h>
#include <Gpio.h>
#include <WriteFile.h>
#include <xenomai/init.h>
#include <vector>
#define KEYS_C
#define SCOPE

#ifdef SCOPE
#include <Scope.h>
Scope gScope;
#endif /* SCOPE */

#ifdef KEYS_C
#include "Keys_c.h"
#else /* KEYS_C */
#include "Keys.h"
#endif /* KEYS_C */

std::vector<int> displayKeys;
int gXenomaiInited = 0; // required by WriteFile's AuxiliaryTask
unsigned int gAuxiliaryTaskStackSize  = 1 << 17; // required by WriteFile's AuxiliaryTask
volatile int gShouldStop = 0;

void catch_function(int signo){
	gShouldStop = 1;
}

BoardsTopology bt;
WriteFile* file;

bool isWhiteKey(int key)
{
	key = (key % 12);
	if(key == 1 || key == 3 || key == 6 || key == 8 || key == 10)
		return false;
	return true;
}
void postCallback(void* arg, float* buffer, unsigned int length)
{
	Keys* keys = (Keys*)arg;
	unsigned int numKeys = bt.getHighestNote() - bt.getLowestNote() + 1;
	float values[numKeys];
	float minValue = 1;
	int minKey = -1;
	for(int n = bt.getLowestNote(); n <= bt.getHighestNote(); ++n)
	{
#ifdef KEYS_C
		float value = Keys_getNoteValue(keys, n);
#else /* KEYS_C */
		float value = keys->getNoteValue(n);
#endif /* KEYS_C */
#ifdef SCOPE
		values[n-bt.getLowestNote()] = value;
		if(value != 0 && value < minValue) // the != 0 is because unconnected boards will read exactly 0, so we rule that out
		{
			minKey = n;
			minValue = value;
		}
#endif /* SCOPE */
	}
	if(file)
		file->log(values, numKeys);
#ifdef SCOPE
#if 0
	static int currentKey = -1;
	if(minValue < 0.3 & minKey >= 0)
	{
		currentKey = minKey;
	}
	if(currentKey >= 0)
	{
		for(int n = 0; n < numKeys; ++n)
		{
			if(n != currentKey)
			{
				//values[n] = 0;
			}
		}
	}
#endif
	float logs[displayKeys.size()];
	for(int n = 0; n < displayKeys.size(); ++n)
	{
		logs[n] = keys->getNoteValue(displayKeys[n]) - 0.1;
	}

	gScope.log(logs);
#endif /* SCOPE */
}

void usage()
{
	printf(
			"single <note> : only display high-precision value for the given key\n"
			"range <note> <note> : only display high-precision values for the given keys\n"
			"white :       only show high-precision values for white keys, and thresholded \"top\" position for black ones.\n"
			"black :       only show high-precision values for black keys, and thresholded \"bottom\" position for white ones.\n"
			"raw         : log raw values as they come in from the sensor to rawsensors.bin\n"
			"in <path>   : load inverse square calibration file from <path> and display calibrated values \n"
			"lin <path>  : load linear calibration file from <path> and display calibrated values \n"
			"out <path>  : generates file  <path> with key top and key bottom values. \n"
			"              make sure no key is been pressed. Start, wait for key values to be displayed\n"
                        "              and then start pressing all keys. Hit ctrl-C when done. Once done, will display\n"
			"              normalized readings.\n"
			);
}
Keys* keys;

int main(int argc, char** argv)
{
	const int noinout = 0;
	const int in = 1;
	const int out = 2;
	const int lin = 3;
	bool whiteHp = true;
	bool blackHp = true;
	bool lograw = false;
	int inout = noinout;
	int highPrecision = 0;
	char* path = NULL;
	while(++argv, --argc)
	{
		printf("arg: %s\n", *argv);
		if(strcmp(*argv, "--help") == 0 || strcmp(*argv, "help") == 0)
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
			highPrecision = 1;
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
		if(strcmp(*argv, "white") == 0)
		{
			blackHp = false;
			continue;
		}
		if(strcmp(*argv, "black") == 0)
		{
			whiteHp = false;
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
		if(strcmp(*argv, "lin") == 0)
			inout = lin;
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
		file = new WriteFile;
		file->init("rawsensors.bin"); //set the file name to write to
		file->setFileType(kBinary);
	}
	signal(SIGINT, catch_function);
#ifdef KEYS_C
	keys = Keys_new();
#else /* KEYS_C */
	keys = new Keys;
#endif /* KEYS_C */

	// Let's build a topology for 6 octaves, C to C
	bt.setLowestNote(0);
	bt.setBoard(0, 0, 24);
	bt.setBoard(1, 0, 23);
	bt.setBoard(2, 0, 23);
	if(displayKeys.size() == 0)
	{
		// default: all the keys from getLowestNote to getHighestNote
		unsigned int start = bt.getLowestNote();
		displayKeys.resize(bt.getHighestNote() - start + 1);
		for(unsigned int n = 0; n < displayKeys.size(); ++n)
		{
			displayKeys[n] = n + start;
		}
	}
#ifdef SCOPE
	gScope.setup(displayKeys.size(), 1000);
#endif /* SCOPE */
	if(lograw
#ifdef SCOPE
		|| 1
#endif /* SCOPE */
	)
	{
#ifdef KEYS_C
		Keys_setPostCallback(keys, postCallback, keys);
#else /* KEYS_C */
		keys->setPostCallback(postCallback, keys);
#endif /* KEYS_C */
	}
#ifdef KEYS_C
	int ret = Keys_start(keys, &bt, NULL);
#else /* KEYS_C */
	int ret = keys->start(&bt, NULL);
#endif /* KEYS_C */
	if(ret < 0)
	{
		fprintf(stderr, "Error while starting the scan of the keys: %d %s\n", ret, strerror(-ret));
		return 1;
	}

	if(inout == out)
	{
#ifdef KEYS_C
		Keys_startTopCalibration(keys);
#else /* KEYS_C */
		keys->startTopCalibration();
#endif /* KEYS_C */
		printf("Top calibration...");
		fflush(stdout);
		while(!gShouldStop && !
#ifdef KEYS_C
				Keys_isTopCalibrationDone(keys)
#else /* KEYS_C */
				keys->isTopCalibrationDone()
#endif /* KEYS_C */
		     )
		{
			for(int n = 0; n < displayKeys.size(); ++n)
			{
				unsigned int idx = displayKeys[n];
				printf("%2d ", (int)(100 * 
#ifdef KEYS_C
					Keys_getNoteValue(keys, idx)
#else /* KEYS_C */
					keys->getNoteValue(idx)
#endif /* KEYS_C */
				));
			}
			printf("\n");
			usleep(10000);
		}
		gShouldStop = 0;
		printf("done\n");
#ifdef KEYS_C
		Keys_dumpTopCalibration(keys);
#else /* KEYS_C */
		keys->dumpTopCalibration();
#endif /* KEYS_C */
	
		printf("Bottom calibration...");
		fflush(stdout);
#ifdef KEYS_C
		Keys_startBottomCalibration(keys);
#else /* KEYS_C */
		keys->startBottomCalibration();
#endif /* KEYS_C */
		while(!gShouldStop)
		{
			printf("bot calib ");
			for(unsigned int n = 0; n < displayKeys.size(); ++n)
			{
				unsigned int idx = displayKeys[n];
				printf("%2d ", (int)(100 * 
#ifdef KEYS_C
					Keys_getNoteValue(keys, idx)
#else /* KEYS_C */
					keys->getNoteValue(idx)
#endif /* KEYS_C */
					));
			}
			printf("\n");
			usleep(100000);	
		}
		gShouldStop = 0;
		printf("done\n");
#ifdef KEYS_C
		Keys_stopBottomCalibration(keys);
		Keys_dumpBottomCalibration(keys);
		Keys_saveLinearCalibrationFile(keys, path);
#else /* KEYS_C */
		keys->stopBottomCalibration();
		keys->dumpBottomCalibration();
		keys->saveLinearCalibrationFile(path);
#endif /* KEYS_C */
	}
	if(inout == in || inout == lin)
	{
		printf("Loading %s calibration file %s\n", inout == in ? "InverseSquare" : "Linear", path);
		if(inout == in)
		{
#ifdef KEYS_C
			Keys_startTopCalibration(keys);
			Keys_loadInverseSquareCalibrationFile(keys, path, 0);
#else /* KEYS_C */
			keys->startTopCalibration();
			keys->loadInverseSquareCalibrationFile(path, 0);
#endif /* KEYS_C */
		}
		if(inout == lin)
		{
#ifdef KEYS_C
			Keys_startTopCalibration(keys);
			Keys_loadLinearCalibrationFile(keys, path);
#else /* KEYS_C */
			keys->startTopCalibration();
			keys->loadLinearCalibrationFile(path);
#endif /* KEYS_C */
		}
	}
	if(inout == noinout)
	{
#ifdef KEYS_C
		Keys_useCalibration(keys, false);
#else /* KEYS_C */
		keys->useCalibration(false);
#endif /* KEYS_C */
	} else {
#ifdef KEYS_C
		Keys_useCalibration(keys, true);
#else /* KEYS_C */
		keys->useCalibration(true);
#endif /* KEYS_C */
	}
	for(unsigned int n = 0; n < displayKeys.size(); ++n)
	{
		printf("%5d |", displayKeys[n]);
	}
	printf("\n");
	if(!highPrecision)
	{
#ifdef KEYS_C
		Keys_setHardClip(keys, true);
#else /* KEYS_C */
		keys->setHardClip(true);
#endif /* KEYS_C */
	}

	while(!gShouldStop)
	{
		for(unsigned int n = 0; n < displayKeys.size(); ++n)
		{
			// print fixed-point omitting all leading zeros (but not the . )
			// e.g.: 0.0123 becomes " . 123", and 1.0123 stays "1.0123"
#ifdef KEYS_C
			float value = Keys_getNoteValue(keys, displayKeys[n]);
#else /* KEYS_C */
			float value = keys->getNoteValue(displayKeys[n]);
#endif /* KEYS_C */
			// Optionally, only show high-precision for white or black
			bool isWhite = isWhiteKey(displayKeys[n]);
			if( !((isWhite && whiteHp) || (!isWhite && blackHp)) )
			{
				// and if it's not high precision, threshold:
				if(isWhite) // is white far from bottom?
					value = value > 0.05;
				else // is black far from top?
					value = value > 0.999;
			}
			{
			if(highPrecision)
			{
				int integer = (int)value;
				int frac = (value - integer) * 10000;
				int len = snprintf(NULL, 0, "%d", integer);
				char str[len + 1];
				if(integer == 0)
					sprintf(str, " ");
				else
					sprintf(str, "%d", integer);
				if(integer)
					printf("%s.%04d ", str, frac);
				else
					printf("%s.%4d ", str, frac);
			} else {
				printf("%X ", (int)(10 * value));
			}
			}
		}
		printf("\n");
		usleep(30000);
	}
#ifdef KEYS_C
	Keys_stopAndWait(keys);
	Keys_delete(keys);
#else /* KEYS_C */
	keys->stopAndWait();
	delete keys;
#endif /* KEYS_C */
	delete file;
	
	return 0;
}

