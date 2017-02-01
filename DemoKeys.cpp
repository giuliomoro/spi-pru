#include "Keys.h"
#include <unistd.h>
#include <signal.h>
#include <Gpio.h>


volatile int gShouldStop = 0;
void catch_function(int signo){
	gShouldStop = 1;
}

int main()
{
	Gpio testGpio;
	testGpio.open(66, 1);
	Gpio testGpio2;
	testGpio2.open(67, 1);
	Gpio testGpio3;
	testGpio3.open(69, 1);
	signal(SIGINT, catch_function);
	Keys keys;
	BoardsTopology bt;
	// Let's build a topology for just above 5 octaves, A to C
	bt.setLowestNote(33);
	bt.setBoard(0, 0, 24);
	bt.setBoard(1, 0, 23);
	bt.setBoard(2, 9, 23);
	int ret = keys.start(&bt, &gShouldStop);
	if(ret < 0)
	{
		fprintf(stderr, "Error while starting the scan of the keys: %d %s\n", ret, strerror(-ret));
		return 1;
	}
	
	while(!gShouldStop)
	{
		for(int n = bt.getLowestNote(); n < bt.getHighestNote(); ++n)
		printf("%d ", (int)(10 * keys.getNoteValue(n)));
		printf("\n");
		usleep(100000);	
	}
	keys.stop();
	
	return 0;
}

