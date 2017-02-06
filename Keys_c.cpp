#include "Keys_c.h"
#include "Keys.h"

Keys* Keys_new()
{
	return new Keys;
}

void Keys_delete(Keys* that)
{
	delete that;
}

int Keys_start(Keys* that, BoardsTopology* bt, volatile int* shouldStop)
{
	return that->start(bt, shouldStop);
}

void Keys_stop(Keys* that)
{
	that->stop();
}

float Keys_getNoteValue(Keys* that, int note)
{
	return that->getNoteValue(note);
}

