#include "Keys_c.h"
#include "Keys.h"

Keys* keys_new()
{
	return new Keys;
}

void keys_delete(Keys* that)
{
	delete that;
}

int keys_start(Keys* that, BoardsTopology* bt, volatile int* shouldStop)
{
	return that->start(bt, shouldStop);
}

void keys_stop(Keys* that)
{
	that->stop();
}

float keys_getNoteValue(Keys* that, int note)
{
	return that->getNoteValue(note);
}

