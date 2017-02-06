#include <assert.h>
#include "BoardsTopology_c.h"
#include <stdio.h>

int main(){
	{
		BoardsTopology* bt;
		unsigned int numBoards = 3;
		unsigned int lowestNote = 33;
		bt = BoardsTopology_new();
		BoardsTopology_setLowestNote(bt, lowestNote);
		unsigned int keyCount = 0;
		unsigned int boardPosition = 0;
		unsigned int firstKey = 0;
		unsigned int lastKey = 24;
		BoardsTopology_setBoard(bt, boardPosition, firstKey, lastKey);
		keyCount += lastKey - firstKey + 1;

		boardPosition = 1;
		firstKey = 0;
		lastKey = 23;
		BoardsTopology_setBoard(bt, boardPosition, firstKey, lastKey);
		keyCount += lastKey - firstKey + 1;

		boardPosition = 2;
		firstKey = 9;
		lastKey = 23;
		BoardsTopology_setBoard(bt, boardPosition, firstKey, lastKey);
		keyCount += lastKey - firstKey + 1;

		int highestNote = lowestNote + keyCount - 1;

		assert(BoardsTopology_getNumBoards(bt) == boardPosition + 1);
		assert(keyCount == BoardsTopology_getNumNotes(bt));
		assert(BoardsTopology_getFirstActiveKey(bt, boardPosition) == firstKey);
		assert(BoardsTopology_getLastActiveKey(bt, boardPosition) == lastKey);
		assert(BoardsTopology_getLowestNoteBoard(bt, BoardsTopology_getNumBoards(bt) - 1) == lowestNote);
		assert(BoardsTopology_getLowestNoteBoard(bt, 0) == 
			lowestNote + keyCount - (BoardsTopology_getLastActiveKey(bt, 0) - BoardsTopology_getFirstActiveKey(bt, 0) + 1));

		unsigned int maxKeysPerBoard = 25;
		int n = lowestNote;
		for(int b = BoardsTopology_getNumBoards(bt) - 1; b >=0;  --b){
			for(unsigned int k = 0; k < maxKeysPerBoard; ++k){
				int note = BoardsTopology_getNote(bt, b, k);
				if(note >= 0){
					assert(note == n);
					++n;
				}
			}
		}
		assert(n - 1 == highestNote);
	}
	return 0;
}

