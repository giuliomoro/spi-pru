#include <assert.h>
#include "Keys.h"
#include <stdio.h>

int main(){
	{
		Board board;
		int firstActiveKey = 10;
		int lastActiveKey = 24;
		int lowestNote = 60;
		board.setKeys(firstActiveKey, lastActiveKey, lowestNote);
		int note;
		int key;

		key = 0;
		note = board.getNote(key);
		assert(note < 0 );

		key = firstActiveKey;
		note = board.getNote(key);
		assert(note == lowestNote);

		key = lastActiveKey;
		note = board.getNote(key);
		assert(note == lowestNote + lastActiveKey - firstActiveKey);

		key = lastActiveKey;
		note = board.getNote(key);
		assert(note == lowestNote + key - firstActiveKey);

		int offset = 10;
		key = firstActiveKey + offset;
		note = board.getNote(key);
		assert(note == lowestNote + offset);
	}
	{
		BoardsTopology bt;
		unsigned int numBoards = 3;
		unsigned int lowestNote = 33;
		bt.setLowestNote(lowestNote);
		unsigned int keyCount = 0;
		unsigned int boardPosition = 0;
		unsigned int firstKey = 0;
		unsigned int lastKey = 23;
		bt.setBoard(boardPosition, firstKey, lastKey);
		keyCount += lastKey - firstKey;
		boardPosition = 1;
		bt.setBoard(boardPosition, firstKey, lastKey);
		keyCount += lastKey - firstKey;
		boardPosition = 2;
		lastKey = 24;
		bt.setBoard(boardPosition, firstKey, lastKey);
		keyCount += lastKey - firstKey;

		unsigned int maxKeysPerBoard = 25;
		int n = lowestNote;
		for(unsigned int b = 0; b < bt.getNumBoards(); ++b){
			for(unsigned int k = 0; k < maxKeysPerBoard; ++k){
				int note = bt.getNote(b, k);
				if(note >= 0){
					assert(note == n);
					++n;
				}
			}
		}
	}

	return 0;
}
