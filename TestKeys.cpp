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
		unsigned int lastKey = 24;
		bt.setBoard(boardPosition, firstKey, lastKey);
		keyCount += lastKey - firstKey + 1;

		boardPosition = 1;
		firstKey = 0;
		lastKey = 23;
		bt.setBoard(boardPosition, firstKey, lastKey);
		keyCount += lastKey - firstKey + 1;

		boardPosition = 2;
		firstKey = 9;
		lastKey = 23;
		bt.setBoard(boardPosition, firstKey, lastKey);
		keyCount += lastKey - firstKey + 1;

		int highestNote = lowestNote + keyCount - 1;

		assert(bt.getNumBoards() == boardPosition + 1);
		assert(keyCount == bt.getNumNotes());
		assert(bt.getFirstActiveKey(boardPosition) == firstKey);
		assert(bt.getLastActiveKey(boardPosition) == lastKey);
		assert(bt.getLowestNote(bt.getNumBoards() - 1) == lowestNote);
		assert(bt.getLowestNote(0) == 
			lowestNote + keyCount - (bt.getLastActiveKey(0) - bt.getFirstActiveKey(0) + 1));

		unsigned int maxKeysPerBoard = 25;
		int n = lowestNote;
		for(int b = bt.getNumBoards() - 1; b >=0;  --b){
			for(unsigned int k = 0; k < maxKeysPerBoard; ++k){
				int note = bt.getNote(b, k);
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
