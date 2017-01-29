// Standard header files
#include <stdio.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <Gpio.h>
#include <stdlib.h>
#include "Keys.h"

void Board::setKeys(unsigned int firstActiveKey, unsigned int lastActiveKey, unsigned int lowestNote)
{
	_firstActiveKey = firstActiveKey;
	_lastActiveKey = lastActiveKey;
	_lowestNote = lowestNote;
}

int Board::getNote(unsigned int key)
{
	if(key < _firstActiveKey)
		return -1;
	if(key > _lastActiveKey)
		return -2;
	int note = key - _firstActiveKey + _lowestNote;
	return note;
}

void BoardsTopology::setBoard(unsigned int boardPosition, unsigned int firstActiveKey, unsigned int lastActiveKey)
{
	unsigned int size = indexes.size();
	unsigned int requiredSize = (boardPosition + 1) * numIndexesPerBoard;
	if(size < requiredSize){
		indexes.resize(requiredSize);
	}
	unsigned int first = boardPosition * numIndexesPerBoard;
	indexes[boardPosition * numIndexesPerBoard] = firstActiveKey;
	indexes[boardPosition * numIndexesPerBoard + 1] = lastActiveKey;
	updateBoards();
}

void BoardsTopology::updateBoards()
{
	unsigned int numBoards = indexes.size() / numIndexesPerBoard;
	deallocBoards();
	boards.resize(numBoards);
	int lowestNote = _lowestNote;
	for(unsigned int n = 0; n < numBoards; ++n){
		int firstActiveKey = indexes[n * numIndexesPerBoard];
		int lastActiveKey = indexes[n * numIndexesPerBoard + 1];
		Board* board = new Board;
		board->setKeys(firstActiveKey, lastActiveKey, lowestNote);
		boards[n] = board;
		int numNotes = board->getNumActiveKeys();
		lowestNote += numNotes;
	}
}

void BoardsTopology::deallocBoards()
{
	for(auto board : boards)
	{
		delete board;
	}
}

