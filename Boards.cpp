#include "Boards.h"

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

unsigned int Board::getLastActiveKey(){
	return _lastActiveKey;
}

unsigned int Board::getFirstActiveKey(){
	return _firstActiveKey;
}

unsigned int Board::getNumActiveKeys(){
	int ret = _lastActiveKey - _firstActiveKey + 1;
	if(ret < 0)
		return 0;
	else
		return ret;
}

void BoardsTopology::setBoard(unsigned int boardPosition, unsigned int firstActiveKey, unsigned int lastActiveKey)
{
	unsigned int size = indexes.size();
	unsigned int requiredSize = std::max(size, (boardPosition + 1) * numIndexesPerBoard);
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
	int startNote = _lowestNote;
	for(int n = numBoards - 1; n >= 0; --n)
	{
		int firstActiveKey = indexes[n * numIndexesPerBoard];
		int lastActiveKey = indexes[n * numIndexesPerBoard + 1];
		Board* board = new Board;
		board->setKeys(firstActiveKey, lastActiveKey, startNote);
		boards[n] = board;
		int numNotes = board->getNumActiveKeys();
		startNote += numNotes;
	}
	_highestNote = startNote - 1;
}

void BoardsTopology::deallocBoards()
{
	for(auto &board : boards)
	{
		delete board;
	}
}

unsigned int BoardsTopology::getNumNotes()
{
	int numNotes = 0;
	for(auto &board: boards)
	{
		numNotes += board->getNumActiveKeys();
	}
	return numNotes;
}

int BoardsTopology::getNote(unsigned int board, unsigned int key)
{
	return boards[board]->getNote(key);
}

unsigned int BoardsTopology::getNumBoards()
{
	return boards.size();
}

unsigned int BoardsTopology::getFirstActiveKey(unsigned int board)
{
	return indexes[board * numIndexesPerBoard];
}

unsigned int BoardsTopology::getLastActiveKey(unsigned int board)
{
	return indexes[board * numIndexesPerBoard + 1];
}

