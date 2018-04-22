#include "Boards.h"

void Board::setKeys(unsigned int firstActiveKey, unsigned int lastActiveKey, unsigned int lowestNote)
{
	_firstActiveKey = firstActiveKey;
	_lastActiveKey = lastActiveKey;
	_lowestNote = lowestNote;
}

int Board::getLowestNote()
{
	return _lowestNote;
}

int Board::getHighestNote()
{
	return _lowestNote + _lastActiveKey - _firstActiveKey;
}

int Board::getKey(unsigned int note)
{
	if(note < _lowestNote)
		return -1;
	if(note > (unsigned int)getHighestNote())
		return -2;
	return note - getLowestNote();
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

int BoardsTopology::findBoardFromNote(unsigned int note)
{
	int board = -1;
	for(unsigned int n = 0; n < getNumBoards(); ++n)
	{
		int key = boards[n]->getKey(note);
		if(key >= 0) // key belongs to this board
		{
			board = n;
			break;
		}
	}
	return board;
}

int BoardsTopology::findKeyFromNote(unsigned int note)
{
	int board = findBoardFromNote(note);
	if(board < 0)
		return -1;
	return boards[board]->getKey(note);
}
