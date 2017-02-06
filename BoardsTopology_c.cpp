#include "BoardsTopology_c.h"

BoardsTopology* BoardsTopology_new()
{
	return new BoardsTopology;
}

void BoardsTopology_delete(BoardsTopology* that)
{
	delete that;
}

void BoardsTopology_setBoard(BoardsTopology* that, unsigned int boardPosition, unsigned int firstActiveKey, unsigned int lastActiveKey)
{
	that->setBoard(boardPosition, firstActiveKey, lastActiveKey);
}

void BoardsTopology_setLowestNote(BoardsTopology* that, unsigned int lowestNote)
{
	that->setLowestNote(lowestNote);
}

int BoardsTopology_getNote(BoardsTopology* that, unsigned int board, unsigned int key)
{
	return that->getNote(board, key);
}

unsigned int BoardsTopology_getFirstActiveKey(BoardsTopology* that, unsigned int board)
{
	return that->getFirstActiveKey(board);
}

unsigned int BoardsTopology_getLastActiveKey(BoardsTopology* that, unsigned int board)
{
	return that->getLastActiveKey(board);
}

int BoardsTopology_getLowestNote(BoardsTopology* that)
{
	return that->getLowestNote();
}

int BoardsTopology_getLowestNoteBoard(BoardsTopology* that, unsigned int board)
{
	return that->getLowestNote(board);
}

int BoardsTopology_getHighestNote(BoardsTopology* that)
{
	return that->getHighestNote();
}

int BoardsTopology_getHighestNoteBoard(BoardsTopology* that, unsigned int board)
{
	return that->getHighestNote(board);
}

int BoardsTopology_getNumBoards(BoardsTopology* that)
{
	return that->getNumBoards();
}

int BoardsTopology_getNumNotes(BoardsTopology* that)
{
	return that->getNumNotes();
}
