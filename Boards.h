#ifndef BOARDS_C_H
#define BOARDS_C_H
#include <vector>

class Board
{
public:
	Board() {};

	~Board() {};
	
	void setKeys(unsigned int firstActiveKey, unsigned int lastActiveKey, unsigned int lowestNote);

	int getKey(unsigned int note);

	int getLowestNote();
	int getHighestNote();
	int getNote(unsigned int key);

	unsigned int getLastActiveKey();

	unsigned int getFirstActiveKey();

	unsigned int getNumActiveKeys();

private:
	unsigned int _firstActiveKey;
	unsigned int _lastActiveKey;
	unsigned int _lowestNote;
};

class BoardsTopology
{
public:
	BoardsTopology():
		_lowestNote(-1)
		, _highestNote(-1)
	{};

	~BoardsTopology()
	{
		deallocBoards();
	}

	void setLowestNote(unsigned int lowestNote)
	{
		_lowestNote = lowestNote;
		updateBoards();
	}

	void setBoard(unsigned int boardPosition, unsigned int firstActiveKey, unsigned int lastActiveKey);

	int getNote(unsigned int board, unsigned int key);

	unsigned int getNumBoards();

	unsigned int getNumNotes();

	unsigned int getFirstActiveKey(unsigned int board);
	
	unsigned int getLastActiveKey(unsigned int board);

	int getLowestNote()
	{
		return _lowestNote;
	}

	int getHighestNote()
	{
		return _highestNote;
	}

	int getLowestNote(unsigned int board)
	{
		return boards[board]->getNote(getFirstActiveKey(board));
	}

	int getHighestNote(unsigned int board)
	{
		return boards[board]->getNote(getLastActiveKey(board));
	}

	int findBoardFromNote(unsigned int note);
	int findKeyFromNote(unsigned int note);
private:
	void deallocBoard(unsigned int boardPosition);
	void deallocBoards();
	void updateBoards();
	void resizeBoards(unsigned int numBoards);

	unsigned int _lowestNote;
	unsigned int _highestNote;
	std::vector<Board*> boards;
	std::vector<unsigned int> indexes;
	static const unsigned int numIndexesPerBoard = 2;
};

#endif /* BOARD_C_H */
