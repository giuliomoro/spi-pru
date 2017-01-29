#include <vector>
#include <stdio.h>

class Board
{
public:
	Board() {};

	~Board() {};
	
	void setKeys(unsigned int firstActiveKey, unsigned int lastActiveKey, unsigned int lowestNote);

	int getNote(unsigned int key);

	unsigned int getLastActiveKey(){
		return _lastActiveKey;
	}

	unsigned int getFirstActiveKey(){
		return _firstActiveKey;
	}

	unsigned int getNumActiveKeys(){
		int ret = _lastActiveKey - _firstActiveKey + 1;
		if(ret < 0)
			return 0;
		else
			return ret;
	}

private:
	unsigned int _firstActiveKey;
	unsigned int _lastActiveKey;
	unsigned int _lowestNote;
};

class BoardsTopology
{
public:
	BoardsTopology():
		_lowestNote(0)
	{};

	~BoardsTopology()
	{
		deallocBoards();
	};

	void setLowestNote(unsigned int lowestNote){
		_lowestNote = lowestNote;
	}
	void setBoard(unsigned int boardPosition, unsigned int firstActiveKey, unsigned int lastActiveKey);

	void deallocBoard(unsigned int boardPosition);

	int getNote(unsigned int board, unsigned int key){
		return boards[board]->getNote(key);
	}

	unsigned int getNumBoards(){
		return boards.size();
	}
	
private:
	void deallocBoards();
	void updateBoards();
	void resizeBoards(unsigned int numBoards);
	unsigned int _lowestNote;
	std::vector<Board*> boards;
	std::vector<unsigned int> indexes;
	static const unsigned int numIndexesPerBoard = 2;
};

class Keys{
public:
	Keys(){};

	~Keys(){};
	
	int init(){
		
	}

private:
	int loadPru();
	bool active;
};

