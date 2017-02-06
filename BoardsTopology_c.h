#ifndef BOARDSTOPOLOGY_C_H
#define BOARDSTOPOLOGY_C_H
#ifdef __cplusplus
#include "Boards.h"
extern "C" {
#else  /* if in C, we should define the types */
typedef void* BoardsTopology;
#endif

BoardsTopology* BoardsTopology_new(void);

void BoardsTopology_delete(BoardsTopology* that);

void BoardsTopology_setBoard(BoardsTopology* that, unsigned int boardPosition, unsigned int firstActiveKey, unsigned int lastActiveKey);

void BoardsTopology_setLowestNote(BoardsTopology* that, unsigned int lowestNote);
int BoardsTopology_getNote(BoardsTopology* that, unsigned int board, unsigned int key);
unsigned int BoardsTopology_getFirstActiveKey(BoardsTopology* that, unsigned int board);
unsigned int BoardsTopology_getLastActiveKey(BoardsTopology* that, unsigned int board);
int BoardsTopology_getLowestNote(BoardsTopology* that);
int BoardsTopology_getLowestNoteBoard(BoardsTopology* that, unsigned int board);
int BoardsTopology_getHighestNote(BoardsTopology* that);
int BoardsTopology_getHighestNoteBoard(BoardsTopology* that, unsigned int board);
int BoardsTopology_getNumBoards(BoardsTopology* that);
int BoardsTopology_getNumNotes(BoardsTopology* that);

#ifdef __cplusplus
}
#endif
#endif /* BOARDSTOPOLOGY_C_H */
