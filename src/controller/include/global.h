
#ifndef GLOBAL
#define GLOBAL

extern TreeItem * itemPaused ;
extern TreeItem * parentItemPaused;
extern bool paused;
extern bool expandedText;
extern bool cancelled;
extern int visualState;
extern int lastVisualState;
extern bool abortingWait;
extern ExecutionTree * lastExecutionTree;
extern bool doneFirst;
extern bool processing_belief_query;//we use this attribute tu assure that the query has done their job
#endif
