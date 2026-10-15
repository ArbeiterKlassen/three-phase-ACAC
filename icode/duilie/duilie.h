


#ifndef DUILIE_DUILIE_H_
#define DUILIE_DUILIE_H_
#define maxsize 200


typedef struct SqQueue{
	float data[maxsize];
	int front;//队首指针
	int rear;//队尾指针
}SqQueue;

SqQueue initQueue();
int isEmpty(SqQueue qu);
int enQueue(SqQueue *qu,float x);
int deQueue(SqQueue *qu);
int printQueue(SqQueue qu);
#endif /*DUILIE_H*/
