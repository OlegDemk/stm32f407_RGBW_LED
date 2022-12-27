/*
 * queue.c
 *
 *  Created on: Dec 26, 2022
 *      Author: odemki
 */


#include "main.h"
#include "queue/queue.h"


// -------------------------------------------------------------------------------------
void InitQueue(Queue *queue)
{
	queue->front = queue->rear = 0;
}
// -------------------------------------------------------------------------------------
int IsFull(Queue *queue)
{
	return NEXT(queue->rear) == queue->front;
}
// -------------------------------------------------------------------------------------
int IsEmpty(Queue *queue)
{
	return queue->front == queue -> rear;
}
// -------------------------------------------------------------------------------------
void Enqueue(Queue *queue, uint8_t data)
{
	uint8_t dummy;

	if(IsFull(queue))
	{
		dummy = Dequeue(queue);
	}
	queue->buf[queue->rear] = data;
	queue->rear = NEXT(queue->rear);
}
// -------------------------------------------------------------------------------------
uint8_t Dequeue(Queue *queue)
{
	uint8_t re = 0;
	if(IsEmpty(queue))
	{
		return re;
	}
	re = queue->buf[queue->front];
	queue->front = NEXT(queue->front);
	return re;
}
// -------------------------------------------------------------------------------------
uint16_t len_queue(Queue *queue)
{
	return ((QUEUE_SIZE - queue->front + queue->rear)%QUEUE_SIZE);
}
// -------------------------------------------------------------------------------------
void print_queue(Queue *queue)
{
	uint16_t i;
	printf("f:%1d, r:%1d" , queue->front, queue->rear);
	for(i = queue->front; i != queue->rear; i = ((i+1)%QUEUE_SIZE));
	{
		printf("[%d]%2d :", i, queue->buf[i]);
	}
	printf("L[%2d]\n\r", len_queue(queue));
}
// -------------------------------------------------------------------------------------
