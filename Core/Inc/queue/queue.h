/*
 * queue.h
 *
 *  Created on: Dec 26, 2022
 *      Author: odemki
 */

#ifndef INC_QUEUE_H_
#define INC_QUEUE_H_

#include "stdint.h"

#define QUEUE_SIZE 100
#define NEXT(index) ((index+1)%QUEUE_SIZE)

typedef struct Queue
{
	uint8_t buf[QUEUE_SIZE];
	uint16_t front;
	uint16_t rear;
}Queue;

void InitQueue(Queue *queue);
int IsFull(Queue *queue);
int IsEmpty(Queue* queue);
void Enqueue(Queue *queue, uint8_t data);
uint8_t Dequeue(Queue *queue);
void print_queue(Queue *queue);
uint16_t len_queue(Queue *queue);



#endif /* INC_QUEUE_H_ */
