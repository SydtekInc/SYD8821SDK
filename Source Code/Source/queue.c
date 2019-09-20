#include "queue.h"
#include "string.h"

void queue_init(QUEUE * queue, uint8_t *buf, int size)
{
    queue->head = queue->tail = 0;
    queue->size = size;
    queue->data = buf;
}

void queue_reset(QUEUE * queue)
{
    queue->head = queue->tail;
}


int enqueue_all(QUEUE * queue, uint8_t *data,uint16_t size)
{
	if((queue->tail+size)  >= queue->size)
	{
		uint16_t len=size-queue->tail;
		memcpy(&queue->data[queue->tail],data,len);
		size -=len;
		if(size>=queue->head) size=queue->head-1;
		memcpy(&queue->data[0],data+len,size);
		queue->tail=size;
		
		return len+size;
	}
	else
	{
		memcpy(&queue->data[queue->tail],data,size);
		queue->tail +=size;
		
		return size;
	}
    
}


int enqueue(QUEUE * queue, uint8_t data)
{
    queue->data[queue->tail++] = data;
    if (queue->tail >= queue->size)
        queue->tail = 0;
    return 1;
}

int dequeue(QUEUE * queue, uint8_t * data)
{
    if (queue->head == queue->tail)
        return 0;
    
    *data = queue->data[queue->head++];
    if (queue->head >= queue->size)
        queue->head = 0;
    
    return 1;
}

int is_queue_empty(QUEUE * queue)
{
    return queue->head == queue->tail;
}

int queue_size(QUEUE * queue)
{
    int extra = 0;
    if (queue->head > queue->tail) {
        extra = queue->size;
    }
    return (queue->tail + extra - queue->head);
}
