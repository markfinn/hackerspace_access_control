#ifndef MRBUS_QUEUE_H
#define MRBUS_QUEUE_H

#include "mrbus-constants.h"

typedef struct 
{
	uint8_t pkt[MRBUS_BUFFER_SIZE];
} MRBusPacket;

typedef struct
{
	volatile uint8_t headIdx;
	volatile uint8_t tailIdx;
	volatile uint8_t full;
	MRBusPacket* pktBufferArray;
	uint8_t pktBufferArraySz;
} MRBusPktQueue;

void mrbusPktQueueInitialize(MRBusPktQueue* q, MRBusPacket* pktBufferArray, uint8_t pktBufferArraySz);
uint8_t mrbusPktQueueDepth(MRBusPktQueue* q);
uint8_t mrbusPktQueuePush(MRBusPktQueue* q, uint8_t* data, uint8_t dataLen);
uint8_t mrbusPktQueuePopInternal(MRBusPktQueue* q, uint8_t* data, uint8_t dataLen, uint8_t snoop);
uint8_t mrbusPktQueueDrop(MRBusPktQueue* q);

#define mrbusPktQueueFull(q) ((q)->full?1:0)
#define mrbusPktQueueEmpty(q) (0 == mrbusPktQueueDepth(q))

#define mrbusPktQueuePeek(q, data, dataLen) mrbusPktQueuePopInternal((q), (data), (dataLen), 1)
#define mrbusPktQueuePop(q, data, dataLen) mrbusPktQueuePopInternal((q), (data), (dataLen), 0)


#endif
