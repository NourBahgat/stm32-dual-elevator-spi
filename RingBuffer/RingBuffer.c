/**
 * RingBuffer.cpp
 *
 * Created on: 2026-04-25
 * Author    : AbdallahDarwish
 */
#include "RingBuffer.h"

void RB_Init(RingBufferType* rb)
{
    rb->writeIndex = 0;
    rb->readIndex = 0;
    rb->count = 0;
}

// 
boolean RB_Enqueue(RingBufferType* rb, uint8 byte)
{   // check if buffer is full before enqueuing
    if (RB_IsFull(rb))
    {
        return FALSE; // buffer full, byte discarded
    }
    rb->buffer[rb->writeIndex] = byte;
    // advance write index with wrap-around
    rb->writeIndex = (rb->writeIndex + 1) % RB_SIZE;
    rb->count++;
    return TRUE;
}

boolean RB_Dequeue(RingBufferType* rb, uint8* byte)
{
    if (RB_IsEmpty(rb))
    {
        return FALSE;
    }
    *byte = rb->buffer[rb->readIndex];
    rb->readIndex = (rb->readIndex + 1) % RB_SIZE;
    rb->count--;
    return TRUE;
}

// read the next byte without removing it from the buffer
// we use it in Usart1_ReadLine to check for line terminators without consuming bytes until we're ready
uint8 RB_Peek(const RingBufferType* rb)
{
    return rb->buffer[rb->readIndex];
}

boolean RB_IsEmpty(const RingBufferType* rb)
{
    return (rb->count == 0);
}

boolean RB_IsFull(const RingBufferType* rb)
{
    return (rb->count == RB_SIZE);
}

uint32 RB_Available(const RingBufferType* rb)
{
    return rb->count;
}

void RB_Flush(RingBufferType* rb)
{
    rb->writeIndex = 0;
    rb->readIndex = 0;
    rb->count = 0;
}
