#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fifo.h"


void fifo_create(fifo_t* temp, int size)
{    
    temp->read_index = 0;
    temp->write_index = 0;
    temp->unread_count = 0;
    temp->max_size = size;
}

int fifo_push(fifo_t* fifo, char val)
{
    if(fifo->unread_count >= fifo->max_size)
        return -1;
    fifo->buff[fifo->write_index] = val;
    (fifo->unread_count)++;
    (fifo->write_index)++;
    if(fifo->write_index >= fifo->max_size)
        fifo->write_index = 0;
    return 0;
}

int fifo_pop(fifo_t* fifo,   char* val)
{
    if(fifo->unread_count == 0)
        return -1;
    *val = fifo->buff[(fifo->read_index)];
    (fifo->read_index)++;

    if(fifo->read_index >= fifo->max_size)
        fifo->read_index = 0;

    (fifo->unread_count)--;
//    if(fifo->read_index >= fifo->max_size)

    return 0;
}

void fifo_flush(fifo_t* fifo)
{
  fifo->unread_count = 0;
  fifo->read_index = fifo->write_index;
 // fifo->write_index = 0;
}

