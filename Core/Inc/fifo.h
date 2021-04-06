#ifndef FIFO_H_INCLUDED
#define FIFO_H_INCLUDED



typedef struct fifo{
    char buff[1000];
    int write_index;
    int read_index;
    size_t unread_count;
    unsigned int max_size;
}fifo_t;


void fifo_create(fifo_t* temp, int size);
int fifo_push(fifo_t* fifo, char val);
int fifo_pop(fifo_t* fifo, char* val);
void fifo_flush(fifo_t* fifo);

#endif // FIFO_H_INCLUDED
