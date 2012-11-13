#include <stdlib.h>
#include "ringbuffer.h"

template <typename T>
Ringbuffer<T>::Ringbuffer(unsigned int n)
{
    alloc_size = n;
    buffer = new T[n];
    tail = alloc_size;
    length = 0;
}

template <typename T>
Ringbuffer<T>::~Ringbuffer()
{
    delete[] buffer;
}

template <typename T>
T& Ringbuffer<T>::operator[](unsigned int i)
{
    return buffer[(tail+alloc_size) % alloc_size];
}

template <typename T>
T& Ringbuffer<T>::at_front(unsigned int i)
{
    if (i >= _length) abort();
    else return (*this)[i];
}

template <typename T>
void Ringbuffer<T>::push(T& x)
{
    tail = (tail+1) % alloc_size;
    buffer[tail] = x;
}

template <typename T>
unsigned int Ringbuffer<T>::length()
{
    return _length;
}
