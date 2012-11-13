#pragma once

template <typename T>
class Ringbuffer {
    T *buffer;
    unsigned int tail;
    unsigned int alloc_size, _length;

    public:
    Ringbuffer(unsigned int n);
    ~Ringbuffer();
    Ringbuffer(const Ringbuffer<T>&) = delete;
    Ringbuffer<T>& operator=(const Ringbuffer<T>&) = delete;

    T& at_front(unsigned int i);
    T& operator[](unsigned int i);
    void push(T& t);
    unsigned int length();
};

