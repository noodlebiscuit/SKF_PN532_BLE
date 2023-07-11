#define SerialBuffer_h

#include <Arduino.h>

template <size_t N>
class SerialBuffer
{
private:
    uint8_t ringBuffer[N];
    size_t index = 0;

public:
    /// @brief adds a new byte value to the ring buffer
    /// @param value value to add
    void add(uint8_t value)
    {
        ringBuffer[index++] = value;
    }

    /// @brief pops the oldest value off the ring buffer
    /// @return buffer value
    int pop()
    {
        if (index < 0)
        {
            return -1;
        }
        return ringBuffer[0];
    }

    /// @brief clear ring buffer contents
    void clear()
    {
        memset(ringBuffer, 0, N);
        index = 0;
    }

    /// @brief this.get(0) is the oldest value, this.get(this.getLength() - 1) is the newest value
    /// @param index
    /// @return value as integer
    uint8_t get(size_t position)
    {
        return ringBuffer[min(position, index)];
    }

    /// @brief get the entire buffer
    /// @return value as integer
    uint8_t *get()
    {
        return ringBuffer;
    }

    /// @brief returns the number of bytes written to the buffer
    /// @return value as integer
    size_t getLength()
    {
        return index;
    }

    /// @brief returns the total number of bytes in the buffer
    /// @return value as integer
    size_t getSize()
    {
        return N;
    }
};