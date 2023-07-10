#define CyclicByteBuffer_h

#include <Arduino.h>

template <size_t N>
class CyclicByteBuffer
{
private:
    uint8_t ringBuffer[N];
    size_t newestIndex = 0;
    size_t length = 0;

public:
    /// @brief adds a new byte value to the ring buffer
    /// @param value value to add
    void add(uint8_t value)
    {
        ringBuffer[newestIndex] = value;
        newestIndex = (newestIndex + 1) % N;
        length = min(length + 1, N);
    }

    /// @brief pops the oldest value off the ring buffer
    /// @return buffer value
    int pop()
    {
        if (length == 0)
        {
            return -1;
        }
        uint8_t result = ringBuffer[(N + newestIndex - length) % N];
        length -= 1;
        return result;
    }

    /// @brief clear ring buffer contents
    void clear()
    {
        for (int i=0; i<length; i++)
        {
            ringBuffer[i] = 0x00;
        }
        newestIndex = 0;
        length = 0;
    }

    /// @brief this.get(0) is the oldest value, this.get(this.getLength() - 1) is the newest value
    /// @param index
    /// @return value as integer
    uint8_t get(size_t index)
    {
        return ringBuffer[(N + newestIndex - length + index) % N];
    }

    /// @brief get the entire buffer
    /// @return value as integer
    uint8_t* get()
    {
        return ringBuffer;
    }

    /// @brief returns buffer length
    /// @return value as integer
    size_t getLength()
    {
        return length;
    }
};