/**************************************************************************************************
 * Author: Anthony Zhang
 *
 * License: (c) 2021, MIT LICENSE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions: The above copyright notice and this
 * permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Description: ByteRingBuffer.cpp
 * Ring buffer for Nordic NRF-52840 UART SPP 
 *
 ***************************************************************************************************/
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
        newestIndex = 0;
        length = 0;
    }

    /// @brief this.get(0) is the oldest value, this.get(this.getLength() - 1) is the newest value
    /// @param index
    /// @return value as integer
    int get(size_t index)
    {
        if (index < 0 || index >= length)
        {
            return -1;
        }
        return ringBuffer[(N + newestIndex - length + index) % N];
    }

    /// @brief returns buffer length
    /// @return value as integer
    size_t getLength()
    {
        return length;
    }
};