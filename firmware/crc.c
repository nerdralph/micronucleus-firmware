#include <stdio.h>

const char DD[] = {
    18,                     // sizeof(descriptor): length in bytes
    1,                      // device descriptor type
    0x10, 0x01,             // USB version supported
    0,                      // device class
    0,                      // device subclass
    0,                      // protocol
    8,                      /* max packet size */
};

// CRC16 of 8 byte block
uint16_t crc16_8(const char* block)
{
    uint16_t count = 8, crc = 0;
    do {
        crc ^= *block++;       
        int lsbit = crc & 0x0001;
        crc = (crc >> 1) | 0x8000;
        if (lsbit == 0)
            crc ^= 0xa001;
    } while (--count);
    return crc;
}

int main()
{
    return crc16_8(&DD[0]);
}
