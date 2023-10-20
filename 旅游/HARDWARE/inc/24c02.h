#ifndef AT24C02_H

#include "main.h"
#include "i2c.h"

int AT24C02_write(uint8_t addr, uint8_t *dataPtr, uint16_t dataSize);
int AT24C02_read (uint8_t addr, uint8_t *dataPtr, uint16_t dataSize);
uint8_t AT24C02_Check(void);


#endif
