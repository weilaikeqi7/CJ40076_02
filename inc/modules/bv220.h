#ifndef BV220_H
#define BV220_H

#include "measure_types.h"

#include <stdbool.h>
#include <stdint.h>

void Bv220_Reset(void);
bool Bv220_ProcessByte(uint8_t byte, GnssData* data);

#endif /* BV220_H */
