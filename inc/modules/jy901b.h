#ifndef JY901B_H
#define JY901B_H

#include "measure_types.h"

#include <stdint.h>

void Jy901b_Reset(void);
void Jy901b_Configure(void);
void Jy901b_CalibrateAccGyro(void);
void Jy901b_CalibrateRefAngle(void);
void Jy901b_StartMagCalibration(void);
void Jy901b_StopMagCalibration(void);
bool Jy901b_ProcessByte(uint8_t byte, OrientationData* data);

#endif /* JY901B_H */
