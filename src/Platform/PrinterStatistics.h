/*
 * PrinterStatistics.h
 *
 *  Created on: May 19, 2026
 *      Author: stedmaja
 *
 *  Lifetime usage/wear statistics.
 *  Motor travel is tracked per LOGICAL DRIVE in absolute microsteps. Because the DDA transforms
 *  axis coordinates into motor steps (CartesianToMotorSteps), this is true per-motor travel and is
 *  therefore correct on every kinematic (Cartesian, CoreXY, delta, SCARA, 5-axis, IDEX, ...).
 */

#ifndef SRC_PLATFORM_PRINTERSTATISTICS_H_
#define SRC_PLATFORM_PRINTERSTATISTICS_H_

#include <RepRapFirmware.h>

struct PrinterStatistics
{
    double lifetimePrintSeconds = 0.0;						// total time spent actually printing
    uint32_t lifetimePrintJobs = 0;							// number of print jobs completed
    uint64_t driveMicrosteps[MaxAxesPlusExtruders] = { 0 };	// absolute commanded motor travel per logical drive, in microsteps
};

#endif /* SRC_PLATFORM_PRINTERSTATISTICS_H_ */
