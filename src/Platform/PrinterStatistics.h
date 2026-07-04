/*
 * PrinterStatistics.h
 *
 *  Created on: May 19, 2026
 *      Author: stedmaja
 */

#ifndef SRC_PLATFORM_PRINTERSTATISTICS_H_
#define SRC_PLATFORM_PRINTERSTATISTICS_H_

struct PrinterStatistics
{
    double lifetimePrintSeconds = 0.0f;
    unsigned long lifetimePrintJobs = 0;
    double xMillis = 0.0f;
    double yMillis = 0.0f;
    double zMillis = 0.0f;
    double bDegrees = 0.0f;
    double cDegrees = 0.0f;
    double eMillis = 0.0f;
};



#endif /* SRC_PLATFORM_PRINTERSTATISTICS_H_ */
