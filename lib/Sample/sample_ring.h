#pragma once

#include "imu_fifo.h"


/*
 * Clear the ring buffer and start a new generation.
 */
void sampleRingClear();


/*
 * Append one sample to the ring buffer.
 *
 * The IMU acquisition task is the only writer.
 */
void sampleRingAppend(const ImuSample& sample);


/*
 * Copy the latest complete 256-sample window.
 *
 * Returns false until 256 samples are available.
 *
 * Call from a thread, not from an ISR.
 */
bool sampleRingCopyWindow(ImuWindow& window);