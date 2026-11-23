#pragma once

#include "imu_fifo.h"


/*
 * Initialize the serial connection used by Teleplot.
 */
void teleplotInit();


/*
 * Output one six-axis sample in Teleplot format.
 */
void teleplotPrintImuSample(const ImuSample& sample);