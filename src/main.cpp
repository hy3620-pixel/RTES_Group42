#include <Arduino.h>
#include <mbed.h>

#include "imu_fifo.h"
#include "teleplot_output.h"


void setup() {
    /*
     * Initialize serial output for Teleplot.
     */
    teleplotInit();

    /*
     * Start the IMU FIFO acquisition task.
     */
    imuStart();
}


void loop() {
    ImuSample sample{};

    /*
     * Display a sample if one is available.
     */
    if (imuGetDisplaySample(sample)) {
        teleplotPrintImuSample(sample);
    }

    /*
     * Allow other RTOS threads to run.
     */
    rtos::ThisThread::sleep_for(5);
}