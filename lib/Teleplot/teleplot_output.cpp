#include "teleplot_output.h"

#include <Arduino.h>


void teleplotInit() {
    Serial.begin(115200);
}


void teleplotPrintImuSample(const ImuSample& sample) {
    /*
     * Accelerometer: g
     */

    Serial.print(">acc_x:");
    Serial.println(sample.ax, 3);

    Serial.print(">acc_y:");
    Serial.println(sample.ay, 3);

    Serial.print(">acc_z:");
    Serial.println(sample.az, 3);


    /*
     * Gyroscope: degrees per second
     */

    Serial.print(">gyro_x:");
    Serial.println(sample.gx, 2);

    Serial.print(">gyro_y:");
    Serial.println(sample.gy, 2);

    Serial.print(">gyro_z:");
    Serial.println(sample.gz, 2);
}