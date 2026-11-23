#pragma once

#include <stddef.h>
#include <stdint.h>


/*
 * Number of six-axis samples in one analysis window.
 */
constexpr size_t IMU_WINDOW_SIZE = 256;


/*
 * One six-axis IMU sample.
 *
 * Accelerometer unit: g
 * Gyroscope unit: degrees per second
 */
struct ImuSample {
    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;
};


/*
 * A snapshot of the latest 256 IMU samples.
 *
 * samples[0] is the oldest sample.
 * samples[IMU_WINDOW_SIZE - 1] is the newest sample.
 */
struct ImuWindow {
    ImuSample samples[IMU_WINDOW_SIZE];

    /*
     * MCU-assigned sequence numbers for this window.
     *
     * These sequence numbers identify samples accepted by the
     * software ring buffer. They do not by themselves prove that
     * no sensor sample was lost before entering the ring.
     */
    uint32_t firstSequence;
    uint32_t lastSequence;

    /*
     * Changes when the ring is cleared after startup or a detected
     * FIFO discontinuity.
     */
    uint32_t generation;
};


/*
 * Basic state of the IMU module.
 */
enum class ImuState : uint8_t {
    STARTING,
    RUNNING,
    ERROR
};


/*
 * Start the IMU FIFO acquisition thread.
 *
 * Returns true if the RTOS thread started.
 * Sensor initialization occurs inside the thread.
 */
bool imuStart();


/*
 * Return the current IMU module state.
 */
ImuState imuGetState();


/*
 * Get one sample for Teleplot display.
 *
 * This function is nonblocking.
 *
 * Returns:
 *     true  - one display sample was copied.
 *     false - no display sample is currently available.
 */
bool imuGetDisplaySample(ImuSample& sample);


/*
 * Copy the latest complete 256-sample window.
 *
 * Returns:
 *     true  - a complete window was copied.
 *     false - fewer than 256 samples are currently available.
 *
 * The caller decides when to call this function.
 *
 * Call it from a thread, not from an ISR.
 */
bool imuCopyLatestWindow(ImuWindow& window);