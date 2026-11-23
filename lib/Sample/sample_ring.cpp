#include "sample_ring.h"

#include <mbed.h>

#include <stddef.h>
#include <stdint.h>


namespace {


/*
 * Ring-buffer storage.
 */
ImuSample ringBuffer[IMU_WINDOW_SIZE];


/*
 * Position where the next sample will be written.
 */
size_t writeIndex = 0;


/*
 * Number of valid samples currently stored.
 *
 * The value increases from 0 to IMU_WINDOW_SIZE.
 */
size_t sampleCount = 0;


/*
 * Sequence number to assign to the next accepted sample.
 */
uint32_t nextSequence = 0;


/*
 * Identifies the current ring-buffer data segment.
 */
uint32_t ringGeneration = 0;


/*
 * Protect the ring while a different thread copies a complete
 * window.
 */
rtos::Mutex ringMutex;


} // namespace


void sampleRingClear() {
    ringMutex.lock();

    writeIndex = 0;
    sampleCount = 0;

    /*
     * Mark the beginning of a new data segment.
     *
     * nextSequence is intentionally not reset.
     */
    ++ringGeneration;

    ringMutex.unlock();
}


void sampleRingAppend(const ImuSample& sample) {
    ringMutex.lock();

    /*
     * Store the new sample.
     */
    ringBuffer[writeIndex] = sample;

    /*
     * Advance to the next position.
     */
    writeIndex = (writeIndex + 1) % IMU_WINDOW_SIZE;

    if (sampleCount < IMU_WINDOW_SIZE) {
        ++sampleCount;
    }

    ++nextSequence;

    ringMutex.unlock();
}


bool sampleRingCopyWindow(ImuWindow& window) {
    ringMutex.lock();

    /*
     * A signal-processing window must contain 256 samples.
     */
    if (sampleCount < IMU_WINDOW_SIZE) {
        ringMutex.unlock();
        return false;
    }

    /*
     * When the ring is full, writeIndex points to the oldest sample.
     *
     * Copy from oldest to newest.
     */
    for (size_t i = 0; i < IMU_WINDOW_SIZE; ++i) {
        size_t ringIndex =
            (writeIndex + i) % IMU_WINDOW_SIZE;

        window.samples[i] = ringBuffer[ringIndex];
    }

    window.firstSequence =
        nextSequence -
        static_cast<uint32_t>(IMU_WINDOW_SIZE);

    window.lastSequence = nextSequence - 1;

    window.generation = ringGeneration;

    ringMutex.unlock();

    return true;
}