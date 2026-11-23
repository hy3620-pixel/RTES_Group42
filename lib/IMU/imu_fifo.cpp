#include "imu_fifo.h"
#include "sample_ring.h"

#include <Arduino.h>
#include <Wire.h>
#include <LSM6DS3.h>
#include <mbed.h>

#include <atomic>
#include <stddef.h>
#include <stdint.h>


namespace {


/* ================================================================
 *  1. IMU and FIFO configuration
 * ================================================================ */

constexpr uint8_t IMU_ADDRESS = 0x6A;

constexpr uint32_t IMU_ODR_HZ = 52;

// One six-axis frame contains six 16-bit words.
constexpr uint16_t WORDS_PER_FRAME = 6;

/*
 * FIFO watermark in six-axis frames.
 *
 * Change this value if your team later chooses another watermark.
 */
constexpr uint16_t FIFO_WATERMARK_FRAMES = 10;

constexpr uint16_t FIFO_WATERMARK_WORDS =
    FIFO_WATERMARK_FRAMES * WORDS_PER_FRAME;

// Maximum number of frames read in one processing batch.
constexpr size_t MAX_BATCH_FRAMES = 16;

// Maximum number of samples waiting for Teleplot.
constexpr size_t DISPLAY_POOL_SIZE = 64;

// Thread flag set by the FIFO watermark ISR.
constexpr uint32_t FIFO_READY_FLAG = 0x01;


/* ================================================================
 *  2. LSM6DS3TR-C register addresses
 * ================================================================ */

constexpr uint8_t REG_FIFO_CTRL1 = 0x06;
constexpr uint8_t REG_FIFO_CTRL2 = 0x07;
constexpr uint8_t REG_FIFO_CTRL3 = 0x08;
constexpr uint8_t REG_FIFO_CTRL4 = 0x09;
constexpr uint8_t REG_FIFO_CTRL5 = 0x0A;

constexpr uint8_t REG_INT1_CTRL = 0x0D;
constexpr uint8_t REG_CTRL3_C = 0x12;
constexpr uint8_t REG_MASTER_CONFIG = 0x1A;

constexpr uint8_t REG_FIFO_STATUS1 = 0x3A;
constexpr uint8_t REG_FIFO_DATA_OUT_L = 0x3E;


/* ================================================================
 *  3. Register values
 * ================================================================ */

// FIFO bypass mode: disabled and cleared.
constexpr uint8_t FIFO_BYPASS = 0x00;

// FIFO ODR 52 Hz and continuous mode.
constexpr uint8_t FIFO_CONTINUOUS_52HZ = 0x1E;

// Route FIFO watermark to INT1.
constexpr uint8_t INT1_FIFO_WATERMARK = 0x08;

// FIFO_STATUS2 overrun flag.
constexpr uint8_t FIFO_OVERRUN_MASK = 0x40;


/* ================================================================
 *  4. Internal data structure
 * ================================================================ */

struct FifoStatus {
    uint16_t unreadWords;
    uint16_t pattern;
    bool overrun;
};


/* ================================================================
 *  5. Internal objects
 * ================================================================ */

LSM6DS3 imu(I2C_MODE, IMU_ADDRESS);

rtos::Thread imuThread(
    osPriorityAboveNormal,
    4096,
    nullptr,
    "IMU_FIFO_Task"
);


/*
 * This queue carries samples for Teleplot only.
 *
 * It is separate from the ring buffer.
 */
rtos::MemoryPool<ImuSample, DISPLAY_POOL_SIZE> displayPool;

rtos::Queue<ImuSample, DISPLAY_POOL_SIZE> displayQueue;


std::atomic<ImuState> imuState{ImuState::STARTING};


/* ================================================================
 *  6. FIFO watermark interrupt
 * ================================================================ */

void fifoWatermarkISR() {
    /*
     * Wake the IMU task.
     *
     * Do not perform I2C or Serial operations inside the ISR.
     */
    imuThread.flags_set(FIFO_READY_FLAG);
}


/* ================================================================
 *  7. I2C read helper
 * ================================================================ */

/*
 * The onboard IMU uses Wire1 in the XIAO nRF52840 Sense
 * Mbed board configuration.
 */
bool readBytes(
    uint8_t startRegister,
    uint8_t* destination,
    uint8_t length
) {
    Wire1.beginTransmission(IMU_ADDRESS);

    Wire1.write(startRegister);

    if (Wire1.endTransmission(false) != 0) {
        return false;
    }

    uint8_t received =
        Wire1.requestFrom(IMU_ADDRESS, length);

    if (received != length) {
        while (Wire1.available()) {
            Wire1.read();
        }

        return false;
    }

    for (uint8_t i = 0; i < length; ++i) {
        if (!Wire1.available()) {
            return false;
        }

        destination[i] =
            static_cast<uint8_t>(Wire1.read());
    }

    return true;
}


/* ================================================================
 *  8. Read FIFO status
 * ================================================================ */

bool readFifoStatus(FifoStatus& status) {
    uint8_t data[4];

    if (
        !readBytes(
            REG_FIFO_STATUS1,
            data,
            sizeof(data)
        )
    ) {
        return false;
    }

    /*
     * FIFO_STATUS1 and FIFO_STATUS2:
     * number of unread 16-bit words.
     */
    status.unreadWords =
        static_cast<uint16_t>(data[0]) |
        (
            static_cast<uint16_t>(data[1] & 0x07)
            << 8
        );

    status.overrun =
        (data[1] & FIFO_OVERRUN_MASK) != 0;

    /*
     * FIFO_STATUS3 and FIFO_STATUS4:
     * next word's position in the FIFO pattern.
     */
    status.pattern =
        static_cast<uint16_t>(data[2]) |
        (
            static_cast<uint16_t>(data[3] & 0x03)
            << 8
        );

    return true;
}


/* ================================================================
 *  9. Read FIFO data
 * ================================================================ */

bool readFifoWord(int16_t& value) {
    uint8_t data[2];

    if (
        !readBytes(
            REG_FIFO_DATA_OUT_L,
            data,
            sizeof(data)
        )
    ) {
        return false;
    }

    uint16_t combined =
        static_cast<uint16_t>(data[0]) |
        (
            static_cast<uint16_t>(data[1])
            << 8
        );

    value = static_cast<int16_t>(combined);

    return true;
}


/*
 * Current FIFO pattern:
 *
 *     gyro X, gyro Y, gyro Z,
 *     accel X, accel Y, accel Z.
 *
 * This order is tied to the current FIFO configuration.
 */
bool readFifoFrame(ImuSample& sample) {
    int16_t gxRaw = 0;
    int16_t gyRaw = 0;
    int16_t gzRaw = 0;

    int16_t axRaw = 0;
    int16_t ayRaw = 0;
    int16_t azRaw = 0;

    if (!readFifoWord(gxRaw)) return false;
    if (!readFifoWord(gyRaw)) return false;
    if (!readFifoWord(gzRaw)) return false;

    if (!readFifoWord(axRaw)) return false;
    if (!readFifoWord(ayRaw)) return false;
    if (!readFifoWord(azRaw)) return false;

    /*
     * Convert raw sensor data to physical units using the
     * configured ranges in the Seeed library.
     */
    sample.gx = imu.calcGyro(gxRaw);
    sample.gy = imu.calcGyro(gyRaw);
    sample.gz = imu.calcGyro(gzRaw);

    sample.ax = imu.calcAccel(axRaw);
    sample.ay = imu.calcAccel(ayRaw);
    sample.az = imu.calcAccel(azRaw);

    return true;
}


/* ================================================================
 *  10. Restart FIFO
 * ================================================================ */

bool restartFifo() {
    /*
     * Bypass mode clears the FIFO.
     */
    if (
        imu.writeRegister(
            REG_FIFO_CTRL5,
            FIFO_BYPASS
        ) != IMU_SUCCESS
    ) {
        return false;
    }

    /*
     * Restart FIFO in continuous mode.
     */
    if (
        imu.writeRegister(
            REG_FIFO_CTRL5,
            FIFO_CONTINUOUS_52HZ
        ) != IMU_SUCCESS
    ) {
        return false;
    }

    return true;
}


/* ================================================================
 *  11. Initialize IMU and FIFO
 * ================================================================ */

bool initIMUAndFIFO() {
    /*
     * Configure sensor output data rates and measurement ranges.
     */
    imu.settings.accelSampleRate = IMU_ODR_HZ;
    imu.settings.accelRange = 4;

    imu.settings.gyroSampleRate = IMU_ODR_HZ;
    imu.settings.gyroRange = 500;

    if (imu.begin() != IMU_SUCCESS) {
        return false;
    }

    /*
     * Disable INT1 routing while configuring the sensor.
     */
    if (
        imu.writeRegister(
            REG_INT1_CTRL,
            0x00
        ) != IMU_SUCCESS
    ) {
        return false;
    }

    /*
     * Disable and clear FIFO before changing its configuration.
     */
    if (
        imu.writeRegister(
            REG_FIFO_CTRL5,
            FIFO_BYPASS
        ) != IMU_SUCCESS
    ) {
        return false;
    }

    /*
     * CTRL3_C:
     *
     * BDU = 1:
     *     Protect multi-byte output reads.
     *
     * IF_INC = 1:
     *     Automatically increment register addresses.
     *
     * BLE = 0:
     *     Use low-byte/high-byte order.
     */
    uint8_t ctrl3 = 0;

    if (
        imu.readRegister(
            &ctrl3,
            REG_CTRL3_C
        ) != IMU_SUCCESS
    ) {
        return false;
    }

    ctrl3 |= 0x44;
    ctrl3 &= static_cast<uint8_t>(~0x02);

    if (
        imu.writeRegister(
            REG_CTRL3_C,
            ctrl3
        ) != IMU_SUCCESS
    ) {
        return false;
    }

    /*
     * The current project does not use the sensor hub.
     */
    if (
        imu.writeRegister(
            REG_MASTER_CONFIG,
            0x00
        ) != IMU_SUCCESS
    ) {
        return false;
    }

    /*
     * FIFO watermark: lower eight bits.
     */
    if (
        imu.writeRegister(
            REG_FIFO_CTRL1,
            static_cast<uint8_t>(
                FIFO_WATERMARK_WORDS & 0xFF
            )
        ) != IMU_SUCCESS
    ) {
        return false;
    }

    /*
     * FIFO watermark: upper bits.
     */
    if (
        imu.writeRegister(
            REG_FIFO_CTRL2,
            static_cast<uint8_t>(
                (FIFO_WATERMARK_WORDS >> 8) & 0x07
            )
        ) != IMU_SUCCESS
    ) {
        return false;
    }

    /*
     * Enable gyroscope and accelerometer in FIFO without
     * decimation.
     */
    if (
        imu.writeRegister(
            REG_FIFO_CTRL3,
            0x09
        ) != IMU_SUCCESS
    ) {
        return false;
    }

    /*
     * Use full 16-bit data and disable extra FIFO data sets.
     */
    if (
        imu.writeRegister(
            REG_FIFO_CTRL4,
            0x00
        ) != IMU_SUCCESS
    ) {
        return false;
    }

    /*
     * Attach MCU interrupt before starting FIFO.
     */
    pinMode(PIN_LSM6DS3TR_C_INT1, INPUT);

    attachInterrupt(
        digitalPinToInterrupt(
            PIN_LSM6DS3TR_C_INT1
        ),
        fifoWatermarkISR,
        RISING
    );

    /*
     * Route FIFO watermark signal to IMU INT1.
     */
    if (
        imu.writeRegister(
            REG_INT1_CTRL,
            INT1_FIFO_WATERMARK
        ) != IMU_SUCCESS
    ) {
        detachInterrupt(
            digitalPinToInterrupt(
                PIN_LSM6DS3TR_C_INT1
            )
        );

        return false;
    }

    /*
     * Start FIFO in continuous mode.
     */
    if (
        imu.writeRegister(
            REG_FIFO_CTRL5,
            FIFO_CONTINUOUS_52HZ
        ) != IMU_SUCCESS
    ) {
        /*
         * Best-effort cleanup of the interrupt configuration.
         */
        imu.writeRegister(REG_INT1_CTRL, 0x00);

        detachInterrupt(
            digitalPinToInterrupt(
                PIN_LSM6DS3TR_C_INT1
            )
        );

        return false;
    }

    return true;
}


/* ================================================================
 *  12. Send one sample for Teleplot
 * ================================================================ */

void sendDisplaySample(const ImuSample& sample) {
    ImuSample* copy = displayPool.alloc();

    if (copy == nullptr) {
        /*
         * Skip this display update rather than block acquisition.
         *
         * The sample has already been written to the ring buffer.
         */
        return;
    }

    *copy = sample;

    if (displayQueue.put(copy, 0) != osOK) {
        displayPool.free(copy);
    }
}


/* ================================================================
 *  13. FIFO recovery
 * ================================================================ */

bool recoverFifo(bool& discardFirstFrame) {
    /*
     * A detected FIFO error can create a data discontinuity.
     *
     * Clear the ring so that samples from before and after the
     * recovery are not returned as one complete window.
     */
    sampleRingClear();

    if (!restartFifo()) {
        imuState.store(ImuState::ERROR);
        return false;
    }

    /*
     * Discard the first complete frame after restarting FIFO.
     */
    discardFirstFrame = true;

    return true;
}


/* ================================================================
 *  14. RTOS IMU acquisition task
 * ================================================================ */

void imuTask() {
    if (!initIMUAndFIFO()) {
        imuState.store(ImuState::ERROR);
        return;
    }

    /*
     * Start with an empty ring buffer.
     */
    sampleRingClear();

    imuState.store(ImuState::RUNNING);

    /*
     * Discard the first complete frame after FIFO starts.
     */
    bool discardFirstFrame = true;

    for (;;) {
        /*
         * Normally the FIFO watermark ISR wakes this task.
         *
         * The timeout permits a periodic FIFO check if an interrupt
         * edge is missed or no interrupt arrives.
         */
        rtos::ThisThread::flags_wait_any_for(
            FIFO_READY_FLAG,
            1000
        );

        /*
         * Drain complete frames until fewer than six words remain.
         */
        for (;;) {
            FifoStatus before{};

            if (!readFifoStatus(before)) {
                if (!recoverFifo(discardFirstFrame)) {
                    return;
                }

                break;
            }

            if (before.overrun) {
                if (!recoverFifo(discardFirstFrame)) {
                    return;
                }

                break;
            }

            if (before.unreadWords < WORDS_PER_FRAME) {
                break;
            }

            /*
             * Pattern 0 is the expected first word of a complete
             * frame in the current FIFO configuration.
             */
            if (before.pattern != 0) {
                if (!recoverFifo(discardFirstFrame)) {
                    return;
                }

                break;
            }

            size_t framesToRead =
                before.unreadWords / WORDS_PER_FRAME;

            if (framesToRead > MAX_BATCH_FRAMES) {
                framesToRead = MAX_BATCH_FRAMES;
            }

            ImuSample batch[MAX_BATCH_FRAMES]{};

            bool readOk = true;

            for (size_t i = 0; i < framesToRead; ++i) {
                if (!readFifoFrame(batch[i])) {
                    readOk = false;
                    break;
                }
            }

            if (!readOk) {
                /*
                 * A partial read can leave FIFO alignment uncertain.
                 */
                if (!recoverFifo(discardFirstFrame)) {
                    return;
                }

                break;
            }

            /*
             * Check FIFO status again before accepting the batch.
             */
            FifoStatus after{};

            if (!readFifoStatus(after) || after.overrun) {
                if (!recoverFifo(discardFirstFrame)) {
                    return;
                }

                break;
            }

            /*
             * Write every accepted FIFO frame to the ring buffer.
             */
            for (size_t i = 0; i < framesToRead; ++i) {
                if (discardFirstFrame) {
                    discardFirstFrame = false;
                    continue;
                }

                sampleRingAppend(batch[i]);

                /*
                 * Send a copy for Teleplot display.
                 *
                 * Display updates may be skipped if the display
                 * queue is full; ring-buffer storage is unaffected.
                 */
                sendDisplaySample(batch[i]);
            }
        }
    }
}


} // namespace


/* ================================================================
 *  15. Public interface
 * ================================================================ */

bool imuStart() {
    osStatus status = imuThread.start(imuTask);

    if (status != osOK) {
        imuState.store(ImuState::ERROR);
        return false;
    }

    return true;
}


ImuState imuGetState() {
    return imuState.load();
}


bool imuGetDisplaySample(ImuSample& sample) {
    osEvent received = displayQueue.get(0);

    if (received.status != osEventMessage) {
        return false;
    }

    ImuSample* queuedSample =
        static_cast<ImuSample*>(received.value.p);

    sample = *queuedSample;

    displayPool.free(queuedSample);

    return true;
}


bool imuCopyLatestWindow(ImuWindow& window) {
    return sampleRingCopyWindow(window);
}