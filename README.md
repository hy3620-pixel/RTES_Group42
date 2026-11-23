Development Notes —— from Haoran(Ryan) Yu

1.Project Objective
This project is the final assignment for the RTES course, involving the 
creation of a wrist-worn motion detection prototype using the Seeed Studio 
XIAO nRF52840 Sense.

The device utilizes a built-in six-axis IMU to collect wrist motion data,
identifying continuous rhythmic movements similar to target simulated 
actions. When the detection results meet the conditions, the system can 
notify the user via onboard LEDs and BLE.

2.Division of Work
2.1 IMU Data Collection and Code structure by Haoran(Ryan) Yu
2.2 Signal Processing and Motion Detection by Rhea Shastri
2.3 LED and BLE notification by Gongze(Jeffrey) Zhang

3.Project Structure
------lib
   |------imu_fifo.h : IMU data structure and external interfaces. The 
                       signal processing module should include this header.
   |------imu_fifo.c : IMU initialization, FIFO configuration, interrupts, 
                       RTOS acquisition threads, and FIFO reads.
   |------sample_ring.h/.c : Save the last 256 sets of samples and provide 
                       window copy function.
   |------teleplot_output.h/.c : Output six axis data to Teleplot.
------src
   |------main.c : RTOS initialization, IMU initialization, and thread

4.Environment Setup
4.1 Hardware
- Seeed Studio XIAO nRF52840 Sense
4.2 Software
- VS Code
- PlatformIO
- Arduino over Mbed
- Seeed Arduino LSM6DS3 lib

5.IMU Configuration
Nominal ODR of accelerometer:	 52 Hz
Nominal ODR of gyroscope:   	 52 Hz
Range of accelerometer:	         ±4 g
Range of gyroscope:         	 ±500 dps
FIFO mode:	                     Continuous mode
FIFO water level:	             Current code is 10 sets of six-axis data
Ring buffer length:	             256 sets of six-axis data
IMU acquisition execution method:Mbed RTOS thread
FIFO data transmission:	         I²C
Teleplot serial port:	         115200 baud

6.IMU data generation and transmission
The IMU data collection task will write valid data into the MCU ring 
buffer group by group; The signal processing task only reads the sorted 
256 point window.

The IMU task will not:
-Create a signal processing thread;
-Automatically notify signal processing tasks for every 52 new sets of data added;
-Automatically execute FFT;
-Automatic classification.

The person in charge of signal processing decides on their own:
-When to start the signal processing task;
-How often should the window be read;
-How to coordinate task priority and execution time.

7.Public Data structure Description
7.1 IMU data
struct ImuSample {
    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;
};
7.2 256 points window
struct ImuWindow {
    ImuSample samples[256];

    uint32_t firstSequence;
    uint32_t lastSequence;
    uint32_t generation;
};
//Samples [0] → The earliest sample in the window
//Samples [255] → Latest samples in the window
If you only want to take the value of a certain axis, you can write:
float value = window.samples[10].ax;
Indicates the X-axis acceleration of the 11th set of data obtained from the window.
7.3 Fields window.firstSequence and window.lastSequence
Two fields are maintained by the MCU when the sample enters the ring buffer.
If the window is complete, under normal circumstances: lastSequence - firstSequence = 255
7.4 Field window.generation
When the system starts or detects a FIFO exception and clears the ring 
buffer, the 'generation' will change.

8.Get the latest window
8.1 Public interface
bool imuCopyLatestWindow(ImuWindow& window);
Return:
- true: Successfully copied the latest 256 points window.
- false: The window is not complete, and the copy failed.

Possible situations that may return false include:
-Just started, haven't accumulated enough data for 256 sets yet;
-FIFO encountered an exception detected by the program, causing the 
ring buffer to be cleared and re accumulating

8.2 Simple usage method
#include "imu_fifo.h"

static ImuWindow window;

void example() {
    if (imuCopyLatestWindow(window)) {
        /*
         * Use here:   
         *
         * window.samples[0] ... window.samples[255]
         */
    }
}

8.3 The window is a snapshot and will not automatically update
imuCopyLatestWindow(window) will copy the 256 sets of samples 
with the closest call time to the window.

Even if the IMU continues to sample afterwards, the window will 
not automatically update. The function needs to be called again 
to obtain a new window.

8.4 This function will briefly use mutex locks
During the window copying process, the ring buffer is protected by an 
RTOS mutex lock.

The signal processing code should:
-First, call imuCopyRatestWindow();
-After the function returns, calculate the features and FFT;
-Do not modify the underlying buffer to perform FFT during lock up.

9.Suggested route for signal processing
Obtain 256 sets of six axis data
↓
Check the window and generation
↓
preprocessing
↓
time-domain features
↓
FFT and Frequency Domain Characteristics
↓
Artificial threshold rule
↓
Duration and state machine
↓
Suspected sports event

10.Constraints on the development of collaborative AI
10.1. Current goal: Implement rule-based motion feature extraction, FFT, 
and classification on XIAO nRF52840 Sense.
10.2. Do not use machine learning models, including TinyML, neural networks,
or other training models.
10.3. IMU/FIFO/ring buffer has been maintained by other members; First, read 
imu_fifo. h, imu_fifo. cpp, sample_ring. h, and sample_ring. cpp.
10.4. Prioritize obtaining input through imuCopyRatestWindow() and do not 
access the IMU's IIC or FIFO simultaneously.
10.5. The signal processing task decides on its own when to read the window;
The current interface only returns the latest snapshot and does not 
guarantee strict delivery of 52 sample steps.

Good luck!
Nov 22
