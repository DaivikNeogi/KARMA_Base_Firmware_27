# Robot Firmware: FreeRTOS Real-Time Architecture Guide

This document explains the transition of the autonomous 4-wheel robot base from a bare-metal loop to a **strict zero-heap FreeRTOS** multi-tasking architecture.

---

## 1. Why Was the Old Code Having Issues?

In the previous bare-metal superloop implementation, several hidden concurrency and timing issues existed:

1. **USB Transmit Overwrites & Garbled JSON:**
   `CDC_Transmit_FS()` is completely non-blocking—it does not copy data; it gives the STM32 USB hardware engine a pointer to RAM (`s_tx_buf`). When the main loop started formatting the next telemetry frame while the USB hardware was still transmitting the previous frame, the Jetson received hybrid, garbled JSON (`JSONDecodeError`).
2. **Silent Packet Drops (`USBD_BUSY`):**
   If a command acknowledgment (ACK) was transmitting over USB and periodic telemetry tried to send at that exact same millisecond, `CDC_Transmit_FS` returned `USBD_BUSY`. Because the return code was ignored, telemetry frames were permanently dropped.
3. **Heavy JSON Parsing in Interrupt Service Routine (ISR):**
   Incoming USB bytes triggered an interrupt that directly executed string searching (`strstr`), ASCII parsing (`atoi`), and timer PWM register updates inside the USB ISR. This stalled CPU interrupts and risked data corruption.
4. **Motor Noise Freezing the I2C Bus:**
   Motor brush electrical noise or an abrupt MCU reset during an I2C transaction could leave the BNO085 holding the SDA data line low indefinitely, freezing the entire superloop.
5. **No Independent Deadman Watchdog:**
   If sensor reads stalled or the loop delayed, the safety watchdog check was delayed, creating a runaway hazard.

---

## 2. The New FreeRTOS Architecture Overview

The system is now structured into **4 isolated tasks** following **Rate Monotonic Scheduling (RMS)**—where higher frequency, safety-critical tasks preempt lower-frequency background tasks:

```
                                  [ Jetson Orin / PC ]
                                      ▲          │
         USB Telemetry (50 Hz JSON)  │          │ USB Commands (JSON)
                                      │          ▼
                        ┌──────────────────────────────┐
                        │    USB Interrupt (Rx ISR)    │
                        └──────────────┬───────────────┘
                                       │ Pushes raw bytes (< 1 µs)
                                       ▼
                              ┌─────────────────┐
                              │    q_usb_rx     │
                              └────────┬────────┘
                                       │
┌──────────────────────────────────────┴───────────────────────────────────────┐
│                               FreeRTOS Tasks                                 │
│                                                                              │
│  [Priority 4 - 100 Hz / 10 ms]                                               │
│  MotorTask                                                                   │
│   • 150 ms Deadman Watchdog: Kills power if Jetson stops communicating.       │
│   • Reads commanded speeds from q_motor_cmd.                                 │
│   • Writes duty cycle & direction pins to 4x MD30C motor drivers.            │
│                                                                              │
│  [Priority 3 - 50 Hz / 20 ms]                                                │
│  AcquisitionTask                                                             │
│   • 2D State Matrix manages BNO085 IMU initialization & recovery.            │
│   • Reads 3-channel dead-wheel hardware encoders (unsigned two's complement).│
│   • Pushes coherent sensor_data_t to q_sensor_data.                          │
│   • (Optional: Optical Flow & ToF via ENABLE_OPTICAL_FLOW_TOF flag).         │
│                                                                              │
│  [Priority 2 - 50 Hz / 20 ms]                                                │
│  EstimationTask                                                              │
│   • Consumes q_sensor_data.                                                  │
│   • Dead Reckoning Math: Wheel radius R = 35 mm, Wheelbase W = 150 mm.       │
│   • Computes (X, Y, Heading, linear & angular velocity).                     │
│   • Updates thread-safe global robot state snapshot.                         │
│                                                                              │
│  [Priority 1 - Background / Event Driven]                                    │
│  CommunicationTask                                                           │
│   • In-place zero-allocation JSON parser (jsmn) extracts motor commands.     │
│   • Formats 50 Hz JSON telemetry string.                                     │
│   • USB Flow Control: Blocks on binary semaphore until hardware completes.   │
│                                                                              │
│  [Software Timer - 500 ms]                                                   │
│  Logger Timer                                                                │
│   • Flushes mutex-protected static ring buffer logger.                       │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Key Upgrades Explained in Plain English

### A. Strict Zero-Heap Embedded Architecture
* **What it means:** There is **zero dynamic memory allocation** in the entire codebase (`malloc`, `free`, and `heap_x.c` are completely eliminated).
* **Why it matters:** In robotics, heap fragmentation is a leading cause of random system crashes after hours of operation. All tasks, queues, semaphores, mutexes, and software timers are statically allocated at compile time.

### B. 150 ms Deadman Safety Loop (MotorTask @ 100 Hz)
* **What it means:** A timestamp is updated every time a valid command packet arrives from the Jetson. If the Jetson crashes, the Python ROS node freezes, or the USB cable disconnects for longer than **150 milliseconds**, `MotorTask` immediately drives all motor PWM duty cycles to **0%**.
* **Why it matters:** Prevents runaway robot collisions without waiting for a slow sensor loop.

### C. USB-CDC Binary Semaphore Flow Control
* **What it means:** `CommunicationTask` never calls `CDC_Transmit_FS()` blindly. It waits on a FreeRTOS binary semaphore (`xUsbTxSemaphore`). When the STM32 USB hardware finishes transmitting the packet across the physical cable, the hardware interrupt calls `CDC_TransmitCplt_FS()`, which releases the semaphore.
* **Why it matters:** Completely eliminates `USBD_BUSY` errors and prevents buffer overwrites. Telemetry and ACKs are delivered 100% reliably.

### D. Automatic 9-Clock I2C Bus Recovery
* **What it means:** Before I2C initialization (and whenever an I2C transaction stalls), the STM32 reconfigures SCL (PA8) and SDA (PB8) as GPIO outputs, pulses the SCL clock line 9 times, and issues a manual STOP condition.
* **Why it matters:** If electrical noise from the motors causes the BNO085 to hang mid-byte holding SDA low, this 9-clock cycle forces the sensor to reset its state machine and free the bus.

### E. Seamless 16-Bit Timer Counter Rollovers
* **What it means:** Encoder counts are read as `uint16_t` and deltas are calculated using unsigned two's complement subtraction:
  ```c
  int16_t delta = (int16_t)(current_raw - previous_raw);
  ```
* **Why it matters:** When the hardware timer rolls over between `65535` and `0` (or `0` to `65535` in reverse), the signed delta remains perfectly accurate (e.g. `(int16_t)(0 - 65535) = +1` tick) with zero branching or `if` statements.

### F. Dead Reckoning Pure Math (`algorithm.c`)
* **What it means:** Uses exact kinematics:
  * Wheel radius $R = 35.0\,\text{mm}$
  * Wheelbase $W = 150.0\,\text{mm}$
  * Distance per count = $\frac{2 \pi R}{\text{CPR}}$
  * Midpoint arc integration:
    $$\Delta s = \frac{\Delta s_{left} + \Delta s_{right}}{2}$$
    $$\Delta \theta = \frac{\Delta s_{right} - \Delta s_{left}}{W}$$
    $$x \mathrel{+}= \Delta s \cos\left(\theta + \frac{\Delta \theta}{2}\right), \quad y \mathrel{+}= \Delta s \sin\left(\theta + \frac{\Delta \theta}{2}\right)$$
* **Why it matters:** Produces smooth local odometry coordinates for autonomous navigation.

---

## 4. File Structure Summary

| File Path | Purpose |
| :--- | :--- |
| [Core/Inc/FreeRTOSConfig.h](file:///c:/Users/daivi/OneDrive/Desktop/EAAO_4_Motor_ENC/Core/Inc/FreeRTOSConfig.h) | FreeRTOS configuration: 1 kHz tick rate, zero-heap static allocation, stack check level 2. |
| [Core/Src/main.c](file:///c:/Users/daivi/OneDrive/Desktop/EAAO_4_Motor_ENC/Core/Src/main.c) | Peripheral setup, 9-clock I2C recovery, task manager initialization, and scheduler start. |
| [Core/Src/stm32f4xx_it.c](file:///c:/Users/daivi/OneDrive/Desktop/EAAO_4_Motor_ENC/Core/Src/stm32f4xx_it.c) | FreeRTOS scheduler tick integration (`xPortSysTickHandler`) and exception routing. |
| [USB_DEVICE/App/usbd_cdc_if.c](file:///c:/Users/daivi/OneDrive/Desktop/EAAO_4_Motor_ENC/USB_DEVICE/App/usbd_cdc_if.c) | Queue-based USB reception and semaphore-based transmit complete callbacks. |
| [tasks/task_manager.c](file:///c:/Users/daivi/OneDrive/Desktop/EAAO_4_Motor_ENC/tasks/task_manager.c) | Static memory definitions for all queues, semaphores, mutexes, and tasks. |
| [tasks/motor_task.c](file:///c:/Users/daivi/OneDrive/Desktop/EAAO_4_Motor_ENC/tasks/motor_task.c) | 100 Hz / 10 ms Motor control loop and 150 ms deadman safety watchdog. |
| [tasks/acquisition_task.c](file:///c:/Users/daivi/OneDrive/Desktop/EAAO_4_Motor_ENC/tasks/acquisition_task.c) | 50 Hz / 20 ms Sensor acquisition task (Encoders, BNO085, Optical Flow, ToF). |
| [tasks/estimation_task.c](file:///c:/Users/daivi/OneDrive/Desktop/EAAO_4_Motor_ENC/tasks/estimation_task.c) | 50 Hz / 20 ms Dead reckoning integration and robot state estimation. |
| [tasks/communication_task.c](file:///c:/Users/daivi/OneDrive/Desktop/EAAO_4_Motor_ENC/tasks/communication_task.c) | Zero-alloc JSON command parsing, 50 Hz telemetry formatting, and USB flow control. |
| [tasks/jsmn.h](file:///c:/Users/daivi/OneDrive/Desktop/EAAO_4_Motor_ENC/tasks/jsmn.h) | Lightweight, single-header in-place JSON tokenizer (zero-heap). |
| [algorithm.c](file:///c:/Users/daivi/OneDrive/Desktop/EAAO_4_Motor_ENC/algorithm.c) | Dead reckoning mathematics ($R=35\,\text{mm}$, $W=150\,\text{mm}$, arc integration). |
| [logger.c](file:///c:/Users/daivi/OneDrive/Desktop/EAAO_4_Motor_ENC/logger.c) | Mutex-protected static ring buffer logger with 500 ms software timer. |
| [drivers/i2c_recovery.c](file:///c:/Users/daivi/OneDrive/Desktop/EAAO_4_Motor_ENC/drivers/i2c_recovery.c) | 9-clock SCL bit-banging and manual STOP routine for I2C3 bus lockups. |
| [drivers/hw_encoders.c](file:///c:/Users/daivi/OneDrive/Desktop/EAAO_4_Motor_ENC/drivers/hw_encoders.c) | 3-channel encoder driver with two's-complement rollover delta tracking. |
| [drivers/motor_driver.c](file:///c:/Users/daivi/OneDrive/Desktop/EAAO_4_Motor_ENC/drivers/motor_driver.c) | 4x MD30C motor driver with duty cycle compare writes and bounds checking. |
| [drivers/optical_flow_tof.c](file:///c:/Users/daivi/OneDrive/Desktop/EAAO_4_Motor_ENC/drivers/optical_flow_tof.c) | Optical Flow and Time-of-Flight sensor data abstraction. |
| [drivers/bno085.c](file:///c:/Users/daivi/OneDrive/Desktop/EAAO_4_Motor_ENC/drivers/bno085.c) | BNO085 hardware interface driven by state machine. |
| [state_machines/imu_sm.c](file:///c:/Users/daivi/OneDrive/Desktop/EAAO_4_Motor_ENC/state_machines/imu_sm.c) | 2D State Matrix transition table for IMU lifecycle, error detection, and recovery. |
| [CMakeLists.txt](file:///c:/Users/daivi/OneDrive/Desktop/EAAO_4_Motor_ENC/CMakeLists.txt) | Build configuration including FreeRTOS kernel, drivers, and tasks. |

---

## 5. How to Enable Optical Flow & ToF in the Future

The Optical Flow and ToF subsystem is implemented as a dormant driver template so it consumes **0 bytes of RAM and 0 CPU cycles** by default:

1. Open [tasks/task_manager.h](file:///c:/Users/daivi/OneDrive/Desktop/EAAO_4_Motor_ENC/tasks/task_manager.h#L20).
2. Change:
   ```c
   #define ENABLE_OPTICAL_FLOW_TOF 0
   ```
   to:
   ```c
   #define ENABLE_OPTICAL_FLOW_TOF 1
   ```
3. Connect your sensor hardware pins and implement the SPI/I2C read logic inside [drivers/optical_flow_tof.c](file:///c:/Users/daivi/OneDrive/Desktop/EAAO_4_Motor_ENC/drivers/optical_flow_tof.c). Everything else (queues, task calls, and telemetry) will automatically integrate.
