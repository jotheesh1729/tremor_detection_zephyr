# Tremor and Dyskinesia Detection Wearable

A wrist worn embedded system built on Zephyr RTOS that detects Parkinsonian tremor and dyskinesia from a wrist mounted IMU, classifies movement episodes in real time on the device, streams severity data over Bluetooth Low Energy, and supports secure over the air firmware updates through MCUboot.

## Problem statement

Parkinson's disease treatment is a balancing act. Dopamine therapy reduces tremor, a rhythmic oscillation typically in the 3 to 5 Hz range in a dominant limb. Too much dopamine causes dyskinesia instead, an involuntary, less rhythmic, dance like movement typically in the 5 to 7 Hz range. Clinicians need objective, continuous data on both symptoms to titrate medication correctly and keep a patient in the "on" state without tipping into dyskinesia.

This project builds a wearable that captures motion data from an onboard IMU, processes it on device with a real time DSP pipeline, and reports tremor and dyskinesia severity over BLE, with the ability to update the firmware in the field without physical access to the device.

## Origin

The project started from a university embedded systems course assignment: detect tremor and dyskinesia from a single accelerometer and gyroscope on a development board, using an FFT over 3 second windows and onboard indicators, with constraints such as no serial output and no additional hardware. That assignment is the origin of the idea, not the specification for this version. This project drops the classroom constraints and rebuilds the system as a production style embedded application, with custom driver development, RTOS level architecture, real time signal processing, and a secure boot and update chain.

## Hardware

- Target board: ST B-L475E-IOT01A IoT Discovery Kit. STM32L475VG, Cortex-M4F at 80 MHz, 1 MB flash, 128 KB SRAM, hardware FPU.
- IMU: LSM6DSL 6 axis accelerometer and gyroscope on I2C, with an onboard hardware FIFO and a watermark interrupt line wired to a GPIO.
- Bluetooth: SPBTLE-RF (BlueNRG-MS) module, HCI over SPI.
- Debug and flash probe: onboard ST-LINK/V2-1, SWD and virtual COM port.

Zephyr board name: `disco_l475_iot1`.

## System architecture

```
IMU (I2C, FIFO with watermark interrupt)
      |
      v   interrupt fires, handler only signals a semaphore
acquisition thread (high priority)
      |   blocking I2C burst read of the FIFO
      |   timestamp samples, convert to g
      v
ring buffer
      |
      v
DSP thread (medium priority)
      |   1. gravity removal (high pass filter)
      |   2. magnitude vector, orientation independent
      |   3. Hann window
      |   4. real FFT (CMSIS-DSP)
      |   5. feature extraction: peak frequency, band power,
      |      broadband power, spectral entropy
      |   6. activity gate, rejects voluntary motion
      |   7. per window classification
      v
episode state machine (hysteresis)
      |   idle -> candidate (K of N windows) -> episode -> cooldown
      |   record: start time, duration, dominant frequency, severity
      v
BLE thread (low priority)
      GATT notify: live severity and episode events
```

The interrupt handler does no I2C work. It only signals a semaphore. Zephyr's I2C transfer API is blocking and can only be called from thread context, so a dedicated acquisition thread does the actual burst read once it wakes up. This keeps the CPU idle between watermark events and avoids doing any bus work in interrupt context.

## Key design decisions

**Custom, out of tree LSM6DSL driver instead of Zephyr's built in one.** Writing the driver is a core goal of the project: devicetree bindings, Kconfig, the Zephyr driver model, and interrupt to thread handoff.

**FIFO plus watermark interrupt instead of per sample interrupts or polling.** The IMU batches samples in hardware, so the CPU wakes once per batch of samples instead of once per sample, which keeps the interrupt rate low and leaves room for low power operation later.

**Magnitude vector instead of full orientation fusion in the first version.** The magnitude of the acceleration vector is invariant to wrist rotation, which is enough for tremor and dyskinesia detection without needing a quaternion based orientation estimate. Orientation fusion is a planned addition that enables axis specific analysis, such as distinguishing rest tremor from postural tremor.

**Activity gate before classification.** Ordinary daily movement such as walking or reaching produces energy in the same frequency range as tremor. Rejecting windows with high broadband energy or high low frequency energy before classification keeps the false positive rate down.

**Episode state machine with hysteresis instead of per window decisions.** A single noisy classification window is not a diagnosis. Requiring several consecutive positive windows to enter an episode, and a cooldown period to exit one, converts noisy per window output into stable episode records with a start time, duration, dominant frequency, and severity.

**Frequency bands are configurable through Kconfig rather than hardcoded.** The original classroom assignment used 3 to 5 Hz for tremor and 5 to 7 Hz for dyskinesia. Clinical literature describes a broader tremor range, roughly 3 to 9 Hz centered around 4 to 6 Hz, and describes dyskinesia as less rhythmic rather than simply higher frequency. That argues for separating the two using a regularity measure such as spectral entropy or peak sharpness, in addition to band power.

**Secure firmware update is treated as a property of the finished product, not a separate demo.** The interesting engineering problems, such as partial writes, power loss during an update, rollback, and key management, only matter once there is a real application being updated.

**Modularity and portability are treated as hard requirements.** Board specific and sensor specific details are kept behind devicetree, Kconfig, and a stable driver API, so switching to a different board or a different IMU part should mean changing configuration and swapping a driver, not rewriting the signal processing, state machine, or BLE application logic.

## Roadmap

1. Board bring up and boot sequence understanding.
2. Custom LSM6DSL driver: devicetree binding, FIFO and interrupt handling, interrupt to thread handoff.
3. DSP pipeline: windowing, FFT, feature extraction, activity gate, validation against known motion.
4. Episode state machine and a BLE GATT service for severity and episode notifications.
5. MCUboot based secure firmware update, over UART first and then BLE, including rollback and anti rollback testing.

Stretch goals: orientation fusion for axis specific analysis, local data logging to flash, power management and current draw measurement, continuous integration.

## Current status

- [x] Toolchain and board bring up, blink and log output confirmed
- [x] I2C communication with the LSM6DSL confirmed by reading the WHO_AM_I register
- [ ] Custom LSM6DSL driver
- [ ] DSP pipeline
- [ ] Episode state machine and BLE service
- [ ] MCUboot secure update

## Building and flashing

This project uses the Zephyr RTOS build system: west, CMake, devicetree, and Kconfig.

```
west build -b disco_l475_iot1
west flash --runner openocd
```

## Repository layout

```
tremor_detection/
  CMakeLists.txt
  prj.conf
  boards/        board specific overlays
  drivers/        out of tree LSM6DSL driver (planned)
  src/
    main.c
    acquisition.c   (planned)
    dsp/            windowing, FFT, feature extraction (planned)
    episode_fsm.c    (planned)
    ble_service.c    (planned)
  tests/          unit tests on recorded sensor data (planned)
  docs/           reference material and datasheets
```

## License

MIT. See `LICENSE`.
