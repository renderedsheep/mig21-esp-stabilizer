# MiG-21 ESP32 Flight Stabilizer

A DIY gyro stabilizer for a **1:10 scale R/C MiG-21 EDF jet**, built on an ESP32 and an MPU-6050 motion sensor.

> [!CAUTION]
> **Status: work in progress, UNTESTED.**
> This experimental hobby software provided as is, with no warranty. You are responsible for testing it and for flying safely and legally. This code has never been run on hardware and has never flown. It has only been syntax-checked on a desktop, and its radio-frame parser has been tested against simulated data. Do not trust it with an aircraft you cannot afford to lose. Every direction setting in the sketch is an unverified guess until you complete the bench tests below. 

## What it does

The ESP32 sits between the receiver and three control surface servos (aileron, elevator, rudder). It reads your stick commands, measures how the plane is rotating, and adds small corrections on top of your commands.

Throttle is **not** routed through the ESP32. The ESC plugs straight into the receiver, so a crash or freeze of the board cannot run the fan up.

### Modes (switch on the transmitter, channel 5)

| Mode | What happens |
|------|--------------|
| OFF | Stick commands pass straight to the servos. Use this for the first flights and as your escape hatch. |
| RATE | Gyro damping. The board pushes against unwanted rotation (wobble, gusts). Strength is set by the channel 6 knob. |
| LEVEL | Self-leveling. Stick position sets a target bank and pitch angle, and the board steers toward it. Meant as a "get me out of trouble" mode, not for aerobatics. |

### How it works

1. **Radio input.** The FS-iA10B receiver sends all channels over a single wire using the iBUS protocol (32-byte frames at 115200 baud). The sketch checks each frame's checksum and re-synchronizes after corrupted data.
2. **Sensing.** The MPU-6050 reports rotation rates (gyro) and gravity direction (accelerometer) over I2C. A low-pass filter on the chip reduces fan vibration.
3. **Attitude estimate.** Roll and pitch angles come from a complementary filter. The gyro is integrated for fast response, and the accelerometer slowly pulls the estimate back to correct drift. The accelerometer is ignored whenever total acceleration differs noticeably from 1 g (for example while pulling in a hard turn).
4. **Control.**
   - RATE: output = stick minus (gain x rotation rate). The correction fades as stick deflection grows, so the board never fights your deliberate inputs.
   - LEVEL: output = stick feed-forward plus a correction proportional to the angle error, minus rate damping.
5. **Servo output.** Three 50 Hz signals with 16-bit resolution. Control loop runs at 250 Hz.
6. **Safety behavior.**
   - Radio signal lost after having had one: LEVEL mode with sticks neutral.
   - Sensor not found, fails calibration, or stops responding: pass-through (acts like a plain wire). Do not fly in this state.
   - Calibration needs the plane perfectly still at power-up and sitting level on its gear. The board uses that pose as "level."

Status LED (GPIO2): solid means all good, slow blink means no radio signal, fast blink means sensor fault.

## Hardware

- ESP32 DevKit V1 (ESP32-WROOM-32). Must be the classic ESP32, not an S2, S3 or C3.
- MPU-6050 on a GY-521 breakout board
- FlySky FS-i6X transmitter and FS-iA10B receiver
- Three servos (aileron, elevator, rudder). Two ailerons share one pin with a Y-lead.
- Brushless EDF motor and ESC, wired directly to the receiver

**Assumption:** a conventional layout with separate aileron, elevator and rudder servos. If your model uses elevons, the sketch needs a mixer added first (not implemented).

### Wiring

| From | To | Notes |
|------|----|-------|
| Receiver iBUS SERVO port, signal | ESP32 GPIO16 | Check with a multimeter that the signal is about 3.3 V. If it reads above 3.6 V, use a 1k/2k resistor divider. |
| Receiver iBUS SERVO port, + and - | ESP32 VIN and GND | 5 V supply for the board |
| MPU-6050 VCC / GND | ESP32 3V3 / GND | |
| MPU-6050 SDA / SCL | ESP32 GPIO21 / GPIO22 | Keep wires short |
| Aileron servo signal | GPIO25 | |
| Elevator servo signal | GPIO26 | |
| Rudder servo signal | GPIO27 | |
| Servo power (+ and -) | A spare receiver port | The ESP32 cannot power servos. Only signal wires go to the ESP32. |
| ESC | Receiver CH3 | Throttle bypasses the ESP32 |

Mount the MPU-6050 flat, chip side up, X arrow pointing to the nose, near the center of gravity, on foam tape to soak up vibration. All grounds must be shared.

### Transmitter setup (FS-i6X)

1. Bind in AFHDS 2A mode.
2. Set the receiver output mode to PWM + i-BUS.
3. Channel 5 on SwC (mode switch), channel 6 on VrA (gain knob).
4. Servo reversing off on channels 1, 2 and 4. Reversing is done in the code.
5. No transmitter mixes on those three channels.
6. Failsafe: channel 3 to low throttle, channel 5 to the LEVEL position. The default FlySky failsafe holds the last throttle position, which is dangerous for an EDF jet.

## Building and uploading

1. Install the Arduino IDE.
2. In Boards Manager, install **esp32 by Espressif Systems**, version 3.x. Version 2.x will not compile this sketch.
3. Open `MiG21_Stabilizer/MiG21_Stabilizer.ino`, select **ESP32 Dev Module**, and upload.
4. Open Serial Monitor at 115200 baud to see live values.

## Bench tests (mandatory before any flight)

A wrong direction setting makes the stabilizer push the wrong way, and the plane will crash. Settings to flip are at the top of the sketch.

**Test A: sticks and sensor, over USB.** Disconnect the ESC and servo power. Do not power the board from USB and from the receiver's 5 V at the same time unless your DevKit has a protection diode. Safest is to unplug the VIN wire while on USB and power the receiver from a flight battery.

1. Stick right shows R positive. Stick back shows P positive. Rudder right shows Y positive. If not, change the `STICK_REV_` setting.
2. Right wing down makes roll and rr positive. Nose up makes pitch and pr positive. Nose swung right makes yr positive. If not, change the matching `ACC_` or `GYRO_` sign.

**Test B: servos, on flight battery, USB unplugged, motor wires disconnected from the ESC.**

1. In OFF mode, every surface follows the sticks correctly. If not, change the `SERVO_REV_` setting.
2. In RATE mode with the knob at maximum, check the corrections:
   - Quickly drop the right wing: ailerons move as if you pushed the stick left.
   - Quickly raise the nose: elevator moves as if you pushed the stick forward.
   - Quickly swing the nose right: rudder moves left.
3. If any correction is backwards, do not fly.

## First flights

1. Fly in OFF mode first and trim the plane.
2. Switch to RATE at a safe height with the knob at about one third.
3. If surfaces start twitching or wagging, especially on a fast pass, switch to OFF at once, then lower the `DAMP_` values.

## Known limitations

- **No real flight data yet.** All tuning values are starting guesses.
- **Gains do not adjust for airspeed.** Settings that feel right at half throttle may oscillate at full speed.
- **LEVEL mode drifts in sustained banked turns.** Without GPS or an airspeed sensor, the accelerometer cannot tell gravity from turning force.
- **Simplified attitude math.** Roll and pitch are treated as independent angles, which loses accuracy at steep bank or pitch.
- **Basic sensor and filtering.** The MPU-6050 is an older, noisy part, and there is only one on-chip low-pass filter. Fan vibration is the most likely source of trouble.
- **250 Hz loop over I2C.** Slower than dedicated flight controllers.
- **Not compiled against the real ESP32 toolchain yet.** The first upload is the real compile test.

If protecting the airframe matters more than building the stabilizer yourself, a ready-made fixed-wing flight controller running INAV is the safer choice.

## Roadmap

- [ ] First real compile and upload
- [ ] Bench tests A and B
- [ ] Elevon mixer option
- [ ] Vibration notch filter for the EDF fan
- [ ] Airspeed-based gain scheduling
- [ ] First flight log and retuned defaults

## License

Copyright (C) 2026 Fernand G.

Licensed under the GNU General Public License, version 3 or (at your option) any later version. See the [LICENSE](LICENSE) file. In short: you may use, modify and redistribute this code, including commercially, but you must keep the copyright notices and release any distributed modifications under the same license.

