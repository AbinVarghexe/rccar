# Self-Driving Car — One-Day Build Sheet

This is the fastest build path for the hardware listed in the reference
project at https://github.com/sanoguzhan/Self-Driving-Car.

## Architecture to use

Use the Arduino UNO as the motor-control board and the Raspberry Pi as the
camera/vision computer:

```text
Pi CSI camera -> Raspberry Pi/OpenCV -> USB serial -> Arduino UNO
                                              |
                                              v
                                      L298N -> motors
```

Do not use the local `rccar` direct-Pi GPIO motor map for this first build.
That is a different architecture and its extra servo/ultrasonic features are
out of scope for the one-day reference demo.

## Wiring

Disconnect every battery or USB power source while wiring. Use the labels on
your L298N board; some boards print `12V`, `VCC`, or `Vs` for motor supply.

### Arduino UNO to L298N

These are the pin assignments used by the reference project:

| Arduino | L298N | Purpose |
|---|---|---|
| D5 (PWM) | ENA | Left-channel speed |
| D7 | IN1 | Left-channel direction |
| D11 | IN2 | Left-channel direction |
| D6 (PWM) | ENB | Right-channel speed |
| D9 | IN3 | Right-channel direction |
| D8 | IN4 | Right-channel direction |
| GND | GND | Common signal ground |

### Wire-color reference

The current motor-driver input wire colors are recorded as follows; orange is
the corrected color for Channel 1:

| L298N channel | Wire color | Arduino pin |
|---|---|---:|
| IN1 / Channel 1 | Orange | D7 |
| IN2 / Channel 2 | Red | D11 |
| IN3 / Channel 3 | Brown | D9 |
| IN4 / Channel 4 | Black | D8 |

Remove the ENA and ENB jumper caps if the Arduino is controlling speed with
D5/D6. If you leave the caps installed, leave the enable wires disconnected
and use fixed full speed until the basic motor test works.

### L298N power and motors

| L298N terminal | Connect to |
|---|---|
| `12V`/`Vs`/motor `+` | Motor battery positive |
| `GND` | Motor battery negative and Arduino GND |
| `OUT1`, `OUT2` | Left motor pair |
| `OUT3`, `OUT4` | Right motor pair |
| `5V`/`Vss` logic input | Only as specified by your exact L298N board |

For a 4-wheel chassis, the two motors on the same side are normally wired in
parallel to one output channel. Check the driver current rating first; if the
motors stall or the driver overheats, use a higher-current dual-channel driver
or one driver per motor pair.

Do not power the motors from the Arduino 5V pin, Raspberry Pi 5V pin, or USB
port. Use a regulated supply for the Arduino if the motor battery is outside
the UNO input range.

### Raspberry Pi, camera, and Arduino

- Insert the CSI ribbon with power removed; verify the contact orientation for
  your Pi and Camera Module V2.
- Connect the Arduino to the Pi with USB. This provides the serial link and a
  common ground; do not connect Pi GPIO directly to the L298N in this setup.
- Power the Pi from the power bank through USB-C.
- Power the motor driver from the motor battery.
- Keep motor wires away from the camera ribbon and Pi power cable.

### Safety checks before power

1. Wheels off the table.
2. Physical battery disconnect reachable.
3. No loose wire can touch an adjacent terminal.
4. Arduino GND and L298N GND are connected.
5. ENA/ENB configuration matches the firmware.
6. Never connect a 5 V Arduino signal to a Pi GPIO.

## Software scope for the one-day deadline

### Block 1 — first 90 minutes: make the platform move and see

1. Install current Raspberry Pi OS and enable SSH.
2. Verify the camera before writing car logic:

   ```bash
   sudo apt update
   sudo apt install -y python3-picamera2 python3-opencv python3-serial
   rpicam-hello
   ```

3. Upload a minimal Arduino motor firmware and test `STOP`, `FORWARD`,
   `BACKWARD`, `LEFT`, and `RIGHT` with the wheels raised.
4. From the Pi, find the Arduino device (`/dev/ttyACM0` is typical), send one
   command at a time, and confirm the Arduino RX/TX LEDs flash.
5. Run the camera capture and line-detection pipeline at 320x240 or 400x240.
6. Tune the region of interest using a high-contrast tape track.

### Block 2 — next 2 hours: integrate the control loop

1. Use the line center error to select `LEFT`, `RIGHT`, or `FORWARD`.
2. Add a serial watchdog: Arduino stops if no valid command arrives for 500 ms.
3. Add a Pi-side emergency stop and make `STOP` the startup/default command.
4. Test at low speed with a person beside the car.
5. Start at the lowest motor speed and test on a wide, high-contrast track.

### Block 3 — final 1–2 hours: stabilize the demo

1. Remove every unused feature from the startup path.
2. Confirm that unplugging USB or stopping the Pi causes an automatic stop.
3. Run the car only in a closed area with a person beside it.
4. Record the demo and document the final pin map and limitations.

If the car does not move reliably after Block 1, stop debugging vision and
finish a reliable manual/serial motor demo. A reliable moving platform is a
better one-day result than a partially integrated detector.

## What to postpone completely today

Do not spend the day on the reference repository's old native
`raspicam`/OpenCV build instructions, road-sign classifier training, cloud AI,
object detection, SLAM, GPS, waypoint navigation, or ultrasonic/servo
expansion. They are not required for the first working line-following
demonstration and will consume the day.

## Acceptance tests

- Camera preview works with `rpicam-hello`.
- Arduino drives each side in both directions with the wheels raised.
- `STOP` stops both sides immediately.
- Disconnecting the Pi USB or stopping the Pi process causes the Arduino to
  stop within the watchdog interval.
- The car follows a simple wide track at low speed for at least one minute.
- A person can remove motor power without reaching over the moving chassis.

## Important reference-project caveat

The reference README describes Arduino pins D5/D7/D11 for the left channel and
D6/D9/D8 for the right channel, plus Raspberry Pi camera/OpenCV processing.
Its Raspberry Pi setup text is from an older software era; on current Raspberry
Pi OS, use the supported `rpicam`/Picamera2 stack and validate the camera first.
