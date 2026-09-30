# Arduino hardware test sketches

Open each `.ino` in its own same-named folder with Arduino IDE. Select
**Arduino Mega or Mega 2560** and the correct port, upload **one sketch at a
time**, and open Serial Monitor at **115200 baud**. The pins target the Mega
2560 (ATmega2560). The G25 encoder uses its interrupt-capable D2 and D3 pins.

| Sketch | Tests | Main settings |
| --- | --- | --- |
| `l298n_two_motors` | Motor A and B independently, forward and reverse at three PWM levels, then together | `MOTOR_A`, `MOTOR_B`, `TEST_PWM`, `STEP_MS`, `REST_MS` |
| `g25_motor_encoder` | One G25 DC motor forward and reverse at three PWM levels, with A/B quadrature counts | Motor/encoder pins, `TEST_PWM`, `RUN_MS`, `SAMPLE_MS`, `ENCODER_EDGES_PER_OUTPUT_REV` |
| `servo_25kg_position` | Move an **angle-controlled** servo through a limited sweep | `MIN_ANGLE`, `MAX_ANGLE`, `STEP_DEGREES`, timing |
| `servo_25kg_continuous` | Run a **continuous-rotation** servo like a motor: forward, stop, reverse | `STOP_US`, `FORWARD_US`, `REVERSE_US`, timing |

All adjustable parameters and functions are described inside each sketch.

## Before uploading

- Check motor and servo model voltage/current ratings. Use a suitable external
  supply for motors and the 25 kg servo, with a common ground to Arduino.
- For PWM on an L298N, remove the jumper from the channel's `EN` pin. Check
  the driver can handle the motor's actual current, including startup/stall.
- For the G25 encoder, confirm its supply voltage and output logic level.
  `ENCODER_EDGES_PER_OUTPUT_REV = 0` intentionally reports raw ticks only;
  enter the measured or specified geared output-shaft value to display RPM.
- A 25 kg torque rating alone does **not** identify whether a servo uses
  position control or continuous rotation. Use the sketch that matches the
  exact model. A normal position servo cannot be made to spin continuously
  through Arduino code alone.

The G25 test changes the **PWM command** and checks encoder response. It does
not measure watts, current, torque, or mechanical output power.
