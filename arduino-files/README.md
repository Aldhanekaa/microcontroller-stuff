# Arduino hardware test sketches

Open each `.ino` in its own same-named folder with Arduino IDE. Select
**Arduino Mega or Mega 2560** and the correct port, upload **one sketch at a
time**, and open Serial Monitor at **115200 baud**. The pins target the Mega
2560 (ATmega2560). The G25 encoder uses its interrupt-capable D2 and D3 pins.

| Sketch | Tests | Main settings |
| --- | --- | --- |
| `l298n_two_motors` | Motor A and B independently, forward and reverse at three PWM levels, then together | `MOTOR_A`, `MOTOR_B`, `TEST_PWM`, `STEP_MS`, `REST_MS` |
| `l298n_two_motors_no_pwm` | Motor A and B independently, then together, using full on/off drive with no PWM | `MOTOR_A`, `MOTOR_B`, `RUN_MS`, `REST_MS` |
| `module_1_semi_auto` | Four motors on two L298Ns (fourth has an encoder), five IR sensors, button status, two command-controlled servos, and a stoppable photo-position sequence | Motor/sensor/encoder pins, stage timeouts, 20-second and 2-second run constants |
| `g25_motor_encoder` | One G25 DC motor forward and reverse at three PWM levels, with A/B quadrature counts | Motor/encoder pins, `TEST_PWM`, `RUN_MS`, `SAMPLE_MS`, `ENCODER_EDGES_PER_OUTPUT_REV` |
| `servo_25kg_position` | Move an **angle-controlled** servo through a limited sweep | `MIN_ANGLE`, `MAX_ANGLE`, `STEP_DEGREES`, timing |
| `servo_25kg_continuous` | Run a **continuous-rotation** servo like a motor: forward, stop, reverse | `STOP_US`, `FORWARD_US`, `REVERSE_US`, timing |

All adjustable parameters and functions are described inside each sketch.

## Before uploading

- Check motor and servo model voltage/current ratings. Use a suitable external
  supply for motors and the 25 kg servo, with a common ground to Arduino.
- For PWM on an L298N, remove the jumper from the channel's `EN` pin. Check
  the driver can handle the motor's actual current, including startup/stall.
- For `l298n_two_motors_no_pwm`, leave both `EN` jumpers installed. Only the
  four `IN` pins connect to the Mega; speed is not adjustable in this sketch.
- For `module_1_semi_auto`, leave all four `EN` jumpers installed. Start with
  `E`, then `AUTO`; use `STOP` at any time. `STATUS`, `SENSORS`,
  `MOTOR-1-ON/OFF` through `MOTOR-4-ON/OFF`, `ALL-ON/OFF`, and
  `SERVO-1-1500`/`SERVO-2-1500` provide manual checks. `ENCODER` reports
  the fourth motor's raw encoder ticks. The button and `ir_rotation_trigger`
  are reported but have no automatic action yet.
- For the G25 encoder, confirm its supply voltage and output logic level.
  `ENCODER_EDGES_PER_OUTPUT_REV = 0` intentionally reports raw ticks only;
  enter the measured or specified geared output-shaft value to display RPM.
- A 25 kg torque rating alone does **not** identify whether a servo uses
  position control or continuous rotation. Use the sketch that matches the
  exact model. A normal position servo cannot be made to spin continuously
  through Arduino code alone.

The G25 test changes the **PWM command** and checks encoder response. It does
not measure watts, current, torque, or mechanical output power.

## Module 1 semi-automatic sequence

The `module_1_semi_auto` sketch uses these Mega 2560 pins:

| Device | Pins |
| --- | --- |
| L298N board 1: `roll_1` / `roll_2` | IN1/IN2 = D7/D8; IN3/IN4 = D9/D10 |
| L298N board 2: `roll_3` / `encoder_motor` | IN1/IN2 = D24/D25; IN3/IN4 = D26/D27 |
| `encoder_motor` encoder A/B | D2/D3 |
| IR `ir_1`, `ir_2`, `ir_3`, `roll_4`, `ir_rotation_trigger` | D30, D31, D32, D33, D34 |
| Button | D35 to GND when pressed |
| Servo 1 / Servo 2 signal | D40 / D41 |

Sensor inputs use `INPUT_PULLUP` and assume LOW means triggered, consistent
with the normally high NPN output in an [E18-D80NK specification](https://www.gie.com.my/download/spec/sensors/range/e18_d80nk.pdf).
Confirm the pinout and output behavior of your actual sensors before motion.
The button and `ir_rotation_trigger` are displayed by `STATUS`/`SENSORS` but
do not start or stop automation. The encoder motor is manual only; its raw
quadrature ticks appear in `STATUS` or `ENCODER`.

After `E` and `AUTO`, the controller runs these stages:

1. `roll_1` + `roll_2` until `ir_1`, stop, call `TAKE_PICTURE_CYCLE()`.
2. `roll_1` + `roll_2` until `ir_2`, stop, call the same function.
3. `roll_2` + `roll_3` until `ir_3`, stop, call the same function.
4. `roll_2` + `roll_3` until the sensor named `roll_4`, stop, call the same function.
5. `roll_2` + `roll_3` for `FINAL_ROLL_2_3_RUN_MS` (20,000 ms), stop, call
   the empty `MODULE_1_STOP()` function.
6. `roll_1` for `FINAL_ROLL_1_RUN_MS` (2,000 ms), then stop.

`STOP` or `D` cancels the sequence and brakes all four motors. Sensor stages
also stop if their sensor does not trigger within `SENSOR_WAIT_TIMEOUT_MS`
(60,000 ms by default; set it to 0 to disable the timeout). The picture and
module-stop functions are deliberately empty until their actions are tested.
The two servos accept manual pulse commands within 1200–1800 microseconds;
check the servo models before expanding that range.
