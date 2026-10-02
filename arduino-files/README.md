# Arduino hardware test sketches

Open each `.ino` in its own same-named folder with Arduino IDE. Select
**Arduino Mega or Mega 2560** and the correct port, upload **one sketch at a
time**, and open Serial Monitor at **115200 baud**. The pins target the Mega
2560 (ATmega2560). The G25 encoder uses its interrupt-capable D2 and D3 pins.

| Sketch | Tests | Main settings |
| --- | --- | --- |
| `l298n_two_motors` | Motor A and B independently, forward and reverse at three PWM levels, then together | `MOTOR_A`, `MOTOR_B`, `TEST_PWM`, `STEP_MS`, `REST_MS` |
| `l298n_two_motors_no_pwm` | Motor A and B independently, then together, using full on/off drive with no PWM | `MOTOR_A`, `MOTOR_B`, `RUN_MS`, `REST_MS` |
| `module_1_semi_auto` | Four regular DC motors on two L298Ns, five IR sensors, button status, two command-controlled servos, and a stoppable photo-position sequence | Motor/sensor pins, stage timeouts, 20-second and 2-second run constants |
| `module_1_semi_auto_v2` | Button-led four-position photo sequence, picture rotator, cutter cycle after position 3, and a timed final roll | Pins, button/LED intervals, sensor timeouts, servo/cutter timing |
| `module_1_semi_auto_v3` | Automatic picture rotation after each roll IR, then button-triggered five-second photo countdown | `ROLL_TO_ROTATION_DELAY_MS`, `PHOTO_COUNTDOWN_SECONDS`, LED blink and servo timing |
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
  `SERVO-1-1500`/`SERVO-2-1500` provide manual checks. The fourth motor is
  a regular DC motor and is manual-only for now. The button and
  `ir_rotation_trigger` are reported but have no automatic action yet.
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
| L298N board 2: `roll_3` / `motor_4` | IN1/IN2 = D24/D25; IN3/IN4 = D26/D27 |
| IR `ir_1`, `ir_2`, `ir_3`, `ir_4`, `ir_rotation_trigger` | D30, D31, D32, D33, D34 |
| Button | D35 to GND when pressed |
| Servo 1 / Servo 2 signal | D40 / D41 |

Sensor inputs use `INPUT_PULLUP` and assume LOW means triggered, consistent
with the normally high NPN output in an [E18-D80NK specification](https://www.gie.com.my/download/spec/sensors/range/e18_d80nk.pdf).
Confirm the pinout and output behavior of your actual sensors before motion.
The button and `ir_rotation_trigger` are displayed by `STATUS`/`SENSORS` but
do not start or stop automation. `motor_4` remains manual only and can be
controlled with `MOTOR-4-ON` and `MOTOR-4-OFF`.

After `E` and `AUTO`, the controller runs these stages:

1. `roll_1` + `roll_2` until `ir_1`, stop, call `TAKE_PICTURE_CYCLE()`.
2. `roll_1` + `roll_2` until `ir_2`, stop, call the same function.
3. `roll_2` + `roll_3` until `ir_3`, stop, call the empty
   `ACTIVATE_CONTINUOUS_SERVO()` hook once, then call `TAKE_PICTURE_CYCLE()`.
4. `roll_2` + `roll_3` until `ir_4`, stop, call `TAKE_PICTURE_CYCLE()`.
5. `roll_2` + `roll_3` for `FINAL_ROLL_2_3_RUN_MS` (20,000 ms), stop, call
   the empty `MODULE_1_STOP()` function.
6. `roll_1` for `FINAL_ROLL_1_RUN_MS` (2,000 ms), then stop.

`STOP` or `D` cancels the sequence and brakes all four motors. Sensor stages
also stop if their sensor does not trigger within `SENSOR_WAIT_TIMEOUT_MS`
(60,000 ms by default; set it to 0 to disable the timeout). The picture,
continuous-servo, and module-stop functions are deliberately empty until
their actions are tested.
The two servos accept manual pulse commands within 1200–1800 microseconds;
check the servo models before expanding that range.

## Module 1 v2: button, picture, and cutting automation

Upload `module_1_semi_auto_v2.ino` from its same-named folder. The v1 sketch
remains separate. V2 requires the ENA/ENB jumpers on both L298N boards and
uses these Mega 2560 pins:

| Device | Pins |
| --- | --- |
| L298N board 1: `roll_1`, `roll_2` | IN1/IN2 = D13/D15; IN3/IN4 = D9/D11 |
| L298N board 2: `roll_3`, `motor_4` picture rotator | IN1/IN2 = D19/D17; IN3/IN4 = D5/D7 |
| Roll IR sensors `ir_1`, `ir_2`, `ir_3`, `ir_4` | D27, D23, D43, D25 |
| Picture-cycle E18 / cutting-cycle E18 | D29 / D45 |
| Button / ButtonLED | D41 to GND / D39 (HIGH = on) |
| Picture position servo / continuous cutting servo | D40 / D31 |

The button pin and LED pin come from `button.ino`. `motor_4` and the separate
D29 picture E18 perform the rotation and position detection from
`taking_picture_cycle.ino`. The cutting E18 from `cutting_cycle.ino` uses D45
in v2 because `ir_3` uses D43. All pin changes are in the v2 sketch; the
source cycle sketches are unchanged. All IR inputs
assume LOW means triggered; confirm this with your actual sensor outputs.

1. With the LED on, press the button. A 350 ms start cue keeps the LED on;
   then `roll_1` and `roll_2` run with LED off until `ir_1` triggers.
2. At each `ir_1` through `ir_4`, the roll motors stop and the LED stays on.
   Press the button once to run one picture cycle. While that cycle runs, the
   LED blinks every `PHOTO_LED_BLINK_MS` (300 ms by default).
3. Each picture cycle turns `motor_4` until `picture_cycle_e18` goes clear
   and then triggers. It stops the rotator, waits 500 ms, moves the position
   servo to 40 degrees for 700 ms, then returns it to 90 degrees.
4. After the first photo, `roll_1` and `roll_2` run to `ir_2`. After the second,
   `roll_2` and `roll_3` run to `ir_3`.
5. After the third photo, the controller waits for the separate cutter IR on
   D45 (`CUT_WAIT_FOR_SENSOR = true`). It runs the cutting servo forward for
   1800 ms, neutral for 250 ms,
   reverse for 1700 ms, then neutral. Each completed cut adds 25 ms to the
   next forward duration and 100 ms to the next reverse duration. Then
   `roll_2` and `roll_3` run to `ir_4`.
6. After the fourth photo, `roll_2` and `roll_3` run for `FINAL_RUN_MS`
   (10,000 ms). The LED blinks at `FINAL_LED_BLINK_MS` (1,000 ms by default)
   during this final run. All motors stop afterward, the LED turns on, and the
   button can start another full run.

`STOP` cancels any phase, brakes all four DC motors, neutralizes the cutting
servo, and restores the position servo to 90 degrees. `D` also disables
button start until `E` is sent. `AUTO` starts the full run from Serial as an
alternative to the button. `STATUS` and `SENSORS` report progress and inputs.
The sensor-driven roll, picture rotation, and cutter wait phases each have a
60-second timeout by default. Set the corresponding timeout constant to 0 to
disable it. Use suitable external supplies and a common ground.
Set `CUT_WAIT_FOR_SENSOR` to `false` to run the cut immediately after the
third picture instead of waiting for D45.
The picture and cutting E18 inputs use `INPUT`, matching their standalone
sketches; each output must provide a defined HIGH level when clear. V2 embeds
the two cycle mechanisms, so changes to either standalone sketch must also
be copied into V2.

## Module 1 v3: automatic rotation and button photo countdown

Upload `module_1_semi_auto_v3.ino` from its same-named folder. V2 remains
available separately. V3 uses these pins from the updated v2 sketch:
roll motors D13/D15, D9/D11, D19/D17; picture rotator `motor_4` D5/D7;
roll IRs D27/D23/D43/D25; picture E18 D29; button D41; ButtonLED D39;
position servo D21; cutting servo D31. V3 reports the cutting E18 on D45 to
avoid the D43 conflict with `ir_3`; the cut still starts immediately after the
third picture. The MAX7219 uses DIN D33, CLK D37, and CS D35 and requires the
`LedControl` library.

At each of the four roll IRs, v3 stops the roll motors after a 30 ms stable
trigger, waits 800 ms, then runs the picture rotation routine until the picture
E18 clears and triggers again. Rotation then stops and the LED stays on while
waiting for a button press. That press starts a five-second serial countdown;
the LED blinks every 300 ms throughout it. The MAX7219 wakes at minimum
intensity and displays 5, 4, 3, 2, 1 on its rightmost digit. It shuts down
before the picture servo moves from 90 to 40 degrees. The servo holds for one
second, then returns to 90 degrees. The display is also shut down at startup
and whenever `STOP` or `D` cancels the run.

After the third picture, the position-controlled cutting servo on D31 moves
from its 40-degree home angle to `CUT_ACTION_ANGLE` (90 degrees by default),
waits 1800 ms for travel, holds another 250 ms, then returns to 40 degrees
and waits 1700 ms before the next roll. Adjust `CUT_ACTION_ANGLE` for the
installed linkage. `SERVO-2-<angle>` accepts manual 0..180 degree commands
while READY; `SERVO-1-<pulse>` still controls the picture servo in microseconds.
The final 10-second roll remains in place. The new cutter travel and hold waits
use `millis()`, so `STOP` can cancel the cutting cycle.

The current v3 rotation routine directly writes D9/D11, while the motor table
assigns those pins to `roll_2` and assigns D5/D7 to `motor_4`. Verify that
wiring before uploading; this cutter change leaves the rotation routine intact.
