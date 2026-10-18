# FSR calibration — 2026-10-13

The replacement pads were calibrated on the xArm6 with the wellplate on the base.
Both sensors independently rose above 1000 ADC counts when pressed by hand.
Their unloaded readings varied with jaw position and mounting preload, so each
grasp measures an unloaded median instead of treating raw readings as pressure.
These thresholds are ADC counts above that baseline, not force in newtons.

`config/visual_servo.yaml` enables both FSRs. Contact thresholds are `[60, 60]`;
grip thresholds are `[150, 150]`. The controller takes a median of five readings,
tares over 25 readings, and closes in steps of 10. Once both sensors reach grip
pressure, it settles the main grip for four seconds, then reads the current
gripper command, adds **40**, verifies the register readback, waits one second
for that final adjustment, and lifts. No other gripper action intervenes between
this increment and the lift. The closing command remains bounded by
`gripper.max_close` and the Arduino's force cutoff.

The status transitions through `grip_settling`, `squeezing`, then `lifting`.
`gripper_before_squeeze`, `gripper_after_squeeze`, `gripper_squeeze_delta`, and
`squeeze_command_verified` expose the command confirmation. The log prints
`PRE-LIFT SQUEEZE: gripper ... -> ... (+40), register readback ...`.
These values confirm the commanded setpoint; the Arduino's force cutoff can
still stop physical jaw travel before it reaches that setpoint. Unconfirmed FSR
contact or a failed squeeze/readback prevents the grasp lift.

Each start command makes one attempt (`grasp.max_retries: 0`). After both pads confirm the grasp, the post-lift check requires retained
pressure on at least one enabled pad: lifting can unload the other pad even
while the plate remains held. Loss of pressure on both pads fails the check.
`grasp.return_to_start: false` keeps the arm raised after lifting. A failed post-lift
check ends the run without another alignment or opening the gripper. Completion,
failure and stop retain the last gripper command. The initial preparation for a
new start command still opens the gripper before alignment and tare.

When one pad touches, or both touch but only one exceeds the grip threshold, the
arm nudges to unload the active jaw while keeping the gripper opening fixed.
If contact switches to the opposite pad while still unbalanced, it takes up a
quarter closing step (2 units with the configured step of 10) and re-reads the
sensors. This prevents alternating contact at a fixed opening. That adjustment
is permitted only when both readings are at or below the firmware force cutoff;
If the same pad stays active, a quarter step is also permitted after a nudge
measurably reduces its pressure (at least 20 ADC counts and 10% of its prior
pressure). A nudge that does not unload the pad never triggers this adjustment.
Both sensors must still reach grip pressure before the final +40 and lift.
Each nudge is 0.5 mm at 5 mm/s; displacement is bounded to 5 mm and ten nudges.
`gripper.fsr_nudge_sign: [1, -1]` maps the pads to the flange finger axis. An
on-arm direction check measured FSR1 at 761 ADC before moving, 0 after +1 mm,
and 910 after -1 mm. The opposing jaw uses the opposite direction.

## Hardware validation

The revised controller grasped on its first attempt after five nudges, totalling
2.5 mm, at gripper setpoint 180. The unloaded baseline was `[0, 86]`.

| Stage | FSR1 ADC | FSR2 ADC |
| --- | ---: | ---: |
| Balanced contact | 632 | 415 |
| Immediately after the controller's 20 mm lift | 546 | 374 |
| Held later at 20 mm, median of 40 reads | 433.5 | 389 |
| Held at 100 mm, median of 40 reads | 427.5 | 387 |
| Minimum during the 100 mm hold | 423 | 383 |

Both pads stayed above their grip thresholds. The additional lift from 20 to
100 mm was performed at 10 mm/s. The plate was then lowered back to the base,
released, and the arm returned to the hover pose. The successful run and full
lift had no Modbus errors. Those initial hardware measurements used `final_squeeze: 0`.

The updated pre-lift timing was also tested on the arm: after the four-second
grip settling phase, the command changed **180 -> 200 (+20)** and register
readback confirmed 200 before entering `lifting`. The normal 100 mm lift at
50 mm/s succeeded on the first attempt, with FSR readings `[526, 577]` above
the `[0, 77]` tare. The run ended as `grasped`, returned to its starting pose,
and retained gripper command 200 without opening. A preceding attempt had
stopped at the nudge limit when contact alternated between pads; the bounded
quarter-step slack adjustment handles that case. All 66 tests passed.
The package's tests cover nudge
direction, holding the opening during balancing, partial contact on both pads,
noise rejection, failed reads, and bounded correction. The robot-container
package was rebuilt.

## Arduino firmware

The Nano with the old bootloader was compiled and flashed with verification:

```bash
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old \
  --build-path /tmp/autolab-fsr-build \
  /home/labx/gripper_debug/Gripper_code/RS485_copy
arduino-cli upload --fqbn arduino:avr:nano:cpu=atmega328old \
  --port /dev/ttyUSB0 --input-dir /tmp/autolab-fsr-build --verify \
  /home/labx/gripper_debug/Gripper_code/RS485_copy
```

The firmware keeps the 600-count closing force cutoff and always permits
opening under load. It discards the first ADC conversion after switching
channels and takes a five-sample median. Registers `0x0702/0x0703` report the
filtered ADC readings; `0x0704/0x0705` expose unfiltered diagnostics;
`0x0707` reports firmware revision 3. Existing setpoint and FSR registers retain
their addresses.

The tool RS-485 baud rate is now **57600**, matched in `visual_servoing/arm.py`,
with a 100 ms controller timeout. The earlier 115200-baud SoftwareSerial link
occasionally latched xArm C19 and prevented a lift. Other clients that use this
gripper must also select 57600. The USB debug baud rate remains 115200.

Both sketch copies in `/home/labx/gripper_debug/Gripper_code` are synchronized.
The original sketch files, including the pre-existing local edits, and a
readback of the original flash are preserved in
`/home/labx/gripper_debug/calibration_backups/2026-10-13`.

## +40 debugging

The failed +40 run stopped before squeezing because FSR1 remained unloaded
and the ten-nudge limit was reached. The corrected loop measures each nudge's
pressure response and permits a two-unit take-up step after successful unloading
under the firmware cutoff. It still requires both pads before the final squeeze.
A monitored run then verified **180 -> 220 (+40)** and raised the arm **100 mm**.
After the lift, readings were **0 / 716** with baseline **0 / 79**; the operator
confirmed that the plate was held above the base. This exposed a false negative
in the old two-pad post-lift check. The revised retention check accepts either
loaded pad after the initial two-pad confirmation.
