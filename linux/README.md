python pc_pd_poc.py --port COM4

python3 pc_pd_poc.py --port /dev/ttyACM0

CLEAR
ZERO
POWER ON
ARM

GAINS 0.30 0.004 0.05
PULSE 2 0.02

# 0.5 A
GAINS 0.15 0.003 0.080

# 2.0 A
GAINS 0.80 0.010 0.12

SET 0 0 0


## PC commands

Commands are ASCII, newline terminated.

```text
ZERO                         capture the current all-vertical pose; SAFE only
POWER ON                     explicitly energise Board02 controlled motor outputs
POWER OFF                    zero/IDLE then cut the Board02 controlled outputs
ARM                          enter torque PD; only after ZERO and healthy data
DISARM                       immediate zero torque + ODrive IDLE request
CLEAR                        leave a latched fault; no motor motion
KEEP                         refresh the 250 ms PC watchdog
SET hip knee ankle            desired output angles in degrees, each +/-10 deg
GAINS kp kd torque_cap        shared outer gains; cap cannot exceed 0.010 Nm
PULSE joint torque            +/-0.002 Nm max for 200 ms; direction calibration
```

`POWER ON` / `POWER OFF` apply when your motor 24 V goes through the two
controlled XT30 outputs.  They use `PC13` and `PC14`, matching the supplied
starting project.  If the Mini ODrives have their own permanent 24 V source,
set `POC_USE_BOARD_MOTOR_POWER` to `0` in `leg_pd_poc.c`; then `ARM` does not
require the `POWER ON` command.

The PC program maintains `KEEP` at 50 Hz after `ARM`.  Closing it or losing
the USB serial link disarms the H723 within 250 ms.

## Safe bring-up order

1. **No mechanics moving:** connect H723 to the PC and G030.  Do not connect
   motor power.  Confirm status lines show `mode=SAFE`, `flags=0E/0F`, three
   changing output angles, and `gage` below 10 ms. `pwr=0` is expected.
2. **CAN telemetry only:** with the leg mechanically supported, issue
   `POWER ON` if the motors use the Board02 controlled outputs; otherwise
   power their external supply. Do not issue ARM. Confirm `pwr=1`,
   `axis=[1,1,1]` or your known idle states, and all error words are zero.
   If nodes are not 1,2,3, change `k_node_id[]` before proceeding.
3. Put the three output links in the upright mechanical pose and run `ZERO`.
   Confirm `q=[0,0,0]` while still SAFE.
4. Mechanically support the leg, set `GAINS 0 0 0.002`, then `ARM`.  This only
   enters torque mode with zero commanded torque.
5. Run `PULSE 0 0.002`.  A positive pulse must make reported `q[0]` increase.
   If it decreases, disarm, change that element of `k_motor_torque_sign[]` to
   `-1.0f`, rebuild, and repeat.  Perform joints 1 and 2 one at a time.
6. With all signs correct: `GAINS 0.020 0.0008 0.006`, `ARM`, then command
   only `SET 1 0 0`, then return to `SET 0 0 0`.  Do one joint at a time.
7. End every bench session with `DISARM`, then `POWER OFF` if using the
   Board02 controlled outputs. Only after all three one-degree tests are
   correct should you use up to
   `SET 5 5 5` or increase torque cap toward 0.010 Nm.

Never test a new motor direction with a 5-degree command.  Keep the leg on
the vertical guide, keep a hand on the hardware emergency stop, and do not
save any ODrive configuration from this POC.
