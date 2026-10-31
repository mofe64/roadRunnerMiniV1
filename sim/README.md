# Simulation

This document is written in ASD-STE100 Simplified Technical English.

This folder has the MuJoCo model of Road Runner Mini and the test scripts.

## The model
- `roadrunner_mini.xml` is the model. The script `build_model_v03.py` makes it.
- The model has two layers. The CAD meshes in `meshes/` show the robot (geom group 2). Simple shapes (boxes, capsules, cylinders) do the physics (geom group 3, hidden). Simple shapes make the simulation fast and the contacts stable.
- Each mesh is in the frame of its body at the "stand" pose (leg height 120 mm). The meshes come from Fusion S58. The body shell (torso_shell.stl: tray, deck, two-part chassis, lid) comes from Fusion S63 (version 90). Large meshes are reduced to 15000 triangles or less. The head print and the OAK-D Lite are separate meshes (head.stl, oak.stl).
- To see the physics shapes in the viewer, turn on geom group 3 in the Rendering panel.
- The masses, centres of mass and joint positions come from the Fusion model, v0.2 S58 (2026-10-28). The total mass is 1502 g. In S63 the body is 6.5 g heavier (988.6 g, centre of mass moves 0.2 mm). This is in the random mass range of the training, so the model does not change.
- The model has 4 hip servos, 2 wheel servos, head yaw and head pitch.
- Head pitch limits: 30 degrees up, 50 degrees down. Head yaw limits: 90 degrees left and right (assumed).
- The wheel servos use battery voltage (3S, 11.1 V). Wheel torque decreases as wheel speed increases: 1.85 Nm at 0 rad/s, 0 Nm at 10.3 rad/s.

## Servo model (v3, 2026-10-28)
- The hip and head servos use the STS3215 values that Open Duck Mini measured with BAM (Rhoban): kp 17.8, damping 0.60, friction loss 0.052, armature 0.028.
- Each hip servo and each wheel servo has a backlash joint behind it. The backlash is +/-0.0087 rad (+/-0.5 degrees).
- The wheel servo (ST3215-HS) friction loss (0.022) is NOT measured. It comes from the STS3215 value, scaled for a gear ratio that is approximately 2.4 times lower. The RL training changes it in a wide range (0.3 to 3 times).
- The wheel servo armature (0.004) is measured (spin-up test, 2026-10-29).
- To measure the servos later, use BAM (https://github.com/Rhoban/bam) with a pendulum test bench.

## Install
Use the Python environment in `../rl/.venv` (MuJoCo 3.17).

## Build the model
Run this command:

```
../rl/.venv/bin/python build_model_v03.py 0.120
```

## Tests
- `tests.py` has helper functions and the balance, push, crouch and drive tests.
- `set_wheel_torque()` adds the lost speed torque back, up to the motor limit.
- The `standup_v03*.py` scripts are studies of the get-up movement. They used a wheel with no speed limit. Their results are not valid for this model.

## Results (v3 servo model, hand-tuned balance controller)
- Balance at 70, 90, 120 and 150 mm: OK. Hip torque is 0.13 to 0.29 Nm.
- Push of 0.3 N s: OK. Push of 0.5 N s and 1.0 N s: the robot falls. (Before the v3 servo model, 0.5 N s was OK. The backlash and the friction make the balance more difficult.)
- Crouch from 150 mm to 70 mm: OK. The hip peak torque is 1.5 Nm at 70 mm.
- Drive: maximum speed 0.35 m/s.

## Model changes, 2026-10-30 (independent review)
- Wheel motor no-load speed: 10.3 rad/s at 11.1 V. The bench test gave 5.76 rad/s at 6.2 V from the position readings. The speed register of the servo reads 13 % low (5.0 rad/s), so do not use it as the truth.
- Wheel command range: +/-2.10 Nm (1.85 Nm x 1.135). This gives space for a full 3S battery (12.6 V). Before, the range cut the push of a full battery.
- Backlash joint armature: 0.0001 (was 0.01, which was too large for a small gear gap).
- Five-bar loop closure: solref 0.004 s, set explicitly. With a 2 ms physics step, MuJoCo needs a time constant of at least 2 steps.
- The RL training uses a 2 ms physics step (`SIM_DT` in `../rl/balance_env.py`). The model file keeps 1 ms for the tests in this folder.
