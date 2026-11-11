# Simulation

This document is written in ASD-STE100 Simplified Technical English.

This folder has the MuJoCo model of Road Runner Mini and the test scripts.

## The model
- `roadrunner_mini.xml` is the model. The script `build_model_v03.py` makes it. Do not edit the XML file. Change the script and build the model again.
- The model has two layers. The CAD meshes in `meshes/` show the robot (geom group 2). Simple shapes (boxes, capsules, cylinders) do the physics (geom group 3, hidden). Simple shapes make the simulation fast and the contacts stable.
- To see the physics shapes in the viewer, turn on geom group 3 in the Rendering panel.
- Each mesh is in the frame of its body at the stand pose (leg height 120 mm). Large meshes are reduced to 15000 triangles or less.
- The body meshes show an older design state. The lid is 12 mm lower than the real lid, and the upper deck has the old layout. The physics shapes and the masses use the correct lid height.
- The model has 4 hip servos, 2 wheel servos, head yaw and head pitch.
- Head pitch limits: 30 degrees up, 50 degrees down. Head yaw limits: 90 degrees left and right.

## Masses
- The masses, the centres of mass and the joint positions come from the CAD model.
- The real robot is 1754 g with the battery. The CAD model does not include the wires, the connectors, the fuse, the UBECs, the bus servo adapters, the inserts and the screws. One point mass of 240 g at deck height replaces them (`M_EXTRA`).
- The position of this point mass is not measured. The RL training changes the centre of mass in a range of +/-10 mm.

## Servo model
Hip and head servos (STS3215):
- kp 98.8, damping 0.49, armature 0.0037, command time constant 0.024 s, maximum torque 1.94 Nm. These values are a fit to step tests on the robot (`../robot/tools/hip_bench.py`, `fit_hip_servo.py`, data in `data/`).
- Friction loss 0.052. This value is from Open Duck Mini. It is not measured.
- The fit is for the 7.4 V version of the servo, which the robot has.

Wheel servos (ST3215-HS, battery voltage):
- Wheel torque decreases as wheel speed increases: 1.85 Nm at 0 rad/s, 0 Nm at 10.3 rad/s (11.1 V).
- Command range: +/-2.10 Nm. This includes a full battery (12.6 V).
- Armature 0.004 (measured with a spin-up test).
- Friction loss 0.022. This value is not measured. It comes from the STS3215 value, for a gear ratio that is approximately 2.4 times lower. The RL training changes it in a wide range.
- Measure the wheel speed from the position readings. The speed register of the servo reads 13 % low.

Backlash:
- Each hip servo and each wheel servo has a backlash joint behind it. The backlash is +/-0.0087 rad (+/-0.5 degrees). The armature of the backlash joint is 0.0001.

Other settings:
- Five-bar loop closure: solref 0.004 s. With a 2 ms physics step, MuJoCo needs a time constant of 2 steps or more.
- The model file uses a 1 ms physics step for the tests in this folder. The RL training uses a 2 ms step (`SIM_DT` in `../rl/balance_env.py`).

## Install
Use the Python environment in `../rl/.venv` (MuJoCo 3.17).

## Build the model
Run this command:

```
../rl/.venv/bin/python build_model_v03.py 0.120
```

The number is the stand height in m.

## Tests
- `tests.py` has helper functions and the balance, push, crouch and drive tests. They use a hand-tuned balance controller, not the trained brain.
- `set_wheel_torque()` adds the lost speed torque back, up to the motor limit.
- `old/` has scripts for an older model. Their results are not valid for this model.
