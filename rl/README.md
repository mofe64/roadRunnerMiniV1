# RL work area

This document is written in ASD-STE100 Simplified Technical English.

Use this folder to train policies (brains) for Road Runner Mini. The current policy does balance, driving and height change. The robot program in `../robot/` runs the policy on the Raspberry Pi 5.

## Terms
- Game: one simulation run of the robot, from the start pose to a fall or to the time limit.
- Level: the difficulty of the games, 0 to 100 %. The training increases it.
- Gate: the condition that lets the level go up.
- Report card: a test of a trained policy. Each line of the card is one test condition.
- Buzz: a fast vibration of the robot (9 to 13 Hz). Rocking is a slow one (2.5 to 5 Hz).

## Goals
A policy is complete only when it does all of these items on the report card:
1. It stays up all the time. The standing lines, the gentle driving lines and the crouch lines must stay up in 50 of 50 games.
2. It follows speed commands (forward to 0.15 m/s, reverse to 0.10 m/s) and turn commands (to 1.5 rad/s).
3. It keeps the two legs at the same length on flat ground.
4. It changes the height (crouch and extend, 80 to 135 mm) and keeps the balance.

Large shoves are not a goal. The wheel motor is too slow to recover from them.

## Rules
Obey these rules. Each rule prevents a mistake that used training time.

1. **Use measured values in the simulation.** Do not copy delays, noise or ranges from a different robot. If a value is not measured, write "not measured" in the code comment and measure it.
2. **Randomise only near the measured value.** The range must include the real robot with a small margin. A range that is too wide makes the task more difficult than the real robot. Then the policy falls in the simulation and learns slowly.
3. **A gate must not pass a policy that falls.** The level goes up only when 90 of the last 100 games end without a fall. Do not use the average or the median game length.
4. **The report card must have sufficient games.** Use 50 games for each line. With 6 games, one fall changes the result by 17 %. Include 60 s standing lines and corner lines (commands at their limits together).
5. **Do not continue to the next task while the current task fails.** "Stays up" is the first pass mark. Speed and turn come second.
6. **Find the cause before you change the reward.** Replay the failed games and record the random settings of each game. Then find which setting goes with the failures. Do one change at a time.
7. **Use one source for each value.** The wheel speed comes from the position readings. The speed register of the servo reads 13 % low. On the Pi, calculate the wheel speed from the position.
8. **Keep the structure simple.** The two legs get the same hip commands (mirrored). This keeps the legs at the same length without a penalty.
9. **Compare the simulation and the robot with the same measurement.** Use the same function for both. For example, `buzz_report.py` and the report card use `count_flips`. If the simulation uses the true wheel speed and the Pi uses the position difference, the results do not agree.
10. **Measure the body-to-wheel ratio only with the hips frozen.** With free hips, the hip movements of the brain change the result.
11. **The start leans and shoves of level 1 depend on the simulation model.** Measure them again when the model changes. Use `envscan.py`: find the largest values that the model recovers from in 9 of 10 games, then use 80 % of them.
12. **The score must ask for the posture that you want.** If posture costs almost nothing, the brain finds a strange posture. Give each score term a size that is important against the other terms.
13. **An observation filter is part of the robot.** The brain learns the delay of every filter on its sensors. Change a filter only together with a new training run. Use the same filter code in the simulation and on the Pi (`../robot/filters.py`; the filter values are in the brain file). The report card prints the phase of the gyro filter at the loop frequencies.
14. **Make sure that the robot is mechanically correct before you use its logs.** Loose knee screws make the robot rock at 2.7 to 3.5 Hz. Logs from a robot with loose knee screws are not valid.

## Tools
- Simulator: MuJoCo 3.17 on the Mac CPU.
- Robot model: `../sim/roadrunner_mini.xml`. The script `../sim/build_model_v03.py` makes this model. Do not make a copy of the model in this folder. Change the builder in `sim/` and build the model again.
- Training: Stable-Baselines3 PPO with parallel robot copies.
- Python environment: `.venv` in this folder. Do not install packages globally.

## Files
- `balance_env.py`: the balance game and the realism (servo model, delays, noise, random robots).
- `drive_env.py`: the driving game (speed, turn and height commands) and the driving report card.
- `train.py`: trains the policy with PPO. It saves the policy in `runs/<RUN>/latest.zip` and the level in `runs/<RUN>/level.txt`.
- `export_brain.py`: makes `../robot/brains/brain_<RUN>.npz` (the brain for the robot) from `runs/<RUN>/latest.zip` and checks it.
- `sim_test.py`: tests the robot program (`../robot/`) against the simulator. It reads the filter of the brain from the brain file.
- `buzz_report.py`: the buzz numbers of a real run log (`../robot/logs/run_<time>.csv`).
- `envscan.py`: scans the random settings of the driving game.
- `bench/`: servo bench tests (`spinup_test.py`, `wheel_bench.py`) and their results.
- `parked/`: the get-up work (`getup_env.py` and the scans). This work is stopped. The get-up is not proved in the simulation.
- The game uses the same filter, brain and controller code as the robot: `balance_env.py` loads them from `../robot/`.

## Commands
- Train a driving policy: `N_ROBOTS=10 TASK=drive RUN=<name> TOTAL_STEPS=20000000 .venv/bin/python train.py`. Ctrl+C stops the training and saves the policy. The same command continues the training and adds TOTAL_STEPS.
- Report card: `TASK=drive RUN=<name> .venv/bin/python drive_env.py --test`.
- Watch the latest policy: `.venv/bin/mjpython train.py --watch`.
- Export the brain for the robot: `.venv/bin/python export_brain.py`. The options are at the top of the file.

Use the same environment variables for the training, the report card, `sim_test.py` and the export.

## The current brain: drive_039
`../robot/brains/brain_drive_039.npz` is the default brain of the robot. Its training used these settings:

```
HIP_VEL_MODE=twotap HIP_VEL_HOLD=0,0 IMU_AGE_MS=4,8 ACT_DELAY_MS=8,12
WHEEL_ARM_SCALE=0.5 WHEEL_ARM_RANGE=0.8,1.25 WHEEL_GAIN=1.15
TORSO_COM_X=-0.0125 COM_DR_MM=2.5 FROZEN_SHARE=0.15 CMD_STAIR=1
```

Pass mark for a new brain: on the standing lines and the 0.15 m/s lines, the 2.5 to 5 Hz gyro band is below 0.10 rad/s in every game, with 8 or more seeds.

## Settings (environment variables)
The defaults in the code are not the values of the current brain. Set the variables as shown above.

| Variable | Default | Function |
|---|---|---|
| `SIM_DT` | 0.002 s | Physics step. 2 ms gives the same report card as 1 ms and is 1.75 times faster. |
| `N_ROBOTS` | 8 | Number of robot copies. Look at the decisions per second in the log to find the best number. |
| `MIRROR` | 1 | The two legs get the same hip commands. `MIRROR=0` gives each hip its own command. |
| `ACT_DELAY_MS` | 10,30 | Command delay range in ms. Measured on the robot: 8 to 12 ms. |
| `IMU_AGE_MS` | 0,0 | Age of the IMU sample in ms. Measured on the robot: 4 to 8 ms. |
| `MAX_DELAY` | 0 | More IMU delay in full policy steps. Keep it at 0. |
| `HIP_VEL_MODE`, `HIP_VEL_HOLD` | empty; 0,0 | `twotap` makes the hip speed as the servo speed register does: (q[k] - q[k-2]) / 40 ms. `HIP_VEL_HOLD` adds a delay in decisions. |
| `GYRO_LP`, `GYRO_LP_RANGE` | 6 Hz; 5,8 | Gyro low-pass on the Pi and its random range in the simulation. |
| `GYRO_NOTCH`, `GYRO_NOTCH_Q` | 0 (off); 1.0 | Gyro notch filter. Not used. |
| `WHEEL_GAIN` | 1.0 | Scales the wheel torque and its ranges. The no-load speed increases with it. |
| `WHEEL_ARM_SCALE`, `WHEEL_ARM_RANGE` | 0.25; 0.6,2.0 | Wheel rotor inertia scale and its random range. |
| `WHEEL_DEADBAND` | 0.040 | A PWM command below this value gives no push. The random range is 0.8 to 1.2 times. |
| `TORSO_COM_X` | 0 | Moves the body centre of mass forward (+) or back (-), in m. 0 is the CAD value. |
| `COM_DR_MM` | 10 | Random range of the centre of mass, forward and back, in mm. |
| `FROZEN_SHARE` | 0 | Share of the training games with the hips held at the 120 mm stand pose, as `run_brain.py --freeze-hips` does. |
| `CMD_STAIR` | 0 | 1 = speed and turn commands change in the gamepad steps (0.05 m/s, 0.25 rad/s, each 0.2 s). |
| `LEVEL_UP_SHARE`, `LEVEL_GAMES` | 0.9; 100 | The gate: the level goes up when this share of the last games ends without a fall. |
| `CARD_GAMES` | 50 | Games for each report card line. |
| `SPEED_MAX`, `REVERSE_MAX` | 0.15; 0.10 m/s | Speed command limits. The 70 mm wheels are at the motor limit at 0.2 m/s. |
| `WOBBLE_PITCH_W`, `WOBBLE_CLIP` | 1.0; 1.0 | Wobble score weights (see below). |

## The driving game
- Observation (3 frames of history): gravity direction, gyro, hip positions and speeds, wheel speeds, the commands, two "behind schedule" signals (speed and heading) and the last action. The two signals slowly forget old errors (0.99 for each step), so they cannot stay at their limit.
- Action: 2 hip commands (front and rear, the same for both legs) and 2 wheel PWM commands. The PWM command goes directly to the servo. The Pi adds nothing.
- Gyro filter: a 2-pole low-pass at 6 Hz (5 to 8 Hz in the simulation).
- Wobble score: -1.0 x min(pitch rate^2, 1) - 0.05 x roll rate^2, from the true gyro.
- Sensor noise: gravity 0.0005, gyro 0.02, hip position 0.0015, hip speed 0.05, wheel speed 0. The game rounds the wheel positions to servo counts before it calculates the speed. With these values, the simulation with frozen hips stands as calmly as the robot.
- Random settings for each game: command delay, IMU age, IMU mounted up to 1 degree off, battery 0.85 to 1.135 (9.4 to 12.6 V), masses +/-10 %, centre of mass, tyre grip 0.5 to 1.0, servo zero offsets +/-1.7 degrees, wheel friction 0.3 to 3 times (not measured), wheel rotor inertia, wheel dead band.

## The report card
- Each line has 50 games.
- The card measures "pitch rms" and "wheel travel rms" over the last 5 s of each game.
- Calm mark on the standing lines: pitch rms below 1 degree and wheel travel rms below 0.3 rad.
- FROZEN line: the hips are held as `run_brain.py --freeze-hips` holds them. The brain still sees its own action.
- Stress lines: full battery, 40 ms command delay, wheel dead band 0.02. Pass: the 2.5 to 5 Hz gyro band is below 0.05 rad/s in every game and there is no 9 to 13 Hz buzz.
- On the robot: `buzz_report.py` with a full battery. Pass: the 2.5 to 5 Hz band is below 0.10 rad/s.

## Tasks (in this sequence)
1. Stay up and drive with speed, turn and height commands. Done (drive_039).
2. Get up from a fall. First do a small test in the simulation.
3. Follow a subject with the head.
