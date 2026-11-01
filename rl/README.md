# RL work area

This document is written in ASD-STE100 Simplified Technical English.

Use this folder to train policies for Road Runner Mini (Biped v0.2). The policies are for balance, driving, height change and fall recovery. Policies for obstacle avoidance and head tracking will come later.

## Goals
A policy is complete only when it does all of these items on the report card:
1. It stays up all the time. Standing still, gentle driving and crouch lines must stay up 50/50.
2. It follows speed commands (forward to 0.15 m/s, reverse to 0.10 m/s) and turn commands (to 1.5 rad/s).
3. It keeps the two legs at the same length on flat ground.
4. It changes the height (crouch and extend, 80 to 135 mm) and keeps the balance.

Large shoves are not a goal. The wheel motor is too slow to recover from them (see `../sim/README.md`).

## Rules (lessons learned, 2026-10-30)
Obey these rules. Each rule comes from a mistake that cost training time.

1. **Use measured values in the simulation.** Do not copy delays, noise or ranges from a different robot. If a value is not measured, write "not measured" in the code comment and measure it soon.
   - Mistake: the command delay (0 to 40 ms) and the sensor noise came from Open Duck Mini. The bench test measured 10 to 25 ms. The 40 ms games caused most of the falls (`diag_017.py`). The noise was 5 to 10 times larger than real IMU and servo noise.
2. **Randomise only near the measured value.** The range must include the real robot with a small margin. A range that is too wide makes the task more difficult than the real robot. Then the policy falls in the simulation and learns slowly.
3. **A gate must not pass a policy that falls.** The level goes up only when 90 of the last 100 games end without a fall. Do not use the average or the median game length.
   - Mistake: a median of 8 s passed a level when many games ended in a fall. A policy at "level 100 %" still fell in 1 of 4 calm games.
4. **The report card must have enough games.** Use 50 games for each line. With 6 games, one fall changes the result by 17 %. Include 60 s standing lines and corner lines (commands at their limits together).
5. **Do not continue to the next task while the current task fails.** "Stays up" is the first pass mark. Speed and turn come second.
6. **Find the cause before you change the reward.** Replay the failed games and record the random settings of each game. Then find which setting goes with the failures. Do one change at a time.
7. **Use one truth for each value.** The wheel speed comes from the position readings (5.76 rad/s at 6.2 V). The speed register of the servo reads 13 % low. On the Pi, calculate the wheel speed from the position.
8. **Keep the structure simple.** The two legs get the same hip commands (mirrored). This keeps the legs at the same length without a penalty.
9. **Compare the sim and the robot with the same measurement.** Use the same function for both (for example, wheel speed from the position difference, `buzz_report.py` and the report card use `count_flips`). Mistake (2026-11-01): the sim used the true wheel speed, the Pi used the position difference, and the real wheel looked 2x stronger.
10. **Measure the body-to-wheel ratio only with the hips frozen.** With free hips, the brain's hip moves change the result.
11. **The level-1 start leans and shoves belong to the plant.** Measure them again when the sim plant changes (`envscan`: the largest values that the plant recovers 9 of 10 times, then use 80 %). Mistake (2026-11-01): drive_022 used the old limits and the level stopped at 96 %.
12. **The score must ask for the posture that you want.** If posture costs almost nothing, the brain finds a strange posture (drive_020 to drive_022 stood with the hips swung far over). Size each score term so it matters against the other terms.
13. **An observation filter is part of the robot.** The brain learns the delay of every filter on its sensors. Print the phase of the filter at the loop frequencies (the report card does it for the gyro filter). Change a filter only together with a new training run, and use the same filter code in the sim and on the Pi (`../robot/filters.py`, values in the brain file).
   - Mistake (2026-11-01): the 6 Hz gyro low-pass stopped the 11 Hz buzz, but it made the gyro 50 degrees late at 3.5 Hz. drive_026 then rocked at 3.5 Hz on the robot, more with a full battery. A Pi-only filter change makes it worse: the brain falls or the 11 Hz buzz comes back.

## Tools
- Simulator: MuJoCo 3.17 on the Mac CPU (Apple M3 Pro: 5 fast cores and 6 slow cores).
- Robot model: `../sim/roadrunner_mini.xml`. The script `../sim/build_model_v03.py` makes this model. Do not make a copy of the model in this folder. Change the builder in `sim/` and build the model again.
- Training: Stable-Baselines3 PPO with parallel robot copies.

## Files
- `balance_env.py`: the balance game and the realism (servo model, delays, noise, random robots).
- `drive_env.py`: the driving game (speed, turn and height commands) and the driving report card.
- `train.py`: trains the policy with PPO. It saves the policy in `runs/<RUN>/latest.zip` and the level in `runs/<RUN>/level.txt`.
- `export_brain.py`: makes `../robot/brains/brain_<RUN>.npz` (the brain for the robot) from `runs/<RUN>/latest.zip` and checks it.
- `sim_test.py`: tests the robot program (`../robot/`) against the simulator.
- `buzz_report.py`: the buzz numbers of a real run log (`../robot/logs/run_<time>.csv`).
- `envscan.py`: scans the random settings of the driving game.
- `bench/`: servo bench tests (`spinup_test.py`, `wheel_bench.py`) and their results.
- `parked/`: the get-up work (parked 2026-10-30, review NO-GO): `getup_env.py` and the scans.
- The game uses the same filter, brain and controller code as the robot: `balance_env.py` loads them from `../robot/`.

## Commands
- Train a driving policy: `N_ROBOTS=10 TASK=drive RUN=<name> TOTAL_STEPS=20000000 .venv/bin/python train.py`. Ctrl+C stops the training and saves the policy. The same command continues the training and adds TOTAL_STEPS.
- Report card: `TASK=drive RUN=<name> .venv/bin/python drive_env.py --test`.
- Watch the latest policy: `.venv/bin/mjpython train.py --watch`.

## Settings (environment variables)
- `SIM_DT` (default 0.002 s): the physics step. 2 ms gives the same report card as 1 ms and is 1.75 times faster.
- `N_ROBOTS` (default 8): the number of robot copies. On the M3 Pro, 10 can be a little faster. Look at the decisions per second in the log.
- `MIRROR` (default 1): the two legs get the same hip commands. `MIRROR=0` gives each hip its own command. Policies trained before 2026-10-30 10:40 used `MIRROR=0` and the old realism; they do not operate with the current files.
- `ACT_DELAY_MS` (default 10,30): the command delay range in ms (measured: 10 to 25 ms). 10,40 was a trial (drive_028/030) and did not help.
- `GYRO_LP` (default 6 Hz), `GYRO_LP_RANGE` (default 5,8): the gyro low-pass on the Pi and its random range in the sim. drive_030 was a trial with `GYRO_LP=10 GYRO_LP_RANGE=9,11`. `GYRO_NOTCH` (default 0 = off) and `GYRO_NOTCH_Q`: the notch of the trial brains drive_028/029. `sim_test.py` reads the filter of the brain from its file.
- `MAX_DELAY` (default 1): the IMU delay, 0 to 1 policy steps (0 to 20 ms). Not measured yet.
- `LEVEL_UP_SHARE` (default 0.9), `LEVEL_GAMES` (default 100): the gate.
- `CARD_GAMES` (default 50): games for each report card line.
- `SPEED_MAX` (0.15 m/s), `REVERSE_MAX` (0.10 m/s): the speed command limits. The 70 mm wheels are at the motor limit at 0.2 m/s.

## The driving game (current)
- Observation (3 frames of history): gravity direction, gyro, hip positions and speeds, wheel speeds, the commands, two "behind schedule" signals (speed and heading) and the last action. The two signals slowly forget old errors (0.99 for each step), so they cannot stay at their limit.
- Action: 2 hip commands (front and rear, the same for both legs) and 2 wheel PWM commands. The PWM command goes directly to the servo. The Pi adds nothing.
- Gyro filter: a 2-pole low-pass at 6 Hz (5 to 8 Hz in the sim). Trials on 2026-11-01: a 10.5 Hz notch (drive_028/029) made the training stop at level 76 to 82 %; a 10 Hz low-pass (drive_030) rocked at 3.5 Hz the same as drive_026.
- Wobble score (from drive_031, review round 8d): -1.0 x min(pitch rate^2, 1) - 0.05 x roll rate^2, from the true gyro. Before: -0.05 x (pitch rate^2 + roll rate^2), which let the brain rock at 3.5 Hz for almost no cost. `WOBBLE_PITCH_W`, `WOBBLE_CLIP`.
- Report card: "pitch rms" and "wheel travel rms" over the last 5 s of each game; CALM MARK on standing lines: pitch rms below 1 degree, wheel travel rms below 0.3 rad (a slow back-and-forth hunt must not replace the 3.5 Hz rocking).
- Report card "stress" lines: full battery, 40 ms command delay, wheel dead band 0.02. Pass: 2.5-5 Hz gyro band below 0.05 rad/s in every game and no 9-13 Hz buzz. On the robot: `buzz_report.py` with a full battery, 2.5-5 Hz band below 0.10 rad/s.
- Realism for each game: command delay 10 to 40 ms, IMU delay 0 to 20 ms, IMU mounted up to 1 degree crooked, small sensor noise, battery 0.85 to 1.135 (9.4 to 12.6 V), masses +/-10 %, centre of mass +/-10 mm, tyre grip 0.5 to 1.0, servo zero offsets +/-1.7 degrees, wheel friction 0.3 to 3 times (not measured), wheel armature 0.7 to 1.4 times (measured).

## Tasks (in this sequence)
1. Stay up and drive with speed, turn and height commands (in progress).
2. Get up from a fall. This is not proved yet. First do a small test in the simulation.
3. Move the policy to the Pi 5 and test it on the bench.
4. Move around obstacles with the OAK-D depth data. Follow a subject with the head.
