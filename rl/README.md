# RL work area

This document is written in ASD-STE100 Simplified Technical English.

Use this folder to train policies for Biped v0.2. The policies are for balance, driving and fall recovery. Policies for obstacle avoidance and head tracking will come later.

## Tools
- Simulator: MuJoCo. This is the same simulator as in `../sim/`.
- Robot model: `../sim/eggbot_v03.xml`. The script `../sim/build_model_v03.py` makes this model from the v0.2 CAD masses. Do not make a copy of the model in this folder. Change the model in `sim/` and build it again.
- Training: train on the Mac CPU (Apple silicon). Use MuJoCo Python with many parallel environments and PPO.
- If the training is too slow, use a rented NVIDIA GPU with MJX or MuJoCo Playground. The same MuJoCo model operates there. You do not have to write the model again.

## Tasks (in this sequence)
1. Keep the balance in one position. Then follow speed and turn commands.
2. Change the height (crouch and stand) and keep the balance.
3. Get up from a fall, from flat face-down and from flat on the back. The scripted wheel get-up from the sit position already operates in `../sim/`.
4. Move around obstacles with the OAK-D depth data. Follow a subject with the head.

## Transfer from simulation to the real robot
- Make a servo model from data recorded on the real servos. Do this for the Feetech STS3215 (hips and head) and the ST3215-HS (wheels, motor mode). The model must include the position loop gains, the delay, the backlash and the torque limit.
- During training, change these values at random: the servo values, the mass, the friction and the IMU noise.
- The policy operates on the Pi 5. The IMU data comes from the OAK-D Lite.
