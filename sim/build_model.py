"""Build the v0 MJCF for the egg-torso wheeled biped (five-bar legs).
Units: metres, kg. X forward, Y left, Z up. Hip frame origin = midpoint of the four hip axes.
"""
import math, sys

# ---- spec v0 numbers (mm -> m) ----
L1 = 0.060      # upper link
L2 = 0.100      # lower link
HIP_DX = 0.025  # half spacing of the two hip axes (50 mm apart)
WHEEL_R = 0.035
TRACK = 0.135
HALF_Y = TRACK / 2
TORSO_W, TORSO_D, TORSO_H = 0.100, 0.150, 0.115
HEAD_W, HEAD_H, HEAD_D = 0.100, 0.055, 0.075
HIP_DROP = 0.025

# masses (kg)
M_TORSO = float(__import__('os').environ.get('M_TORSO','0.85'))     # battery 0.277, Pi+cooler 0.07, UBECs 0.064, boards 0.02, 4 hip servos 0.232, prints ~0.19
M_HEAD = float(__import__('os').environ.get('M_HEAD','0.13'))      # OAK-D 0.061 + servo 0.058 + print
M_NECK = float(__import__('os').environ.get('M_NECK','0.0'))        # yaw servo on the lid (v0.2)
HEAD_PIVOT_Z = float(__import__('os').environ.get('HEAD_PIVOT_Z', str(HIP_DROP + TORSO_H/2 + 0.004)))
M_UP = 0.015
M_LOW = 0.020
M_WHEELSERVO = 0.068
M_WHEEL = 0.012

STS_TORQUE = 1.5   # Nm, 7.4 V STS3215 limit used in sim (stall 1.9)
HS_TORQUE = 1.2    # Nm, ST3215-HS at 7.4 V (2.0 at 12 V)
HS_VEL = 6.8       # rad/s at 7.4 V (~65 rpm)


def leg_angles(h):
    """Given hip-to-axle vertical distance h (axle straight below hip centre),
    return (theta_front, theta_rear) hip angles measured from straight-down (+ = knee forward)
    and knee positions, for the symmetric five-bar."""
    d = math.hypot(HIP_DX, h)
    # angle at hip between hip->axle line and upper link
    cos_a = (L1**2 + d**2 - L2**2) / (2 * L1 * d)
    a = math.acos(max(-1, min(1, cos_a)))
    # front hip at x=+HIP_DX, axle at x=0: hip->axle direction is down and back
    # MuJoCo hinge about +Y: positive q swings a hanging link backward (-X).
    # geometric angle from -Z toward +X of hip->axle line:
    phi = math.atan2(-HIP_DX, h)
    geom_f = phi + a          # front knee forward of hip->axle line
    th_f = -geom_f            # joint angle
    th_r = -th_f
    return th_f, th_r


def knee_angle(h):
    d = math.hypot(HIP_DX, h)
    cos_b = (L1**2 + L2**2 - d**2) / (2 * L1 * L2)
    return math.acos(max(-1, min(1, cos_b)))


SHELL_R = 0.020  # corner radius of the rounded box shell


def shell_geoms(shell):
    if shell == "egg":
        return f"""<geom name="egg" type="ellipsoid" size="{TORSO_D/2} {TORSO_W/2} {TORSO_H/2}" pos="0 0 {HIP_DROP}"
            mass="{M_TORSO}" rgba="0.93 0.90 0.84 1"/>
      <geom name="chin" type="capsule" fromto="{TORSO_D/2 - 0.012} -0.03 {HIP_DROP - 0.035} {TORSO_D/2 - 0.012} 0.03 {HIP_DROP - 0.035}" size="0.012" mass="0.01" rgba="0.3 0.3 0.33 1"/>
      <geom name="tail" type="capsule" fromto="{-(TORSO_D/2 - 0.012)} -0.03 {HIP_DROP - 0.035} {-(TORSO_D/2 - 0.012)} 0.03 {HIP_DROP - 0.035}" size="0.012" mass="0.01" rgba="0.3 0.3 0.33 1"/>"""
    # rounded box = three slabs + capsules along the 12 edges
    a, b, c, r = TORSO_D/2, TORSO_W/2, TORSO_H/2, SHELL_R
    g = [f'<geom name="egg" type="box" size="{a-r} {b-r} {c}" pos="0 0 {HIP_DROP}" mass="{M_TORSO}" rgba="0.93 0.90 0.84 1"/>',
         f'<geom type="box" size="{a-r} {b} {c-r}" pos="0 0 {HIP_DROP}" mass="0" rgba="0.93 0.90 0.84 1"/>',
         f'<geom type="box" size="{a} {b-r} {c-r}" pos="0 0 {HIP_DROP}" mass="0" rgba="0.93 0.90 0.84 1"/>']
    k = 0
    for sx in (1, -1):
        for sz in (1, -1):  # edges along Y
            z = HIP_DROP + sz*(c-r); x = sx*(a-r)
            nm = ' name="chin"' if (sx > 0 and sz < 0) else (' name="tail"' if (sx < 0 and sz < 0) else '')
            g.append(f'<geom{nm} type="capsule" fromto="{x} {-(b-r)} {z} {x} {b-r} {z}" size="{r}" mass="0" rgba="0.93 0.90 0.84 1"/>')
        for sy in (1, -1):  # edges along Z
            g.append(f'<geom type="capsule" fromto="{sx*(a-r)} {sy*(b-r)} {HIP_DROP-(c-r)} {sx*(a-r)} {sy*(b-r)} {HIP_DROP+(c-r)}" size="{r}" mass="0" rgba="0.93 0.90 0.84 1"/>')
    for sy in (1, -1):
        for sz in (1, -1):  # edges along X
            g.append(f'<geom type="capsule" fromto="{-(a-r)} {sy*(b-r)} {HIP_DROP+sz*(c-r)} {a-r} {sy*(b-r)} {HIP_DROP+sz*(c-r)}" size="{r}" mass="0" rgba="0.93 0.90 0.84 1"/>')
    return "\n      ".join(g)


def build(h_stand=0.120, shell="egg"):
    th_f, th_r = leg_angles(h_stand)
    kb = knee_angle(h_stand)
    # knee joint angle relative to upper link: lower link bends back toward axle
    knee_f = (math.pi - kb)
    knee_r = -(math.pi - kb)

    def leg(side, sy):
        s = side
        return f"""
      <body name="up_f_{s}" pos="{HIP_DX} {sy} 0">
        <joint name="hip_f_{s}" type="hinge" axis="0 1 0" range="-2.4 2.4" damping="0.002"/>
        <geom class="link" type="capsule" fromto="0 0 0 0 0 {-L1}" size="0.004" mass="{M_UP}" rgba="0.85 0.85 0.85 1"/>
        <body name="low_f_{s}" pos="0 0 {-L1}">
          <joint name="knee_f_{s}" type="hinge" axis="0 1 0" damping="0.002"/>
          <geom class="link" type="capsule" fromto="0 0 0 0 0 {-L2}" size="0.004" mass="{M_LOW}" rgba="0.85 0.85 0.85 1"/>
          <body name="axle_{s}" pos="0 0 {-L2}">
            <geom class="link" type="box" size="0.012 0.0225 0.0175" pos="0 {-0.014*(1 if sy>0 else -1)} 0.01" mass="{M_WHEELSERVO}" rgba="0.2 0.2 0.25 1"/>
            <site name="axle_{s}" pos="0 0 0" size="0.003"/>
            <body name="wheel_{s}" pos="0 {0.0085*(1 if sy>0 else -1)} 0">
              <joint name="wheel_{s}" type="hinge" axis="0 1 0" damping="0.0005"/>
              <geom type="cylinder" size="{WHEEL_R} 0.004" quat="0.7071 0.7071 0 0" mass="{M_WHEEL}" rgba="0.1 0.1 0.1 1" friction="1.0 0.005 0.0001" condim="3"/>
            </body>
          </body>
        </body>
      </body>
      <body name="up_r_{s}" pos="{-HIP_DX} {sy} 0">
        <joint name="hip_r_{s}" type="hinge" axis="0 1 0" range="-2.4 2.4" damping="0.002"/>
        <geom class="link" type="capsule" fromto="0 0 0 0 0 {-L1}" size="0.004" mass="{M_UP}" rgba="0.85 0.85 0.85 1"/>
        <body name="low_r_{s}" pos="0 0 {-L1}">
          <joint name="knee_r_{s}" type="hinge" axis="0 1 0" damping="0.002"/>
          <geom class="link" type="capsule" fromto="0 0 0 0 0 {-L2}" size="0.004" mass="{M_LOW}" rgba="0.85 0.85 0.85 1"/>
          <site name="tip_r_{s}" pos="0 0 {-L2}" size="0.003"/>
        </body>
      </body>"""

    xml = f"""
<mujoco model="eggbot_v0">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.001" gravity="0 0 -9.81" integrator="implicitfast"/>
  <default>
    <geom contype="1" conaffinity="1" friction="0.9 0.005 0.0001"/>
    <default class="link"><geom contype="2" conaffinity="0"/></default>
    <equality solref="0.002 1"/>
  </default>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.9 0.93 0.97" rgb2="1 1 1" width="64" height="64"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.82 0.82 0.82" rgb2="0.9 0.9 0.9" width="256" height="256"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.05"/>
  </asset>
  <worldbody>
    <light pos="0.5 -1 2" dir="-0.3 0.5 -1" diffuse="0.9 0.9 0.9"/>
    <light pos="-1 1 1.5" dir="0.5 -0.5 -1" diffuse="0.4 0.4 0.4"/>
    <geom name="floor" type="plane" size="3 3 0.1" material="grid" contype="1" conaffinity="3"/>
    <body name="torso" pos="0 0 {WHEEL_R + h_stand}">
      <freejoint name="root"/>
      <site name="imu" pos="0 0 0.02" size="0.004"/>
      <!-- egg torso: ellipsoid, hips at its bottom -->
      {shell_geoms(shell)}
      {('<geom name="neck" type="box" size="0.0124 0.0226 0.019" pos="0.004 -0.0125 %.4f" mass="%.3f" rgba="0.2 0.2 0.25 1"/>' % (HIP_DROP + TORSO_H/2 + 0.019, M_NECK)) if M_NECK > 0 else ''}
      <body name="head" pos="0.004 0 {HEAD_PIVOT_Z}">
        <joint name="head_pitch" type="hinge" axis="0 1 0" range="-0.8 0.8" damping="0.01"/>
        <geom type="ellipsoid" size="{HEAD_D/2} {HEAD_W/2} {HEAD_H/2}" pos="0.008 0 {(HEAD_H/2 + 0.002) if M_NECK == 0 else 0.0145}" mass="{M_HEAD}" rgba="0.93 0.90 0.84 1"/>
        <site name="cam" pos="{HEAD_D/2} 0 {HEAD_H/2}" size="0.003"/>
      </body>
      {leg('L', HALF_Y - 0.0125)}
      {leg('R', -(HALF_Y - 0.0125))}
    </body>
  </worldbody>
  <equality>
    <connect name="fivebar_L" site1="tip_r_L" site2="axle_L"/>
    <connect name="fivebar_R" site1="tip_r_R" site2="axle_R"/>
  </equality>
  <actuator>
    <position name="hip_f_L" joint="hip_f_L" kp="30" kv="0.6" forcerange="-{STS_TORQUE} {STS_TORQUE}"/>
    <position name="hip_r_L" joint="hip_r_L" kp="30" kv="0.6" forcerange="-{STS_TORQUE} {STS_TORQUE}"/>
    <position name="hip_f_R" joint="hip_f_R" kp="30" kv="0.6" forcerange="-{STS_TORQUE} {STS_TORQUE}"/>
    <position name="hip_r_R" joint="hip_r_R" kp="30" kv="0.6" forcerange="-{STS_TORQUE} {STS_TORQUE}"/>
    <position name="head_pitch" joint="head_pitch" kp="5" kv="0.1" forcerange="-1 1"/>
    <motor name="wheel_L" joint="wheel_L" ctrlrange="-{HS_TORQUE} {HS_TORQUE}"/>
    <motor name="wheel_R" joint="wheel_R" ctrlrange="-{HS_TORQUE} {HS_TORQUE}"/>
  </actuator>
  <sensor>
    <gyro name="gyro" site="imu"/>
    <accelerometer name="acc" site="imu"/>
    <framequat name="torso_quat" objtype="site" objname="imu"/>
  </sensor>
  <keyframe>
    <key name="stand" qpos="0 0 {WHEEL_R + h_stand} 1 0 0 0  0  {th_f} {knee_f} 0 {th_r} {knee_r}  {th_f} {knee_f} 0 {th_r} {knee_r}"/>
  </keyframe>
</mujoco>"""
    return xml, (th_f, th_r, knee_f, knee_r)


if __name__ == "__main__":
    h = float(sys.argv[1]) if len(sys.argv) > 1 else 0.120
    shell = sys.argv[2] if len(sys.argv) > 2 else "egg"
    xml, ang = build(h, shell)
    out = __import__('os').environ.get('OUT_XML') or (f"eggbot_{shell}.xml")
    open(out, "w").write(xml)
    print("angles", ang)
