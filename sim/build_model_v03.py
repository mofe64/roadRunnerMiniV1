"""Build the MuJoCo model (MJCF) of Road Runner Mini (five-bar legs, wheels, pan-tilt head).
Masses and joint positions come from the Fusion model "Biped v0.1", v0.2 S59 (2026-10-28).
Units: metres, kg. X forward, Y left, Z up. Hip frame origin = midpoint of the four hip axes.
"""
import math, sys

# ---- spec v0 numbers (mm -> m) ----
L1 = 0.060      # upper link
L2 = 0.100      # lower link
HIP_DX = 0.025  # half spacing of the two hip axes (50 mm apart)
WHEEL_R = float(__import__('os').environ.get('WHEEL_R', '0.035'))   # m; WHEEL_R=0.05 builds the 100 mm printed-wheel test
TRACK = 0.135
HALF_Y = TRACK / 2
TORSO_W, TORSO_D, TORSO_H = 0.100, 0.150, 0.115
HEAD_W, HEAD_H, HEAD_D = 0.100, 0.065, 0.0955
HIP_DROP = 0.025

# masses (kg) and centres of mass (m, hip frame) from CAD S59
M_TORSO = float(__import__('os').environ.get('M_TORSO','0.982'))     # Body: chassis, deck, tray, lid, battery, Pi, boards, 4 hip servos
TORSO_COM = (-0.0018, 0.0, 0.0249)
M_YAWSERVO = 0.058                       # yaw STS3215, fixed in the lid
# CAD S67 (2026-10-31): the lid walls are 12 mm taller (cable room). The yaw servo, neck and head moved up
# exactly 12 mm (pure translation); the lid gained 13.88 g in the wall band (COM about 73 mm above the hips).
# The visual meshes of the body are still the S56 export (the lid looks 12 mm short); physics is S67.
LID_RISE = 0.012
M_LID_BAND, LID_BAND_COM = 0.01388, (0.0, 0.0, 0.073)
# Real robot WEIGHED 2026-10-31: 1754 g with the battery and the head, without the bumps. The CAD (S67, no bumps)
# is 1514 g, so 240 g is not in the CAD: wires, connectors, fuse, the 2 UBECs, the bus boards, heat-set inserts,
# screws. Most of it is in the body around the deck, so it is one point mass at deck height (where exactly is
# not measured; 240 g anywhere from the hip line to the lid moves the body COM by less than +/-6 mm, inside the
# +/-10 mm randomisation).
M_EXTRA = float(__import__('os').environ.get('M_EXTRA', '0.240'))
EXTRA_COM = (0.0, 0.0, 0.040)
YAWSERVO_COM = (-0.0097, 0.0, 0.0987 + LID_RISE)
YAW_AXIS = (0.004, 0.0, 0.1177 + LID_RISE)   # vertical axis, at the top of the yaw drive horn
M_BRACKET = 0.0136                       # pitch bracket, turns with yaw
BRACKET_COM = (0.0026, 0.0047, 0.1323 + LID_RISE)
M_HEAD = float(__import__('os').environ.get('M_HEAD','0.2008'))      # head print 0.082 + OAK-D Lite 0.061 + pitch servo 0.058
HEAD_COM = (0.0122, 0.0011, 0.0196)       # from the pitch axis
HEAD_PIVOT_Z = float(__import__('os').environ.get('HEAD_PIVOT_Z', '0.139')) + LID_RISE
HEAD_PITCH_RANGE = (-0.524, 0.873)       # rad, + = nose down: 30 deg up, 50 deg down
HEAD_YAW_RANGE = (-1.571, 1.571)         # rad, +/-90 deg (assumed, cable limited)
M_UP = 0.0071                            # upper link
M_LOW_F = 0.0146                         # lower front plate + knee screw
M_LOW_R = 0.0235                         # lower rear dogleg + knee screw
M_WHEELSERVO = 0.060
WHEELSERVO_POS = (0.0091, 0.0153, 0.0103)  # from the axle; y is inward
M_WHEEL = float(__import__('os').environ.get('M_WHEEL', '0.0114'))   # wheel 0.009 + hub 0.0024 (a printed 100 mm wheel is about 0.03)
WHEEL_OFFSET = 0.0142                    # leg plane to tyre centre (track 138 mm)
WHEEL_HALF_W = 0.0055

# STS3215 hip servo model MEASURED 2026-10-31 on this robot (robot/tools/hip_bench.py, legs in the air, register
# 0x55 = 0 = no acceleration limit, as set_gains.py sets it now). Data: sim/data/hip_bench_20261031_231154_8deg.csv
# and _231209_20deg.csv (front-left hip). Real: 8 deg step 50 % at 54 ms, 90 % at 76 ms; 20 deg step 50 % at
# 78 ms, 90 % at 112 ms, top 4.3 rad/s (data sheet 4.7). Fit (sim/fit_hip_servo.py, both step sizes together):
# error 0.022; force 1.94 Nm = the stall torque, so the fit is physical. The fitted pure delay (24 ms) is the
# same as the command delay every game already gets (ACT_DELAY_MS 10-30 in balance_env.py), so it is not added.
# Before: Open Duck Mini values (kp 17.8, damping 0.60, armature 0.028, no filter). With the factory
# acceleration limit (0x55 = 254) the real hip was 1.7x slower; that limit is now switched off on the robot.
# Friction is still the Open Duck value (the step fit cannot separate it from damping).
STS_TORQUE = 1.94  # Nm (fitted; stall 1.9 at 7.4 V)
STS_KP, STS_DAMPING, STS_FRICTIONLOSS, STS_ARMATURE = 98.8, 0.49, 0.052, 0.0037
STS_TIMECONST = 0.024   # s: the servo's own command smoothing (fitted)
BACKLASH = 0.0087  # rad: +/-0.5 deg of gear slop behind each hip servo (Open Duck value)
# Wheel servo (ST3215-HS) gearbox: NOT measured yet. Guess scaled from the STS3215: it is about
# 2.4x faster, so a ~2.4x lower gear ratio -> friction /2.4 and rotor inertia /2.4^2. Same backlash.
WHEEL_FRICTIONLOSS, WHEEL_ARMATURE = 0.022, 0.004   # armature MEASURED 2026-10-29: spin-up test, 20 ms to 63 % at 6.2 V
# Wheel ST3215-HS servos run on battery voltage (3S, 11.1 V nominal).
# Motor curve: torque falls in a straight line from HS_TORQUE at rest to 0 at HS_VEL.
HS_TORQUE = 1.85   # Nm stall at 11.1 V (2.0 at 12 V, scaled)
HS_VEL = 10.3      # rad/s no-load at 11.1 V: bench test 2026-10-30, 5.76 rad/s at 6.2 V from the POSITION readings, scaled.
                   # The servo's own speed register read 5.0 (13 % low), so the Pi must compute wheel speed from position.
BATTERY_MAX = 1.135   # full 3S battery 12.6 V / 11.1 V: the wheel command range leaves room for a full battery


# Get-up bumps (Mofe approved 2026-10-30): two printed rods across the front and back of the body.
# A fallen robot rests on a bump at about 35 deg, not flat, so the legs can push it up again.
# 130 mm at hip height: the smallest bump that stopped all 24 real test falls (fall speed up to 3 rad/s,
# tuck at 20 or 35 deg; getup/fallscan.py). 105 mm at -30 mm rolled over in every real fall.
# BUMPS=0 builds the old body for a comparison only.
BUMPS = int(__import__('os').environ.get('BUMPS', '1'))
BUMP_X = float(__import__('os').environ.get('BUMP_X', '0.130'))   # m, rod centre from the hip line centre (torso frame)
BUMP_Z = float(__import__('os').environ.get('BUMP_Z', '0.0'))       # m, at the hip line height
BUMP_R, BUMP_HALF_Y = 0.012, 0.030
M_BUMP = 0.010                   # kg each, rod + mount (estimate, not weighed)


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


SHELL_R = 0.016  # corner radius of the rounded box shell

# CAD meshes for looks only (group 2). The simple shapes do the physics (group 3, hidden).
# Each STL is in its body's frame at the "stand" pose (h = 120 mm), exported from Fusion S59.
MESH_DIR = __import__('os').path.join(__import__('os').path.dirname(__import__('os').path.abspath(__file__)), "meshes")
PLA, DARK, PCB, TYRE = "0.93 0.90 0.84 1", "0.18 0.18 0.2 1", "0.15 0.45 0.25 1", "0.08 0.08 0.08 1"
BODY_MESHES = {"torso": [("torso_shell", PLA), ("torso_parts", DARK), ("torso_pi", PCB), ("yaw_servo", DARK)],
               "neck": [("neck", PLA)], "head": [("head", PLA), ("oak", DARK)]}
for _s in "LR":
    BODY_MESHES.update({f"up_f_{_s}": [(f"up_f_{_s}", PLA)], f"up_r_{_s}": [(f"up_r_{_s}", PLA)],
                        f"low_f_{_s}": [(f"low_f_{_s}", PLA)], f"low_r_{_s}": [(f"low_r_{_s}", PLA)],
                        f"axle_{_s}": [(f"axle_{_s}", DARK)], f"wheel_{_s}": [(f"wheel_{_s}", TYRE)]})


def add_meshes(xml):
    """Put a visual mesh geom in each body, right after its opening tag."""
    import os, re
    if not os.path.isdir(MESH_DIR):
        return xml
    assets = "".join(f'\n    <mesh name="{n}" file="{n}.stl" inertia="shell"/>' for v in BODY_MESHES.values() for n, _ in v)
    xml = xml.replace("  <asset>", "  <asset>" + assets, 1)
    for body, meshes in BODY_MESHES.items():
        geoms = "".join(f'\n        <geom class="visual" mesh="{n}" rgba="{c}"/>' for n, c in meshes)
        xml, k = re.subn(rf'(<body name="{body}"[^>]*>)', lambda mt: mt.group(1) + geoms, xml, count=1)
        assert k == 1, body
    return xml


def shell_geoms(shell):
    # rounded box = three slabs + capsules along the 12 edges
    a, b, c, r = TORSO_D/2, TORSO_W/2, TORSO_H/2, SHELL_R
    g = [f'<geom name="shell" type="box" size="{a-r} {b-r} {c}" pos="0 0 {HIP_DROP}" mass="0" rgba="0.93 0.90 0.84 1"/>',
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


def bump_geoms():
    if not BUMPS:
        return ""
    return "\n      ".join(
        f'<geom name="bump_{n}" type="capsule" fromto="{sx*BUMP_X} {-BUMP_HALF_Y} {BUMP_Z} {sx*BUMP_X} {BUMP_HALF_Y} {BUMP_Z}" size="{BUMP_R}" mass="0" rgba="0.9 0.3 0.2 1"/>'
        for n, sx in (("front", 1), ("back", -1)))


def torso_inertial():
    """One inertial for the torso body: the Body shell plus the fixed yaw servo."""
    m1, m2 = M_TORSO, M_YAWSERVO
    pts = [(m2, YAWSERVO_COM), (M_LID_BAND, LID_BAND_COM), (M_EXTRA, EXTRA_COM)] + ([(M_BUMP, (sx*BUMP_X, 0.0, BUMP_Z)) for sx in (1, -1)] if BUMPS else [])
    m = m1 + sum(mm for mm, _ in pts)
    c = [(m1*TORSO_COM[i] + sum(mm*p[i] for mm, p in pts))/m for i in range(3)]
    a, b, h = TORSO_D, TORSO_W + 0.0046, TORSO_H
    I = [m1/12*(b*b + h*h), m1/12*(a*a + h*h), m1/12*(a*a + b*b)]
    # parallel axis terms (both parts to the combined COM)
    for mm, p in [(m1, TORSO_COM)] + pts:
        dx, dy, dz = (p[i] - c[i] for i in range(3))
        I[0] += mm*(dy*dy + dz*dz); I[1] += mm*(dx*dx + dz*dz); I[2] += mm*(dx*dx + dy*dy)
    return (f'<inertial pos="{c[0]:.4f} {c[1]:.4f} {c[2]:.4f}" mass="{m:.4f}" '
            f'diaginertia="{I[0]:.4e} {I[1]:.4e} {I[2]:.4e}"/>')


def build(h_stand=0.120, shell="box"):
    th_f, th_r = leg_angles(h_stand)
    kb = knee_angle(h_stand)
    # knee joint angle relative to upper link: lower link bends back toward axle
    knee_f = (math.pi - kb)
    knee_r = -(math.pi - kb)

    def leg(side, sy):
        s = side
        return f"""
      <body name="up_f_{s}" pos="{HIP_DX} {sy} 0">
        <joint name="hip_f_{s}" class="sts3215" type="hinge" axis="0 1 0" range="-2.4 2.4"/>
        <joint name="hip_f_{s}_backlash" class="backlash" type="hinge" axis="0 1 0"/>
        <geom class="link" type="capsule" fromto="0 0 0 0 0 {-L1}" size="0.004" mass="{M_UP}" rgba="0.85 0.85 0.85 1"/>
        <body name="low_f_{s}" pos="0 0 {-L1}">
          <joint name="knee_f_{s}" type="hinge" axis="0 1 0" damping="0.002"/>
          <geom class="link" type="capsule" fromto="0 0 0 0 0 {-L2}" size="0.004" mass="{M_LOW_F}" rgba="0.85 0.85 0.85 1"/>
          <body name="axle_{s}" pos="0 0 {-L2}">
            <geom class="link" type="box" size="0.012 0.0225 0.0175" pos="{WHEELSERVO_POS[0]} {-WHEELSERVO_POS[1]*(1 if sy>0 else -1)} {WHEELSERVO_POS[2]}" mass="{M_WHEELSERVO}" rgba="0.2 0.2 0.25 1"/>
            <site name="axle_{s}" pos="0 0 0" size="0.003"/>
            <body name="wheel_{s}" pos="0 {WHEEL_OFFSET*(1 if sy>0 else -1)} 0">
              <joint name="wheel_{s}" type="hinge" axis="0 1 0" damping="0.0005" frictionloss="{WHEEL_FRICTIONLOSS}" armature="{WHEEL_ARMATURE}"/>
              <joint name="wheel_{s}_backlash" class="backlash" type="hinge" axis="0 1 0"/>
              <geom type="cylinder" size="{WHEEL_R} {WHEEL_HALF_W}" quat="0.7071 0.7071 0 0" mass="{M_WHEEL}" rgba="0.1 0.1 0.1 1" friction="1.0 0.005 0.0001" condim="3"/>
            </body>
          </body>
        </body>
      </body>
      <body name="up_r_{s}" pos="{-HIP_DX} {sy} 0">
        <joint name="hip_r_{s}" class="sts3215" type="hinge" axis="0 1 0" range="-2.4 2.4"/>
        <joint name="hip_r_{s}_backlash" class="backlash" type="hinge" axis="0 1 0"/>
        <geom class="link" type="capsule" fromto="0 0 0 0 0 {-L1}" size="0.004" mass="{M_UP}" rgba="0.85 0.85 0.85 1"/>
        <body name="low_r_{s}" pos="0 0 {-L1}">
          <joint name="knee_r_{s}" type="hinge" axis="0 1 0" damping="0.002"/>
          <geom class="link" type="capsule" fromto="0 0 0 0 0 {-L2}" size="0.004" mass="{M_LOW_R}" rgba="0.85 0.85 0.85 1"/>
          <site name="tip_r_{s}" pos="0 0 {-L2}" size="0.003"/>
        </body>
      </body>"""

    xml = f"""
<mujoco model="roadrunner_mini">
  <compiler angle="radian" autolimits="true" meshdir="meshes"/>
  <option timestep="0.001" gravity="0 0 -9.81" integrator="implicitfast"/>
  <default>
    <geom contype="1" conaffinity="1" friction="0.9 0.005 0.0001" group="3"/>
    <default class="link"><geom contype="2" conaffinity="0"/></default>
    <default class="sts3215">
      <joint damping="{STS_DAMPING}" frictionloss="{STS_FRICTIONLOSS}" armature="{STS_ARMATURE}"/>
      <position kp="{STS_KP}" kv="0" forcerange="-{STS_TORQUE} {STS_TORQUE}" timeconst="{STS_TIMECONST}"/>
    </default>
    <!-- gear slop: a tiny free joint that can wobble +/-0.5 deg behind the servo -->
    <default class="backlash"><joint damping="0.01" frictionloss="0" armature="0.0001" limited="true" range="-{BACKLASH} {BACKLASH}"/></default>
    <default class="visual"><geom type="mesh" contype="0" conaffinity="0" group="2" mass="0"/></default>
    <equality solref="0.004 1"/>
  </default>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.9 0.93 0.97" rgb2="1 1 1" width="64" height="64"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.82 0.82 0.82" rgb2="0.9 0.9 0.9" width="256" height="256"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.05"/>
  </asset>
  <worldbody>
    <light pos="0.5 -1 2" dir="-0.3 0.5 -1" diffuse="0.9 0.9 0.9"/>
    <light pos="-1 1 1.5" dir="0.5 -0.5 -1" diffuse="0.4 0.4 0.4"/>
    <geom name="floor" type="plane" size="3 3 0.1" material="grid" contype="1" conaffinity="3" group="0" friction="0.5 0.005 0.0001"/>  <!-- low on purpose: MuJoCo uses the larger of the two, so the tyre sets the grip -->
    <body name="torso" pos="0 0 {WHEEL_R + h_stand}">
      <freejoint name="root"/>
      <site name="imu" pos="0 0 0.02" size="0.004"/>
      <!-- torso: rounded box, hips at its bottom; one inertial for the Body and the yaw servo -->
      {torso_inertial()}
      {shell_geoms(shell)}
      {bump_geoms()}
      <geom name="yaw_servo" type="box" size="0.0227 0.0124 0.0189" pos="-0.0085 0 {0.0988 + LID_RISE:.4f}" rgba="0.2 0.2 0.25 1"/>
      <body name="neck" pos="{YAW_AXIS[0]} {YAW_AXIS[1]} {YAW_AXIS[2]}">
        <joint name="head_yaw" class="sts3215" type="hinge" axis="0 0 1" range="{HEAD_YAW_RANGE[0]} {HEAD_YAW_RANGE[1]}"/>
        <geom name="pitch_bracket" class="link" type="box" size="0.012 0.02 0.012" pos="{BRACKET_COM[0]-YAW_AXIS[0]:.4f} {BRACKET_COM[1]-YAW_AXIS[1]:.4f} {BRACKET_COM[2]-YAW_AXIS[2]:.4f}" mass="{M_BRACKET}" rgba="0.2 0.2 0.25 1"/>
      <body name="head" pos="0 0 {HEAD_PIVOT_Z - YAW_AXIS[2]:.4f}">
        <joint name="head_pitch" class="sts3215" type="hinge" axis="0 1 0" range="{HEAD_PITCH_RANGE[0]} {HEAD_PITCH_RANGE[1]}"/>
        <inertial pos="{HEAD_COM[0]} {HEAD_COM[1]} {HEAD_COM[2]}" mass="{M_HEAD}" diaginertia="{M_HEAD/12*(HEAD_W**2+HEAD_H**2):.3e} {M_HEAD/12*(HEAD_D**2+HEAD_H**2):.3e} {M_HEAD/12*(HEAD_D**2+HEAD_W**2):.3e}"/>
        <geom type="ellipsoid" size="{HEAD_D/2} {HEAD_W/2} {HEAD_H/2}" pos="0.00175 0 0.0195" mass="0" rgba="0.93 0.90 0.84 1"/>
        <site name="cam" pos="{HEAD_D/2} 0 {HEAD_H/2}" size="0.003"/>
      </body>
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
    <position name="hip_f_L" joint="hip_f_L" class="sts3215"/>
    <position name="hip_r_L" joint="hip_r_L" class="sts3215"/>
    <position name="hip_f_R" joint="hip_f_R" class="sts3215"/>
    <position name="hip_r_R" joint="hip_r_R" class="sts3215"/>
    <position name="head_yaw" joint="head_yaw" class="sts3215"/>
    <position name="head_pitch" joint="head_pitch" class="sts3215"/>
    <!-- wheels: torque = ctrl - (HS_TORQUE/HS_VEL) * wheel speed, so no torque is left at HS_VEL -->
    <general name="wheel_L" joint="wheel_L" gainprm="1" biastype="affine" biasprm="0 0 {-HS_TORQUE/HS_VEL:.4f}" ctrlrange="-{HS_TORQUE * BATTERY_MAX:.3f} {HS_TORQUE * BATTERY_MAX:.3f}" forcerange="-{HS_TORQUE * BATTERY_MAX:.3f} {HS_TORQUE * BATTERY_MAX:.3f}"/>
    <general name="wheel_R" joint="wheel_R" gainprm="1" biastype="affine" biasprm="0 0 {-HS_TORQUE/HS_VEL:.4f}" ctrlrange="-{HS_TORQUE * BATTERY_MAX:.3f} {HS_TORQUE * BATTERY_MAX:.3f}" forcerange="-{HS_TORQUE * BATTERY_MAX:.3f} {HS_TORQUE * BATTERY_MAX:.3f}"/>
  </actuator>
  <sensor>
    <gyro name="gyro" site="imu"/>
    <accelerometer name="acc" site="imu"/>
    <framequat name="torso_quat" objtype="site" objname="imu"/>
  </sensor>
  <keyframe>
    <key name="stand" qpos="0 0 {WHEEL_R + h_stand} 1 0 0 0  0 0  {th_f} 0 {knee_f} 0 0 {th_r} 0 {knee_r}  {th_f} 0 {knee_f} 0 0 {th_r} 0 {knee_r}"/>
  </keyframe>
</mujoco>"""
    return add_meshes(xml), (th_f, th_r, knee_f, knee_r)


if __name__ == "__main__":
    h = float(sys.argv[1]) if len(sys.argv) > 1 else 0.120
    shell = sys.argv[2] if len(sys.argv) > 2 else "box"
    xml, ang = build(h, shell)
    out = __import__('os').environ.get('OUT_XML') or "roadrunner_mini.xml"
    open(out, "w").write(xml)
    print("angles", ang)
