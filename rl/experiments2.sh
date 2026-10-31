#!/bin/bash
# Round 2: which part of the SERVO MODEL makes balancing hard?
# Delay, noise and random robots are OFF in all of these. Each run keeps ONE servo part ON.
# Each experiment trains a new brain from zero for 4M decisions (about 8 minutes on the Mac).
# Run from the rl folder:  bash experiments2.sh
cd "$(dirname "$0")"
PY=.venv/bin/python
STEPS=${STEPS:-4000000}
REAL=noise,delay,random
run() {   # name, then the parts to turn off
    name=$1; off=$2
    echo "=== $name (turned off: $off) ==="
    rm -rf runs/exp_$name
    ABLATE=$off RUN=exp_$name TOTAL_STEPS=$STEPS $PY train.py > runs/exp_$name.log 2>&1
    echo "    reached level: $(cat runs/exp_$name/level.txt)"
}
run only_hips      "$REAL,wheelgear,backlash,hipspeed"
run only_wheelgear "$REAL,hips,backlash,hipspeed"
run only_backlash  "$REAL,hips,wheelgear,hipspeed"
run only_hipspeed  "$REAL,hips,wheelgear,backlash"
echo
echo "SUMMARY (level reached after $STEPS decisions; 1.00 = hardest game passed)"
for d in runs/exp_*/; do printf "  %-20s %s\n" "$(basename $d)" "$(cat $d/level.txt)"; done
