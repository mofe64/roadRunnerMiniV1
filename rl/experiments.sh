#!/bin/bash
# Finds which part of the realism makes balancing hard.
# Each experiment trains a new brain from zero for 4M decisions (about 8 minutes on the Mac),
# then the summary shows the level each one reached (100 % = the hardest game).
# Run from the rl folder:  bash experiments.sh
cd "$(dirname "$0")"
PY=.venv/bin/python
STEPS=${STEPS:-4000000}
run() {   # name, then the parts to turn off
    name=$1; off=$2
    echo "=== $name (turned off: ${off:-nothing}) ==="
    rm -rf runs/exp_$name
    ABLATE=$off RUN=exp_$name TOTAL_STEPS=$STEPS $PY train.py > runs/exp_$name.log 2>&1
    echo "    reached level: $(cat runs/exp_$name/level.txt)"
}
run full ""
run servo_only "noise,delay,random"
run servo_random "noise,delay"
run no_noise "noise"
run no_random "random"
run no_delay "delay"
echo
echo "SUMMARY (level reached after $STEPS decisions; 1.00 = hardest game passed)"
for d in runs/exp_*/; do printf "  %-20s %s\n" "$(basename $d)" "$(cat $d/level.txt)"; done
