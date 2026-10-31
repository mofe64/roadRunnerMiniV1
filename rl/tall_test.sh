#!/bin/bash
# Does standing taller make balancing easier? (A taller broom tips more slowly.)
# Builds the robot standing at 135 mm and 140 mm (normal: 120 mm), trains a new brain for each
# (8M decisions, about 15 minutes each on the Mac), then prints each report card.
# Run from the rl folder:  bash tall_test.sh
cd "$(dirname "$0")"
PY=$PWD/.venv/bin/python
for h in 135 140; do
    echo "=== standing height $h mm ==="
    (cd ../sim && OUT_XML=roadrunner_mini_h$h.xml $PY build_model_v03.py 0.$h > /dev/null)
    rm -rf runs/tall_$h
    MODEL_XML=../sim/roadrunner_mini_h$h.xml RUN=tall_$h TOTAL_STEPS=8000000 $PY train.py > runs/tall_$h.log 2>&1
    echo "    reached level: $(cat runs/tall_$h/level.txt)"
    MODEL_XML=../sim/roadrunner_mini_h$h.xml RUN=tall_$h $PY train.py --test | grep -E "random starts|leaning 12|shove 0.3|shove 0.5|shove 1.0"
done
