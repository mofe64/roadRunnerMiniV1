# Simulation

This document is written in ASD-STE100 Simplified Technical English.

This folder has the MuJoCo model of the biped and the test scripts.

## Install
Install the Python packages:

```
pip install mujoco numpy
```

## Build the model
Run this command:

```
OUT_XML=eggbot_v03.xml python3 build_model_v03.py 0.120 box
```

## Tests
- `tests.py` has helper functions. Load them with `MODEL_XML=eggbot_v03.xml python3 -c "import tests"`.
- The `standup_v03*.py` scripts are studies of the get-up movement.
