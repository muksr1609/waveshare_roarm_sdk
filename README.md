# Waveshare Roarm SDK

![Python 2.7](https://img.shields.io/badge/Python-v2.7%5E-green?logo=python)
![Python 3](https://img.shields.io/badge/Python-v3%5E-green?logo=python)

This is a python API for serial or http communication with waveshare roarm and controlling it.

| Series | Models | Image |
|--------|--------|-------|
| RoArm-M2 | [**RoArm-M2-S**](https://www.waveshare.com/roarm-m2-s.htm), [**RoArm-M2-Pro**](https://www.waveshare.com/roarm-m2-s.htm) | <a href="https://www.waveshare.com/roarm-m2-s.htm"><img src="./images/roarm_m2.png" alt="RoArm-M2" width="300" height="200"/></a> |
| RoArm-M2 | [**RoArm-M2-GA**](https://www.waveshare.com/roarm-m2-ga.htm) | <a href="https://www.waveshare.com/roarm-m2-ga.htm"><img src="./images/roarm_m2_ga.jpg" alt="RoArm-M2-GA" width="300" height="200"/></a> |
| RoArm-M3 | [**RoArm-M3-S**](https://www.waveshare.com/roarm-m3.htm), [**RoArm-M3-Pro**](https://www.waveshare.com/roarm-m3.htm) | <a href="https://www.waveshare.com/roarm-m3.htm"><img src="./images/roarm_m3.png" alt="RoArm-M3" width="300" height="200"/></a> |

Model mapping:

- **RoArm-M2-S / RoArm-M2-Pro**: `roarm_type="roarm_m2"`, `gripper_type="angular_direct"` (default)
- **RoArm-M2-GA**: `roarm_type="roarm_m2"`, `gripper_type="angular_gear"`
- **RoArm-M3-S / RoArm-M3-Pro**: `roarm_type="roarm_m3"` (single gripper type; `gripper_type` usually omitted)

## Installation
### Pip install

```bash
pip install roarm-sdk==0.1.1
```

### Source code

```bash
git clone https://github.com/waveshareteam/waveshare_roarm_sdk.git <your-path>
cd <your-path>/waveshare_roarm_sdk
# Install
[sudo] python2 setup.py install
# or
[sudo] python3 setup.py install
```

## Usage:

```python
from roarm_sdk.roarm import roarm

# for roarm_m2 Serial communication example
roarm = roarm(roarm_type="roarm_m2", port="/dev/ttyUSB0", baudrate=115200)

# for roarm_m3 Serial communication example
#roarm = roarm(roarm_type="roarm_m3", port="/dev/ttyUSB0", baudrate=115200)

# Optional: gripper type and debug
# gripper_type: "angular_direct" (default, invert gripper) or "angular_gear" (pass-through)
# debug: False (default, no print) or True (print TX/RX data)
# RoArm-M2-GA example:
#roarm = roarm(roarm_type="roarm_m2", port="/dev/ttyUSB0", baudrate=115200, gripper_type="angular_gear", debug=True)
```

Constructor notes:

- **gripper_type** – `"angular_direct"` (default) or `"angular_gear"`. Default keeps legacy invert behavior; `angular_gear` sends/reads gripper values without invert. Omit it for backward compatibility.
- **RoArm-M2-GA** – use `roarm_type="roarm_m2"` with `gripper_type="angular_gear"`.
- **debug** – `False` (default) or `True`. When `True`, print serial/HTTP payload data.
- **host** – HTTP mode is **control-only** (send commands). Use serial for feedback reads and drag teach.

The [`demo`](./demo) directory stores some test case files.

You can find out which interfaces roarm_sdk provides in [`./doc/README.md`](./doc/README.md).

![jaywcjlove/sb](https://jaywcjlove.github.io/sb/lang/chinese.svg)   ![jaywcjlove/sb](https://jaywcjlove.github.io/sb/lang/english.svg)

[roarm_m2 api 说明](./doc/roarm_m2_zh.md) | [roarm_m2 api Description](./doc/roarm_m2_en.md)

[roarm_m3 api 说明](./doc/roarm_m3_zh.md) | [roarm_m3 api Description](./doc/roarm_m3_en.md)