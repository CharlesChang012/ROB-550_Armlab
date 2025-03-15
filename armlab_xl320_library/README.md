This is a python library for XL320 servo using DynamixelSDK. This is for **Armlab in ROB 550** use and is different from mbot_xl320_library.

## Install
```bash
$ ./install.sh
```

## Uninstall
```bash
$ sudo python3 -m pip uninstall -y armlab_xl320_library dynamixel_sdk
```
## Examples
Example `rotate_full_range.py`: 

Run the following:
```bash
$ cd examples/
$ sudo python3 rotate_full_range.py
```

---

Python Auto Formatter [Black](https://github.com/psf/black) is used in this project.

```bash
$ black -l 100 servo.py
```