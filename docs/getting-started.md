# Getting Started

This guide takes you from an assembled BugBot to running your first Python program in about ten minutes.

!!! note "Don't have a built robot yet?"
    If you are starting from a kit or from a bare board, go to the [Build guide](build/index.md) first, then come back here.

---

## What you need

- An assembled BugBot robot
- A BugBot USB dongle
- A PC with **Python 3.9 or later**
- A USB-A or USB-C cable (for the one-time robot setup step)

---

## Step 1 — Set up the project

**1. Download the BugBot project files** and unzip them into a folder on your PC. Everything you need is included — no pip or internet connection required. You will get a structure like this:

```
bugbot-project/
├── bugbot/          ← the BugBot library
├── lib/             ← bundled dependencies (pyserial and others)
├── data/            ← stores your robot roster
└── scripts/         ← setup and utility scripts
```

**2. Write your own Python scripts inside that same folder**, alongside the `bugbot/` directory. The library automatically finds everything inside `lib/` — you do not need to install or configure anything.

```
bugbot-project/
├── bugbot/
├── lib/
├── data/
├── scripts/
└── my_script.py     ← your code goes here
```

---

## Step 2 — Provision the robot (one time per robot)

Before the dongle can talk to a robot wirelessly, the robot needs to be paired to your PC. You only do this once per robot (or whenever you need to re-pair it to a new PC).

1. **Plug the robot in over USB** (the robot's USB port, not the dongle).
2. Open a terminal in the `Software/` folder and run:

```
python scripts/bugbot_init.py
```

The script:

- Auto-detects the robot's serial port.
- Sends a provisioning command to the robot.
- Reads back the device ID and pairing key.
- Saves the robot to `Software/data/robots.json`.
- The robot then reboots into wireless (ESP-NOW) mode.

You should see something like:

```
Provisioned. Added 'A2C594' (id=ea956cc2ccf261d5, mac=aabbccddeeff, ch=1, fw=4).
Roster: Software/data/robots.json
The robot is rebooting into ESP-NOW mode.
```

**Give the robot a name** using `--name`:

```
python scripts/bugbot_init.py --name FROG-14
```

**Run it again on the same robot** to update it (no duplicates are created).

!!! note
    The robot must be plugged in via USB for this step. After provisioning, unplug it and power it from battery — it will stay in wireless mode.

---

## Step 3 — Plug in the dongle

Unplug the robot from USB. Plug the **dongle** into a USB port on your PC. No driver installation is needed on most systems — it appears as a serial device automatically.

---

## Step 4 — Your first program

Create a new file called `my_first_bugbot.py`:

```python
import bugbot; bugbot.go()   # always the first line — connects to the robot

led("green")       # turn the LED green
beep(440, 300)     # beep at 440 Hz for 300 ms
forward(50)        # drive forward at 50% speed
wait(1.5)          # drive for 1.5 seconds
stop()
led("off")
print("done")
```

Run it:

```
python my_first_bugbot.py
```

The robot should light up green, beep, roll forward for 1.5 seconds, then stop.

!!! tip "How it works"
    `bugbot.go()` connects to the robot and injects all the control functions
    (`forward`, `stop`, `distance`, etc.) directly into your script — no `bot.`
    prefix needed. The script runs on your PC and sends each command to the robot
    wirelessly as you go.

### Advanced: Robot class

If you need more control (multiple robots, explicit connect/disconnect timing),
use the `Robot` class directly:

```python
from bugbot import Robot

with Robot(0) as bot:
    bot.led("green")
    bot.forward(50)
    import time; time.sleep(1.5)
    bot.stop()
    bot.led("off")
```

---

## Multiple robots

If you have more than one robot in your roster, use the index to select which one to connect to:

```python
bot0 = Robot(0)   # first robot in robots.json
bot1 = Robot(1)   # second robot
```

To use both at the same time on a single dongle, share the dongle explicitly:

```python
from bugbot import Robot, Dongle

with Dongle() as dongle:
    bot0 = Robot(0, transport=dongle)
    bot1 = Robot(1, transport=dongle)

    bot0.forward(50)
    bot1.forward(50)
    # ...
    bot0.disconnect()
    bot1.disconnect()
```

---

## Troubleshooting

**"No BugBot dongle found"**
The dongle is not plugged in, or the USB driver has not loaded. On Windows, check Device Manager for an Espressif device under *Ports (COM & LPT)*.

**"Robot index 0 is out of range — roster has 0 robot(s)"**
You have not provisioned any robots yet. Run `python scripts/bugbot_init.py` with the robot plugged in over USB.

**"Did not respond (off or out of range)"**
The robot is powered off, too far from the dongle (more than a few metres), or on a different Wi-Fi channel. Power the robot on and move it closer.

**"Rejected our key (it may have been re-paired)"**
The robot was paired to a different PC. Plug the robot in over USB and run `bugbot_init.py` again to re-pair it to this PC. Use `--regenerate` if the robot is already in your roster:

```
python scripts/bugbot_init.py --regenerate
```

---

## What's next

- [Lessons](lessons/index.md) — self-guided coding activities to learn the robot API step by step
- [Hardware overview](hardware/index.md) — components, pin reference, I2C addresses
- [Python Library](python-api/index.md) — full API reference for `Robot`, `Dongle`, and exceptions
