# Python Library

The `bugbot` library provides a Python interface for controlling BugBot robots
wirelessly. Your PC talks to a **BugBot USB dongle** over serial; the dongle
relays commands to the robot over ESP-NOW.

Authentication and session management are handled internally.

## Setup

Download the BugBot project files and unzip them into a folder on your PC. All
dependencies are bundled in the `lib/` subfolder — no pip or internet required.
See the [Getting Started](../getting-started.md) guide for the full walkthrough.

## Add a robot (once per robot)

Plug the robot in over USB, then run:

```
python scripts/bugbot_init.py
```

This saves the robot's identity and pairing key to `data/robots.json`.
Re-running the same robot updates its entry without creating duplicates.

## Two ways to use the library

### Student scripting with `bugbot.go()` (recommended for lessons)

Put `import bugbot; bugbot.go()` as the **first line** of your script. This
injects all the robot control functions (`forward`, `stop`, `distance`, etc.)
directly into your script's global namespace — no `bot.` prefix needed.

```python
import bugbot; bugbot.go()

forward(50)        # drive forward at 50% speed
wait(2)            # wait 2 seconds
stop()

print("distance:", distance(), "cm")
```

By default the script runs **on your PC** and sends commands to the robot
wirelessly (proxy mode). To run the script **directly on the robot** (upload
mode — zero wireless latency for sensor reads), pass `upload=True`:

```python
import bugbot; bugbot.go(upload=True)

turn(360, 10)      # IMU-closed-loop spin using the robot's own IMU
print("done!")
```

### Advanced usage with `Robot` class

For multi-robot setups or more control over the connection lifecycle, import the
`Robot` class directly:

```python
from bugbot import Robot

with Robot(0) as bot:
    bot.led("green")
    bot.forward(50, distance=30)   # drive 30 cm, then stop
    bot.turn(90)                   # turn 90° clockwise
    print("distance:", bot.distance(), "cm")
    print("battery: ", bot.battery(), "%")
```

## Classes and functions

| Name | Description |
|------|-------------|
| `go(upload=False)` | Start of every student script. Injects robot functions into globals. |
| [`Robot`](robot.md) | Advanced class — connects to a robot and exposes all control methods. |
| [`Dongle`](dongle.md) | USB dongle handle — only needed when sharing one dongle across multiple robots. |
| [`connect()`](robot.md#connect-fn) | Module-level shorthand for `Robot(robot, port=port)`. |
| [`RobotConnectError`](exceptions.md#robotconnecterror) | Raised when the connection handshake fails. |
| [`RobotBusyError`](exceptions.md#robotbusyerror) | Raised when the robot is already claimed by another host. |

## Functions available after `bugbot.go()`

These are injected into your script's globals by `go()`. They work identically
in proxy mode (PC-side) and upload mode (on the robot), with one exception:
`turn()` in upload mode uses the IMU for closed-loop accuracy.

| Function | Default speed | Description |
|----------|---------------|-------------|
| `forward(speed=50)` | 50 % | Drive forward. Non-blocking. |
| `backward(speed=50)` | 50 % | Drive backward. Non-blocking. |
| `left(speed=50)` | 50 % | Spin counter-clockwise. Non-blocking. |
| `right(speed=50)` | 50 % | Spin clockwise. Non-blocking. |
| `stop()` | — | Stop all motors immediately. |
| `turn(degrees, speed=30)` | 30 % | Spin by an angle. **Blocking.** Positive = clockwise. |
| `wait(seconds)` | — | Pause the script for this many seconds. |
| `distance()` | — | Front distance in cm (LiDAR). **Blocking.** |
| `heading()` | — | IMU heading in degrees 0–360. **Blocking.** |
| `position()` | — | `(x, y)` position in cm from odometry. **Blocking.** |
| `battery()` | — | Battery level 0–100 %. **Blocking.** |
| `led(r, g, b)` or `led("red")` | — | Set the RGB LED. |
| `beep(freq, ms=200)` | — | Sound the buzzer. **Blocking** when ms > 0. |
| `servo(index, deg)` | — | Move a servo to a target angle. |
