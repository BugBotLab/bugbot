# Robot

`Robot` represents a single connected BugBot robot. Each instance manages one
wireless session: it runs a background heartbeat every 2 s and sends RELEASE to
the robot when the session ends.

```python
from bugbot import Robot
```

---

## Constructor

```python
class Robot(
    robot=None,
    port=None,
    transport=None,
    connect=True,
)
```

Create and (by default) immediately connect to a robot.

**Parameters**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `robot` | `int` | — | Zero-based roster index. `0` = first robot in `robots.json`, `1` = second, etc. |
| `robot` | `str` | — | 16-character hex device ID. Explicit alternative to an index. |
| `robot` | `None` | `None` | Auto-connects when the roster has exactly one robot; otherwise opens a picker dialog. |
| `port` | `str \| None` | `None` | Serial port of the dongle (e.g. `"COM4"`, `"/dev/ttyACM0"`). Auto-detected when `None`. |
| `transport` | `Dongle \| None` | `None` | A pre-opened [`Dongle`](dongle.md) to share across multiple `Robot` instances. When supplied, this instance does not open or close the serial port. |
| `connect` | `bool` | `True` | Call `connect()` immediately. Pass `False` to defer the handshake. |

**Raises**

| Exception | Condition |
|-----------|-----------|
| [`RobotConnectError`](exceptions.md#robotconnecterror) | Index out of range, device not in roster, no PROBE response within 2 s, or pairing key rejected. |
| [`RobotBusyError`](exceptions.md#robotbusyerror) | Another host already holds the session lease. |
| `RuntimeError` | No dongle found and `port` was not specified. |

```python
bot = Robot(0)        # first robot in robots.json
bot = Robot(1)        # second robot
bot = Robot()         # auto-pick (picker shown if more than one)
```

### Context manager

`Robot` implements `__enter__` / `__exit__`. `__exit__` calls
[`disconnect()`](#disconnect) unconditionally, even if an exception was raised
inside the block.

```python
with Robot(0) as bot:
    bot.forward(50)
# disconnect() is called here automatically
```

---

## Connection

### `connect`

```python
connect() → Robot
```

Execute the PROBE → CLAIM handshake and start the background heartbeat thread.
Returns `self`. Called automatically by the constructor unless `connect=False`
was passed.

**Raises**

| Exception | Condition |
|-----------|-----------|
| [`RobotConnectError`](exceptions.md#robotconnecterror) | Robot did not respond within 2 s, not in roster, or key rejected. |
| [`RobotBusyError`](exceptions.md#robotbusyerror) | Robot already claimed by another host. |

---

### `disconnect`

```python
disconnect() → None
```

Send RELEASE to the robot, stop the heartbeat thread, and close the serial port.
If a shared [`Dongle`](dongle.md) was supplied at construction, only this robot's
receive callback is deregistered — the serial port remains open.

Safe to call more than once. Also triggered automatically on process exit and by
the context manager.

---

## Motion

All `speed` parameters are **0–100 %** of full motor power. Values outside this range are clamped silently.

---

### `forward`

```python
forward(speed: float, distance: float | None = None) → None
```

Drive forward.

| Parameter | Type | Description |
|-----------|------|-------------|
| `speed` | `float` | Motor power, 0–100 %. |
| `distance` | `float \| None` | Distance in centimetres. When supplied, **blocks** for the estimated travel time then calls `stop()`. When `None`, returns immediately. |

```python
bot.forward(50)                  # non-blocking
bot.forward(50, distance=30)     # blocking — drives ~30 cm then stops
```

---

### `backward`

```python
backward(speed: float, distance: float | None = None) → None
```

Drive backward. Identical semantics to `forward`.

---

### `left`

```python
left(speed: float) → None
```

Spin counter-clockwise in place. Non-blocking.

---

### `right`

```python
right(speed: float) → None
```

Spin clockwise in place. Non-blocking.

---

### `turn`

```python
turn(deg: float) → None
```

Spin in place by a fixed angle. **Blocking.**

Sends a `left` or `right` command at a fixed internal speed (~40 %), sleeps for
an estimated duration based on `abs(deg) / 90 deg/s`, then calls `stop()`.

| Parameter | Type | Description |
|-----------|------|-------------|
| `deg` | `float` | Degrees to rotate. Positive = **clockwise** (right). Negative = counter-clockwise (left). |

```python
bot.turn(90)     # quarter-turn clockwise
bot.turn(-90)    # quarter-turn counter-clockwise
bot.turn(180)    # U-turn
```

!!! note "Upload mode"
    When the script runs directly on the robot (`bugbot.go(upload=True)`), `turn()` takes an optional second argument `turn(degrees, speed=30)` and uses the IMU for closed-loop control instead of a fixed sleep. The sign convention is the same: positive = clockwise.

---

### `drive`

```python
drive(left: float, right: float) → None
```

Tank drive. Sets left and right wheel groups independently. Non-blocking.

| Parameter | Type | Description |
|-----------|------|-------------|
| `left` | `float` | Left wheel power, −100 to 100. Negative drives that side backward. |
| `right` | `float` | Right wheel power, −100 to 100. |

```python
bot.drive(50, 50)    # straight forward
bot.drive(-50, 50)   # spin left
bot.drive(50, 25)    # curve right
```

---

### `stop`

```python
stop() → None
```

Stop all motors immediately. Non-blocking.

---

## Output

### `led`

```python
led(r: int | str, g: int | None = None, b: int | None = None) → None
```

Set the RGB LED. Accepts a color name string **or** explicit R, G, B integer values.

**Color name form** — pass a single string:

| Name | R | G | B |
|------|---|---|---|
| `"red"` | 255 | 0 | 0 |
| `"green"` | 0 | 255 | 0 |
| `"blue"` | 0 | 0 | 255 |
| `"yellow"` | 255 | 255 | 0 |
| `"cyan"` | 0 | 255 | 255 |
| `"magenta"` | 255 | 0 | 255 |
| `"white"` | 255 | 255 | 255 |
| `"orange"` | 255 | 128 | 0 |
| `"purple"` | 128 | 0 | 128 |
| `"pink"` | 255 | 105 | 180 |
| `"off"` / `"black"` | 0 | 0 | 0 |

**RGB form** — pass three integers (0–255 each, clamped):

```python
bot.led("orange")        # by name
bot.led("off")           # turn LED off
bot.led(255, 128, 0)     # raw RGB — same as "orange"
```

**Raises:** `ValueError` — `r` is a string not in the color name table.

---

### `beep`

```python
beep(freq: int, ms: int = 200) → None
```

Sound the piezo buzzer.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `freq` | `int` | — | Frequency in Hz. Pass `0` to silence immediately (non-blocking). |
| `ms` | `int` | `200` | Duration in milliseconds. When `ms > 0`, the call **blocks** for `ms / 1000` s then silences the buzzer. |

```python
bot.beep(440)        # A4 for 200 ms
bot.beep(880, 500)   # A5 for 500 ms
bot.beep(0)          # silence immediately (non-blocking)
```

---

### `servo`

```python
servo(index: int, angle: float) → None
```

Command a servo to a target angle. Non-blocking.

| Parameter | Type | Description |
|-----------|------|-------------|
| `index` | `int` | Servo channel: `0`, `1`, or `2`. Maps to firmware pins 7, 8, and 9 respectively. |
| `angle` | `float` | Target position in degrees. Clamped to the servo's physical travel by the firmware. |

```python
bot.servo(0, 90)    # center servo 0
bot.servo(1, 0)     # servo 1, full one way
bot.servo(2, 180)   # servo 2, full other way
```

---

## Sensors

### `distance`

```python
distance() → float
```

Read the front distance sensor. **Blocking** (up to 2 s).

**Returns:** Distance in centimetres. Maximum reliable range ~400 cm.

**Raises:** `TimeoutError` — no response within 2 s.

---

### `heading`

```python
heading() → float
```

Read the current heading from the IMU. **Blocking** (up to 2 s).

**Returns:** Heading in degrees, 0.0–360.0. 0° = the direction the robot faced when odometry was last reset. Increases clockwise.

**Raises:** `TimeoutError` — no response within 2 s.

---

### `position`

```python
position() → tuple[float, float]
```

Read the estimated (x, y) position from IMU odometry. **Blocking** (up to 2 s).

**Returns:** `(x, y)` in centimetres. Origin is the robot's boot position. Positive x = right, positive y = forward.

**Raises:** `TimeoutError` — no response within 2 s.

---

### `battery`

```python
battery() → int
```

Read the battery level. **Blocking** (up to 2 s).

**Returns:** Battery percentage 0–100.

**Raises:** `TimeoutError` — no response within 2 s.

---

## Module-level `connect` { #connect-fn }

```python
connect(robot=None, port=None) → Robot
```

Convenience function. Equivalent to `Robot(robot, port=port)`.

```python
from bugbot import connect

bot = connect(0)
bot.forward(50)
bot.disconnect()
```

---

## Motion deadman

The robot firmware has a **500 ms motion watchdog** on the command path. If no
drive command arrives within 500 ms, the firmware stops all motors automatically.
For continuous free-running motion, re-send the command more frequently than
that:

```python
import time
deadline = time.time() + 5.0
while time.time() < deadline:
    bot.forward(60)
    time.sleep(0.15)   # re-send every 150 ms
bot.stop()
```

Blocking calls (`forward(speed, distance=N)`, `backward(...)`, `turn(deg)`)
handle the deadman internally.
