# Exceptions

```python
from bugbot import RobotConnectError, RobotBusyError
```

---

## `RobotConnectError`

```python
class RobotConnectError(RuntimeError)
```

Raised when the connection handshake fails. Subclass of `RuntimeError`.

Conditions that trigger this exception:

- Requested roster index is out of range.
- Device ID string is not present in `robots.json`.
- Robot did not respond to the PROBE packet within 2 seconds (powered off, out of range, or on a different ESP-NOW channel).
- Robot rejected the pairing key — it has been re-paired to a different host. Run `bugbot_init.py` again to re-pair.
- CLAIM packet timed out.

---

## `RobotBusyError`

```python
class RobotBusyError(RuntimeError)
```

Raised when the robot returns `CLAIM_ACK(DENIED)`, meaning another host currently
holds the session lease. Subclass of `RuntimeError`.

---

## Handling exceptions

```python
from bugbot import Robot, RobotConnectError, RobotBusyError

try:
    bot = Robot(0)
except RobotBusyError:
    print("Robot is already in use by another host.")
except RobotConnectError as e:
    print("Could not connect:", e)
```

---

## Other exceptions

These are standard Python exceptions raised by the library under specific conditions.

| Exception | Raised by | Condition |
|-----------|-----------|-----------|
| `RuntimeError` | Any command method | Called before [`connect()`](robot.md#connect). |
| `RuntimeError` | [`Robot()`](robot.md#constructor) | No dongle found and `port` was not specified. |
| `TimeoutError` | Any sensor method | No response from robot within 2 seconds. |
| `ValueError` | [`led()`](robot.md#led) | String passed is not a recognized color name. |
