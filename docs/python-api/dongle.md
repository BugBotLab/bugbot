# Dongle

`Dongle` is a handle to the USB dongle's serial link. It is only needed when
sharing one dongle across multiple [`Robot`](robot.md) instances. A `Robot`
created without a `transport` argument manages its own `Dongle` internally.

```python
from bugbot import Dongle
```

---

## Constructor

```python
class Dongle(port=None, baud=921600)
```

**Parameters**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | `str \| None` | `None` | Serial device path (e.g. `"COM3"`, `"/dev/ttyACM0"`). When `None`, scans all connected ports for an Espressif USB VID (`0x303A`) and uses the first match. |
| `baud` | `int` | `921600` | Baud rate. Must match the dongle firmware. |

**Raises:** `RuntimeError` — No dongle found and `port` was not specified.

---

## Methods

### `open`

```python
open() → Dongle
```

Open the serial port and start the background receive thread. Returns `self`.
Called automatically by `__enter__`.

---

### `close`

```python
close() → None
```

Stop the receive thread and close the serial port. Called automatically by
`__exit__`.

---

### `add_recv_callback`

```python
add_recv_callback(fn: Callable[[bytes, int, bytes], None]) → None
```

Register a callback invoked for every ESP-NOW packet received from the dongle.
Multiple callbacks may be registered; each is called in registration order.
Adding the same callable a second time is a no-op.

**Parameters**

| Parameter | Type | Description |
|-----------|------|-------------|
| `fn` | `Callable[[bytes, int, bytes], None]` | Signature: `fn(packet, rssi, mac)`. `rssi` is signal strength in dBm (negative integer). `mac` is the 6-byte sender MAC address. |

---

### `remove_recv_callback`

```python
remove_recv_callback(fn: Callable[[bytes, int, bytes], None]) → None
```

Deregister a callback previously added with
[`add_recv_callback()`](#add_recv_callback). Silently ignores `fn` if it is not
currently registered.

---

## Context manager

`Dongle` implements `__enter__` / `__exit__`. `__enter__` calls [`open()`](#open)
and returns `self`; `__exit__` calls [`close()`](#close) unconditionally.

---

## Multiple robots example

One dongle communicates with all robots on the same ESP-NOW channel. Pass the
same `Dongle` to each `Robot` — each robot registers its own receive callback and
filters packets by device ID.

```python
from bugbot import Robot, Dongle

with Dongle() as dongle:
    bot0 = Robot(0, transport=dongle)
    bot1 = Robot(1, transport=dongle)

    bot0.led("red")
    bot1.led("blue")

    bot0.forward(50)
    bot1.forward(50)

    bot0.disconnect()
    bot1.disconnect()
# serial port closes here
```
